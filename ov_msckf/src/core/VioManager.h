/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H


#include <string>
#include <algorithm>
#include <fstream>
#include <memory>
#include <Eigen/StdVector>
#include <boost/filesystem.hpp>

#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
//#include "track/TrackSIM.h"
#include "init/InertialInitializer.h"
#include "types/LandmarkRepresentation.h"
#include "types/Landmark.h"
#include "utils/sensor_data.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"

#include "VioManagerOptions.h"

#include "types.h"

namespace ov_msckf {


    /**
     * @brief Core class that manages the entire system
     *
     * This class contains the state and other algorithms needed for the MSCKF to work.
     * We feed in measurements into this class and send them to their respective algorithms.
     * If we have measurements to propagate or update with, this class will call on our state to do that.
     */
    class VioManager {


    public:

        /**
         * @brief Default constructor, will load all configuration variables
         * @param params_ Parameters loaded from either ROS or CMDLINE
         */
        VioManager(VioManagerOptions &params_);

        /**
         * @brief Feed function for inertial data
         * @param message Contains our timestamp and inertial information
         */
        void feed_measurement_imu(const ov_core::ImuData &message);

        /**
         * @brief Feed function for camera measurements
         * @param message Contains our timestamp, images, and camera ids
         */
        void feed_measurement_camera(const ov_core::CameraData &message) {
            camera_queue.push_back(message);
            std::sort(camera_queue.begin(), camera_queue.end());
        }

        /**
         * @brief Feed function for a synchronized simulated cameras
         * @param timestamp Time that this image was collected
         * @param camids Camera ids that we have simulated measurements for
         * @param feats Raw uv simulated measurements
         */
        void feed_measurement_simulation(f_ts timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t, Eigen::Matrix<f_ekf,Eigen::Dynamic,1>>>> &feats);

        /**
         * @brief Given a state, this will initialize our IMU state.
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         */
        void initialize_with_gt(Eigen::Matrix<f_ekf, 17, 1> imustate) {

            // Initialize the system
            state->_imu->set_value(imustate.block(1, 0, 16, 1));
            state->_imu->set_fej(imustate.block(1, 0, 16, 1));
            state->_timestamp = imustate(0, 0);
            startup_time = imustate(0, 0);
            is_initialized_vio = true;

            // Fix the global yaw and position gauge freedoms
            StateHelper::fix_4fof_gauge_freedoms(state, imustate.block(1, 0, 4, 1));

            // Cleanup any features older then the initialization time
            trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
            if (trackARUCO != nullptr) {
                trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
            }

            // Print what we init'ed with
            printf(GREEN "[INIT]: INITIALIZED FROM GROUNDTRUTH FILE!!!!!\n" RESET);
            printf(GREEN "[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET, double(state->_imu->quat()(0)), double(state->_imu->quat()(1)), double(state->_imu->quat()(2)), double(state->_imu->quat()(3)));
            printf(GREEN "[INIT]: bias gyro = %.4f, %.4f, %.4f\n" RESET, double(state->_imu->bias_g()(0)), double(state->_imu->bias_g()(1)), double(state->_imu->bias_g()(2)));
            printf(GREEN "[INIT]: velocity = %.4f, %.4f, %.4f\n" RESET, double(state->_imu->vel()(0)), double(state->_imu->vel()(1)), double(state->_imu->vel()(2)));
            printf(GREEN "[INIT]: bias accel = %.4f, %.4f, %.4f\n" RESET, double(state->_imu->bias_a()(0)), double(state->_imu->bias_a()(1)), double(state->_imu->bias_a()(2)));
            printf(GREEN "[INIT]: position = %.4f, %.4f, %.4f\n" RESET, double(state->_imu->pos()(0)), double(state->_imu->pos()(1)), double(state->_imu->pos()(2)));

        }


        /// If we are initialized or not
        bool initialized() {
            return is_initialized_vio;
        }

        /// Timestamp that the system was initialized at
        f_ts initialized_time() {
            return startup_time;
        }

        /// Accessor for current system parameters
        VioManagerOptions get_params() {
            return params;
        }

        /// Accessor to get the current state
        std::shared_ptr<State> get_state() {
            return state;
        }

        /// Accessor to get the current propagator
        std::shared_ptr<Propagator> get_propagator() {
            return propagator;
        }

        /// Get a nice visualization image of what tracks we have
        cv::Mat get_historical_viz_image() {

            // Get our image of history tracks
            cv::Mat img_history;
            if(did_zupt_update) {
                img_history = zupt_image;
            } else {

                // Build an id-list of what features we should highlight (i.e. SLAM)
                std::vector<size_t> highlighted_ids;
                for(const auto &feat : state->_features_SLAM) {
                    highlighted_ids.push_back(feat.first);
                }

                // Get the current active tracks
                trackFEATS->display_history(img_history,255,255,0,255,255,255, highlighted_ids);
                if(trackARUCO != nullptr) {
                    trackARUCO->display_history(img_history, 0, 255, 255, 255, 255, 255);
                    trackARUCO->display_active(img_history, 0, 255, 255, 255, 255, 255);
                }

            }

            // Finally return the image
            return img_history;

        }

        /// Returns 3f features used in the last update in global frame
        std::vector<Eigen::Matrix<f_ekf,3,1>> get_good_features_MSCKF() {
            return good_features_MSCKF;
        }

        /// Returns 3f SLAM features in the global frame
        std::vector<Eigen::Matrix<f_ekf,3,1>> get_features_SLAM() {
            std::vector<Eigen::Matrix<f_ekf,3,1>> slam_feats;
            for (auto &f : state->_features_SLAM) {
                if ((int) f.first <= state->_options.max_aruco_features) continue;
                if (LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id != -1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<f_ekf, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<f_ekf, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<f_ekf, 3, 3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<f_ekf, 3, 1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    slam_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose() * (f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    slam_feats.push_back(f.second->get_xyz(false));
                }
            }
            return slam_feats;
        }

        /// Returns 3f ARUCO features in the global frame
        std::vector<Eigen::Matrix<f_ekf,3,1>> get_features_ARUCO() {
            std::vector<Eigen::Matrix<f_ekf,3,1>> aruco_feats;
            for (auto &f : state->_features_SLAM) {
                if ((int) f.first > state->_options.max_aruco_features) continue;
                if (LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id != -1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<f_ekf, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<f_ekf, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<f_ekf, 3, 3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<f_ekf, 3, 1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    aruco_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose() * (f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    aruco_feats.push_back(f.second->get_xyz(false));
                }
            }
            return aruco_feats;
        }

        /// Return the image used when projecting the active tracks
        void get_active_image(f_ts &timestamp, cv::Mat &image) {
            timestamp = active_tracks_time;
            image = active_image;
        }

        /// Returns active tracked features in the current frame
        void get_active_tracks(f_ts &timestamp,
                               std::unordered_map<size_t, Eigen::Matrix<f_ekf,3,1>> &feat_posinG,
                               std::unordered_map<size_t, Eigen::Matrix<f_ekf,3,1>> &feat_tracks_uvd) {
            timestamp = active_tracks_time;
            feat_posinG = active_tracks_posinG;
            feat_tracks_uvd = active_tracks_uvd;
        }


    protected:

        /**
         * @brief Given a new set of camera images, this will track them.
         *
         * If we are having stereo tracking, we should call stereo tracking functions.
         * Otherwise we will try to track on each of the images passed.
         *
         * @param message Contains our timestamp, images, and camera ids
         */
        void track_image_and_update(const ov_core::CameraData &message);

        /**
         * @brief This will do the propagation and feature updates to the state
         * @param message Contains our timestamp, images, and camera ids
         */
        void do_feature_propagate_update(const ov_core::CameraData &message);

        /**
         * @brief This function will try to initialize the state.
         *
         * This should call on our initializer and try to init the state.
         * In the future we should call the structure-from-motion code from here.
         * This function could also be repurposed to re-initialize the system after failure.
         *
         * @return True if we have successfully initialized
         */
        bool try_to_initialize();

        /**
         * @brief This function will will re-triangulate all features in the current frame
         *
         * For all features that are currently being tracked by the system, this will re-triangulate them.
         * This is useful for downstream applications which need the current pointcloud of points (e.g. loop closure).
         * This will try to triangulate *all* points, not just ones that have been used in the update.
         *
         * @param message Contains our timestamp, images, and camera ids
         */
        void retriangulate_active_tracks(const ov_core::CameraData &message);


        /// Manager parameters
        VioManagerOptions params;

        /// Our master state object :D
        std::shared_ptr<State> state;

        /// Propagator of our state
        std::shared_ptr<Propagator> propagator;

        /// Complete history of our feature tracks
        std::shared_ptr<FeatureDatabase> trackDATABASE;

        /// Our sparse feature tracker (klt or descriptor)
        std::shared_ptr<TrackBase> trackFEATS;

        /// Our aruoc tracker
        std::shared_ptr<TrackBase> trackARUCO;

        /// State initializer
        std::shared_ptr<InertialInitializer> initializer;

        /// Boolean if we are initialized or not
        bool is_initialized_vio = false;

        /// Our MSCKF feature updater
        std::shared_ptr<UpdaterMSCKF> updaterMSCKF;

        /// Our SLAM/ARUCO feature updater
        std::shared_ptr<UpdaterSLAM> updaterSLAM;

        /// Our zero velocity tracker
        std::shared_ptr<UpdaterZeroVelocity> updaterZUPT;

        /// Queue up camera measurements sorted by time and trigger once we have
        /// exactly one IMU measurement with timestamp newer than the camera measurement
        /// This also handles out-of-order camera measurements, which is rare, but
        /// a nice feature to have for general robustness to bad camera drivers.
        std::deque<ov_core::CameraData> camera_queue;

        // Timing statistic file and variables
        std::ofstream of_statistics;
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

        // Track how much distance we have traveled
        f_ts timelastupdate = -1;
        f_ekf distance = 0;

        // Startup time of the filter
        f_ts startup_time = -1;

        // If we did a zero velocity update
        bool did_zupt_update = false;
        cv::Mat zupt_image;
        std::map<size_t, cv::Mat> zupt_img_last;

        // Good features that where used in the last update (used in visualization)
        std::vector<Eigen::Matrix<f_ekf,3,1>> good_features_MSCKF;

        /// Feature initializer used to triangulate all active tracks
        std::shared_ptr<FeatureInitializer> active_tracks_initializer;

        // Re-triangulated features 3f positions seen from the current frame (used in visualization)
        f_ts active_tracks_time = -1;
        std::unordered_map<size_t, Eigen::Matrix<f_ekf,3,1>> active_tracks_posinG;
        std::unordered_map<size_t, Eigen::Matrix<f_ekf,3,1>> active_tracks_uvd;
        cv::Mat active_image;


    };


}


#endif //OV_MSCKF_VIOMANAGER_H
