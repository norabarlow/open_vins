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
#ifndef OV_EVAL_ALIGNUTILS_H
#define OV_EVAL_ALIGNUTILS_H


#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>

#include "utils/Math.h"
#include "utils/Colors.h"

#include "types.h"

namespace ov_eval {


    /**
     * @brief Helper functions for the trajectory alignment class.
     *
     * The key function is an implementation of Umeyama's
     * [Least-squares estimation of transformation parameters between two point patterns](https://ieeexplore.ieee.org/document/88573).
     * This is what allows us to find the transform between the two
     * trajectories without worrying about singularities for the absolute trajectory error.
     */
    class AlignUtils {

    public:

        /**
         * @brief Gets best yaw from Frobenius problem.
         * Equation (17)-(18) in [Zhang et al. 2018 IROS](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf) paper.
         * @param C Data matrix
         */
        static inline f_ekf get_best_yaw(const Eigen::Matrix<f_ekf, 3, 3> &C) {
            f_ekf A = C(0, 1) - C(1, 0);
            f_ekf B = C(0, 0) + C(1, 1);
            //return M_PI_2 - atan2(B, A);
            return atan2(A, B);
        }

        /**
         * @brief Gets mean of the vector of data
         * @param data Vector of data
         * @return Mean value
         */
        static inline Eigen::Matrix<f_ekf, 3, 1> get_mean(const std::vector<Eigen::Matrix<f_ekf, 3, 1>> &data) {
            Eigen::Matrix<f_ekf, 3, 1> mean = Eigen::Matrix<f_ekf, 3, 1>::Zero();
            for (size_t i = 0; i < data.size(); i++) {
                mean.noalias() += data[i];
            }
            mean /= f_ekf(data.size());
            return mean;
        }

        /**
         * @brief Given a set of points in a model frame and a set of points in a data frame,
         * finds best transform between frames (subject to constraints).
         *
         * @param data Vector of points in data frame (i.e., estimates)
         * @param model Vector of points in model frame (i.e., gt)
         * @param R Output rotation from data frame to model frame
         * @param t Output translation from data frame to model frame
         * @param s Output scale from data frame to model frame
         * @param known_scale Whether to fix scale
         * @param yaw_only Whether to only use yaw orientation (such as when frames are already gravity aligned)
         */
        static void align_umeyama(const std::vector<Eigen::Matrix<f_ekf, 3, 1>> &data,
                                  const std::vector<Eigen::Matrix<f_ekf, 3, 1>> &model,
                                  Eigen::Matrix<f_ekf, 3, 3> &R, Eigen::Matrix<f_ekf, 3, 1> &t,
                                  f_ekf &s, bool known_scale, bool yaw_only);

        /**
         * @brief Will intersect our loaded data so that we have common timestamps.
         * @param offset Time offset to append to our estimate
         * @param max_difference Biggest allowed difference between matched timesteps
         */
        static void perform_association(f_ekf offset, f_ekf max_difference,
                                        std::vector<f_ts> &est_times, std::vector<f_ts> &gt_times,
                                        std::vector<Eigen::Matrix<f_ekf,7,1>> &est_poses, std::vector<Eigen::Matrix<f_ekf,7,1>> &gt_poses);

        /**
         * @brief Will intersect our loaded data so that we have common timestamps.
         * @param offset Time offset to append to our estimate
         * @param max_difference Biggest allowed difference between matched timesteps
         */
        static void perform_association(f_ekf offset, f_ekf max_difference,
                                        std::vector<f_ts> &est_times, std::vector<f_ts> &gt_times,
                                        std::vector<Eigen::Matrix<f_ekf,7,1>> &est_poses, std::vector<Eigen::Matrix<f_ekf,7,1>> &gt_poses,
                                        std::vector<Eigen::Matrix<f_ekf,3,3>> &est_covori, std::vector<Eigen::Matrix<f_ekf,3,3>> &est_covpos,
                                        std::vector<Eigen::Matrix<f_ekf,3,3>> &gt_covori, std::vector<Eigen::Matrix<f_ekf,3,3>> &gt_covpos);



    };


}

#endif //OV_EVAL_ALIGNUTILS_H
