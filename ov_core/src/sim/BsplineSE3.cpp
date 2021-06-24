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
#include "BsplineSE3.h"


using namespace ov_core;





void BsplineSE3::feed_trajectory(std::vector<Eigen::Matrix<f_ekf,Eigen::Dynamic,1>> traj_points) {


    // Find the average frequency to use as our uniform timesteps
    f_ts sumdt = 0;
    for(size_t i=0; i<traj_points.size()-1; i++) {
        sumdt += traj_points.at(i+1)(0)-traj_points.at(i)(0);
    }
    dt = sumdt/(traj_points.size()-1);
    dt = (dt < f_ts(0.05))? f_ts(0.05) : dt;
    printf("[B-SPLINE]: control point dt = %.3f (original dt of %.3f)\n",double(dt),double(sumdt)/(traj_points.size()-1));

    // convert all our trajectory points into SE(3) matrices
    // we are given [timestamp, p_IinG, q_GtoI]
    AlignedEigenMat4f trajectory_points;
    for(size_t i=0; i<traj_points.size()-1; i++) {
        Eigen::Matrix<f_ekf,4,4> T_IinG = Eigen::Matrix<f_ekf,4,4>::Identity();
        T_IinG.block(0,0,3,3) = quat_2_Rot(traj_points.at(i).block(4,0,4,1)).transpose();
        T_IinG.block(0,3,3,1) = traj_points.at(i).block(1,0,3,1);
        trajectory_points.insert({traj_points.at(i)(0),T_IinG});
    }

    // Get the oldest timestamp
    f_ts timestamp_min = INFINITY;
    f_ts timestamp_max = -INFINITY;
    for(const auto &pose : trajectory_points) {
        if(pose.first <= timestamp_min) {
            timestamp_min = pose.first;
        }
        if(pose.first >= timestamp_min) {
            timestamp_max = pose.first;
        }
    }
    printf("[B-SPLINE]: trajectory start time = %.6f\n",double(timestamp_min));
    printf("[B-SPLINE]: trajectory end time = %.6f\n",double(timestamp_max));


    // then create spline control points
    f_ts timestamp_curr = timestamp_min;
    while(true) {

        // Get bounding posed for the current time
        f_ts t0, t1;
        Eigen::Matrix<f_ekf,4,4> pose0, pose1;
        bool success = find_bounding_poses(timestamp_curr, trajectory_points, t0, pose0, t1, pose1);
        //printf("[SIM]: time curr = %.6f | lambda = %.3f | dt = %.3f | dtmeas = %.3f\n",timestamp_curr,(timestamp_curr-t0)/(t1-t0),dt,(t1-t0));

        // If we didn't find a bounding pose, then that means we are at the end of the dataset
        // Thus break out of this loop since we have created our max number of control points
        if(!success)
            break;

        // Linear interpolation and append to our control points
        f_ts lambda = (timestamp_curr-t0)/(t1-t0);
        Eigen::Matrix<f_ekf,4,4> pose_interp = exp_se3(f_ekf(lambda)*log_se3(pose1*Inv_se3(pose0)))*pose0;
        control_points.insert({timestamp_curr, pose_interp});
        timestamp_curr += dt;
        //std::cout << pose_interp(0,3) << "," << pose_interp(1,3) << "," << pose_interp(2,3) << std::endl;

    }

    // The start time of the system is two dt in since we need at least two older control points
    timestamp_start = timestamp_min + 2*dt;
    printf("[B-SPLINE]: start trajectory time of %.6f\n",double(timestamp_start));

}





bool BsplineSE3::get_pose(f_ts timestamp, Eigen::Matrix<f_ekf,3,3> &R_GtoI, Eigen::Matrix<f_ekf,3,1> &p_IinG) {

    // Get the bounding poses for the desired timestamp
    f_ts t0, t1, t2, t3;
    Eigen::Matrix<f_ekf,4,4> pose0, pose1, pose2, pose3;
    bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
    //printf("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success = %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

    // Return failure if we can't get bounding poses
    if(!success) {
        R_GtoI.setIdentity();
        p_IinG.setZero();
        return false;
    }

    // Our De Boor-Cox matrix scalars
    f_ts DT = (t2-t1);
    f_ts u = (timestamp-t1)/DT;
    f_ekf b0 = 1.0/6.0*(5+3*u-3*u*u+u*u*u);
    f_ekf b1 = 1.0/6.0*(1+3*u+3*u*u-2*u*u*u);
    f_ekf b2 = 1.0/6.0*(u*u*u);

    // Calculate interpolated poses
    Eigen::Matrix<f_ekf,4,4> A0 = exp_se3(b0*log_se3(Inv_se3(pose0)*pose1));
    Eigen::Matrix<f_ekf,4,4> A1 = exp_se3(b1*log_se3(Inv_se3(pose1)*pose2));
    Eigen::Matrix<f_ekf,4,4> A2 = exp_se3(b2*log_se3(Inv_se3(pose2)*pose3));

    // Finally get the interpolated pose
    Eigen::Matrix<f_ekf,4,4> pose_interp = pose0*A0*A1*A2;
    R_GtoI = pose_interp.block(0,0,3,3).transpose();
    p_IinG = pose_interp.block(0,3,3,1);
    return true;

}




bool BsplineSE3::get_velocity(f_ts timestamp, Eigen::Matrix<f_ekf,3,3> &R_GtoI, Eigen::Matrix<f_ekf,3,1> &p_IinG, Eigen::Matrix<f_ekf,3,1> &w_IinI, Eigen::Matrix<f_ekf,3,1> &v_IinG) {

    // Get the bounding poses for the desired timestamp
    f_ts t0, t1, t2, t3;
    Eigen::Matrix<f_ekf,4,4> pose0, pose1, pose2, pose3;
    bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
    //printf("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success = %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

    // Return failure if we can't get bounding poses
    if(!success) {
        w_IinI.setZero();
        v_IinG.setZero();
        return false;
    }

    // Our De Boor-Cox matrix scalars
    f_ts DT = (t2-t1);
    f_ts u = (timestamp-t1)/DT;
    f_ekf b0 = 1.0/6.0*(5+3*u-3*u*u+u*u*u);
    f_ekf b1 = 1.0/6.0*(1+3*u+3*u*u-2*u*u*u);
    f_ekf b2 = 1.0/6.0*(u*u*u);
    f_ekf b0dot = 1.0/(6.0*DT)*(3-6*u+3*u*u);
    f_ekf b1dot = 1.0/(6.0*DT)*(3+6*u-6*u*u);
    f_ekf b2fot = 1.0/(6.0*DT)*(3*u*u);

    // Cache some values we use alot
    Eigen::Matrix<f_ekf,6,1> omega_10 = log_se3(Inv_se3(pose0)*pose1);
    Eigen::Matrix<f_ekf,6,1> omega_21 = log_se3(Inv_se3(pose1)*pose2);
    Eigen::Matrix<f_ekf,6,1> omega_32 = log_se3(Inv_se3(pose2)*pose3);

    // Calculate interpolated poses
    Eigen::Matrix<f_ekf,4,4> A0 = exp_se3(b0*omega_10);
    Eigen::Matrix<f_ekf,4,4> A1 = exp_se3(b1*omega_21);
    Eigen::Matrix<f_ekf,4,4> A2 = exp_se3(b2*omega_32);
    Eigen::Matrix<f_ekf,4,4> A0dot = b0dot*hat_se3(omega_10)*A0;
    Eigen::Matrix<f_ekf,4,4> A1dot = b1dot*hat_se3(omega_21)*A1;
    Eigen::Matrix<f_ekf,4,4> A2fot = b2fot*hat_se3(omega_32)*A2;

    // Get the interpolated pose
    Eigen::Matrix<f_ekf,4,4> pose_interp = pose0*A0*A1*A2;
    R_GtoI = pose_interp.block(0,0,3,3).transpose();
    p_IinG = pose_interp.block(0,3,3,1);

    // Finally get the interpolated velocities
    // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
    Eigen::Matrix<f_ekf,4,4> vel_interp = pose0*(A0dot*A1*A2+A0*A1dot*A2+A0*A1*A2fot);
    w_IinI = vee(pose_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3));
    v_IinG = vel_interp.block(0,3,3,1);
    return true;

}




bool BsplineSE3::get_acceleration(f_ts timestamp, Eigen::Matrix<f_ekf,3,3> &R_GtoI, Eigen::Matrix<f_ekf,3,1> &p_IinG,
                                  Eigen::Matrix<f_ekf,3,1> &w_IinI, Eigen::Matrix<f_ekf,3,1> &v_IinG,
                                  Eigen::Matrix<f_ekf,3,1> &alpha_IinI, Eigen::Matrix<f_ekf,3,1> &a_IinG) {

    // Get the bounding poses for the desired timestamp
    f_ts t0, t1, t2, t3;
    Eigen::Matrix<f_ekf,4,4> pose0, pose1, pose2, pose3;
    bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

    // Return failure if we can't get bounding poses
    if(!success) {
        alpha_IinI.setZero();
        a_IinG.setZero();
        return false;
    }

    // Our De Boor-Cox matrix scalars
    f_ts DT = (t2-t1);
    f_ts u = (timestamp-t1)/DT;
    f_ts b0 = 1.0/6.0*(5+3*u-3*u*u+u*u*u);
    f_ts b1 = 1.0/6.0*(1+3*u+3*u*u-2*u*u*u);
    f_ts b2 = 1.0/6.0*(u*u*u);
    f_ts b0dot = 1.0/(6.0*DT)*(3-6*u+3*u*u);
    f_ts b1dot = 1.0/(6.0*DT)*(3+6*u-6*u*u);
    f_ts b2fot = 1.0/(6.0*DT)*(3*u*u);
    f_ts b0dotdot = 1.0/(6.0*DT*DT)*(-6+6*u);
    f_ts b1dotdot = 1.0/(6.0*DT*DT)*(6-12*u);
    f_ts b2fotdot = 1.0/(6.0*DT*DT)*(6*u);

    // Cache some values we use alot
    Eigen::Matrix<f_ekf,6,1> omega_10 = log_se3(Inv_se3(pose0)*pose1);
    Eigen::Matrix<f_ekf,6,1> omega_21 = log_se3(Inv_se3(pose1)*pose2);
    Eigen::Matrix<f_ekf,6,1> omega_32 = log_se3(Inv_se3(pose2)*pose3);
    Eigen::Matrix<f_ekf,4,4> omega_10_hat = hat_se3(omega_10);
    Eigen::Matrix<f_ekf,4,4> omega_21_hat = hat_se3(omega_21);
    Eigen::Matrix<f_ekf,4,4> omega_32_hat = hat_se3(omega_32);

    // Calculate interpolated poses
    Eigen::Matrix<f_ekf,4,4> A0 = exp_se3(f_ekf(b0)*omega_10);
    Eigen::Matrix<f_ekf,4,4> A1 = exp_se3(f_ekf(b1)*omega_21);
    Eigen::Matrix<f_ekf,4,4> A2 = exp_se3(f_ekf(b2)*omega_32);
    Eigen::Matrix<f_ekf,4,4> A0dot = f_ekf(b0dot)*omega_10_hat*A0;
    Eigen::Matrix<f_ekf,4,4> A1dot = f_ekf(b1dot)*omega_21_hat*A1;
    Eigen::Matrix<f_ekf,4,4> A2fot = f_ekf(b2fot)*omega_32_hat*A2;
    Eigen::Matrix<f_ekf,4,4> A0dotdot = f_ekf(b0dot)*omega_10_hat*A0dot+f_ekf(b0dotdot)*omega_10_hat*A0;
    Eigen::Matrix<f_ekf,4,4> A1dotdot = f_ekf(b1dot)*omega_21_hat*A1dot+f_ekf(b1dotdot)*omega_21_hat*A1;
    Eigen::Matrix<f_ekf,4,4> A2fotdot = f_ekf(b2fot)*omega_32_hat*A2fot+f_ekf(b2fotdot)*omega_32_hat*A2;

    // Get the interpolated pose
    Eigen::Matrix<f_ekf,4,4> pose_interp = pose0*A0*A1*A2;
    R_GtoI = pose_interp.block(0,0,3,3).transpose();
    p_IinG = pose_interp.block(0,3,3,1);

    // Get the interpolated velocities
    // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
    Eigen::Matrix<f_ekf,4,4> vel_interp = pose0*(A0dot*A1*A2+A0*A1dot*A2+A0*A1*A2fot);
    w_IinI = vee(pose_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3));
    v_IinG = vel_interp.block(0,3,3,1);

    // Finally get the interpolated velocities
    // NOTE: Rdot = R*skew(omega)
    // NOTE: Rdotdot = Rdot*skew(omega) + R*skew(alpha) => R^T*(Rdotdot-Rdot*skew(omega))=skew(alpha)
    Eigen::Matrix<f_ekf,4,4> acc_interp = pose0*(A0dotdot*A1*A2+A0*A1dotdot*A2+A0*A1*A2fotdot
                                        +f_ekf(2)*A0dot*A1dot*A2+f_ekf(2)*A0*A1dot*A2fot+f_ekf(2)*A0dot*A1*A2fot);
    Eigen::Matrix<f_ekf,3,3> omegaskew = pose_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3);
    alpha_IinI = vee(pose_interp.block(0,0,3,3).transpose()*(acc_interp.block(0,0,3,3)-vel_interp.block(0,0,3,3)*omegaskew));
    a_IinG = acc_interp.block(0,3,3,1);
    return true;

}


bool BsplineSE3::find_bounding_poses(const f_ts timestamp, const AlignedEigenMat4f &poses,
                                     f_ts &t0, Eigen::Matrix<f_ekf,4,4> &pose0, f_ts &t1, Eigen::Matrix<f_ekf,4,4> &pose1) {

    // Set the default values
    t0 = -1;
    t1 = -1;
    pose0 = Eigen::Matrix<f_ekf,4,4>::Identity();
    pose1 = Eigen::Matrix<f_ekf,4,4>::Identity();

    // Find the bounding poses
    bool found_older = false;
    bool found_newer = false;

    // Find the bounding poses for interpolation.
    auto lower_bound = poses.lower_bound(timestamp); // Finds timestamp or next(timestamp) if not available
    auto upper_bound = poses.upper_bound(timestamp); // Finds next(timestamp)

    if(lower_bound != poses.end()) {
        // Check that the lower bound is the timestamp.
        // If not then we move iterator to previous timestamp so that the timestamp is bounded
        if(lower_bound->first == timestamp) {
            found_older = true;
        } else if(lower_bound != poses.begin()) {
            --lower_bound;
            found_older = true;
        }
    }

    if(upper_bound != poses.end()) {
        found_newer = true;
    }

    // If we found the older one, set it
    if (found_older) {
        t0 = lower_bound->first;
        pose0 = lower_bound->second;
    }

    // If we found the newest one, set it
    if (found_newer) {
        t1 = upper_bound->first;
        pose1 = upper_bound->second;
    }

    // Assert the timestamps
    if (found_older && found_newer)
        assert(t0 < t1);

    // Return true if we found both bounds
    return (found_older && found_newer); 

}



bool BsplineSE3::find_bounding_control_points(const f_ts timestamp, const AlignedEigenMat4f &poses,
                                              f_ts &t0, Eigen::Matrix<f_ekf,4,4> &pose0, f_ts &t1, Eigen::Matrix<f_ekf,4,4> &pose1,
                                              f_ts &t2, Eigen::Matrix<f_ekf,4,4> &pose2, f_ts &t3, Eigen::Matrix<f_ekf,4,4> &pose3) {

    // Set the default values
    t0 = -1;
    t1 = -1;
    t2 = -1;
    t3 = -1;
    pose0 = Eigen::Matrix<f_ekf,4,4>::Identity();
    pose1 = Eigen::Matrix<f_ekf,4,4>::Identity();
    pose2 = Eigen::Matrix<f_ekf,4,4>::Identity();
    pose3 = Eigen::Matrix<f_ekf,4,4>::Identity();

    // Get the two bounding poses
    bool success = find_bounding_poses(timestamp, poses, t1, pose1, t2, pose2);

    // Return false if this was a failure
    if (!success)
        return false;

    // Now find the poses that are below and above
    auto iter_t1 = poses.find(t1);
    auto iter_t2 = poses.find(t2);

    // Check that t1 is not the first timestamp
    if(iter_t1 == poses.begin()) {
        return false;
    }

    // Move the older pose backwards in time
    // Move the newer one forwards in time
    auto iter_t0 = --iter_t1;
    auto iter_t3 = ++iter_t2;

    // Check that it is valid
    if (iter_t3 == poses.end()) {
        return false;
    }

    // Set the oldest one
    t0 = iter_t0->first;
    pose0 = iter_t0->second;

    // Set the newest one
    t3 = iter_t3->first;
    pose3 = iter_t3->second;

    // Assert the timestamps
    if (success) {
        assert(t0 < t1);
        assert(t1 < t2);
        assert(t2 < t3);
    }

    // Return true if we found all four bounding poses
    return success;
 
}


