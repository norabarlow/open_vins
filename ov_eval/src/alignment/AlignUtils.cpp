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
#include "AlignUtils.h"

using namespace ov_eval;


void AlignUtils::align_umeyama(const std::vector<Eigen::Matrix<f_ekf, 3, 1>> &data,
                               const std::vector<Eigen::Matrix<f_ekf, 3, 1>> &model,
                               Eigen::Matrix<f_ekf, 3, 3> &R, Eigen::Matrix<f_ekf, 3, 1> &t,
                               f_ekf &s, bool known_scale, bool yaw_only) {

    assert(model.size() == data.size());

    // Substract mean of each trajectory
    Eigen::Matrix<f_ekf, 3, 1> mu_M = get_mean(model);
    Eigen::Matrix<f_ekf, 3, 1> mu_D = get_mean(data);
    std::vector<Eigen::Matrix<f_ekf, 3, 1>> model_zerocentered, data_zerocentered;
    for (size_t i = 0; i < model.size(); i++) {
        model_zerocentered.push_back(model[i] - mu_M);
        data_zerocentered.push_back(data[i] - mu_D);
    }

    // Get correlation matrix
    f_ekf n = f_ekf(model.size());
    Eigen::Matrix<f_ekf, 3, 3> C = Eigen::Matrix<f_ekf, 3, 3>::Zero();
    for (size_t i = 0; i < model_zerocentered.size(); i++) {
        C.noalias() += model_zerocentered[i] * data_zerocentered[i].transpose();
    }
    C *= 1.0 / n;

    // Get data sigma
    f_ekf sigma2 = 0;
    for (size_t i = 0; i < data_zerocentered.size(); i++) {
        sigma2 += data_zerocentered[i].dot(data_zerocentered[i]);
    }
    sigma2 *= 1.0 / n;

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix<f_ekf, 3, 3>> svd(C, Eigen::ComputeFullV | Eigen::ComputeFullU);

    Eigen::Matrix<f_ekf, 3, 3> U_svd = svd.matrixU();
    Eigen::Matrix<f_ekf, 3, 1> D_svd = svd.singularValues();
    Eigen::Matrix<f_ekf, 3, 3> V_svd = svd.matrixV();

    Eigen::Matrix<f_ekf, 3, 3> S = Eigen::Matrix<f_ekf, 3, 3>::Identity();
    if (U_svd.determinant() * V_svd.determinant() < 0) {
        S(2, 2) = -1;
    }

    // If only yaw, use that specific solver (optimizes over yaw angle)
    // Else get best full 3 dof rotation
    if (yaw_only) {
        Eigen::Matrix<f_ekf, 3, 3> rot_C = n * C.transpose();
        f_ekf theta = AlignUtils::get_best_yaw(rot_C);
        R = Math::rot_z(theta);
    } else {
        R.noalias() = U_svd * S * V_svd.transpose();
    }

    //If known scale, fix it
    if (known_scale) {
        s = f_ekf(1);
    } else {
        //Get best scale
        s = f_ekf(1.0) / sigma2 * (D_svd.asDiagonal() * S).trace();
    }

    // Get best translation
    t.noalias() = mu_M - s * R * mu_D;

    // Debug printing
    //std::cout << "R- " << std::endl << R << std::endl;
    //std::cout << "t- " << std::endl << t << std::endl;

}


void AlignUtils::perform_association(f_ekf offset, f_ekf max_difference,
                                     std::vector<f_ts> &est_times, std::vector<f_ts> &gt_times,
                                     std::vector<Eigen::Matrix<f_ekf,7,1>> &est_poses, std::vector<Eigen::Matrix<f_ekf,7,1>> &gt_poses) {
    std::vector<Eigen::Matrix<f_ekf,3,3>> est_covori, est_covpos, gt_covori, gt_covpos;
    AlignUtils::perform_association(offset, max_difference, est_times, gt_times, est_poses, gt_poses, est_covori, est_covpos, gt_covori, gt_covpos);
}


void AlignUtils::perform_association(f_ekf offset, f_ekf max_difference,
                                    std::vector<f_ts> &est_times, std::vector<f_ts> &gt_times,
                                    std::vector<Eigen::Matrix<f_ekf,7,1>> &est_poses, std::vector<Eigen::Matrix<f_ekf,7,1>> &gt_poses,
                                    std::vector<Eigen::Matrix<f_ekf,3,3>> &est_covori, std::vector<Eigen::Matrix<f_ekf,3,3>> &est_covpos,
                                    std::vector<Eigen::Matrix<f_ekf,3,3>> &gt_covori, std::vector<Eigen::Matrix<f_ekf,3,3>> &gt_covpos) {

    // Temp results which keeps only the matches
    std::vector<f_ts> est_times_temp, gt_times_temp;
    std::vector<Eigen::Matrix<f_ekf,7,1>> est_poses_temp, gt_poses_temp;
    std::vector<Eigen::Matrix<f_ekf,3,3>> est_covori_temp, est_covpos_temp, gt_covori_temp, gt_covpos_temp;

    // Iterator over gt (only ever increases to enforce injectivity of matches)
    size_t gt_pointer = 0;

    // Try to find closest GT pose for each estimate
    for (size_t i = 0 ; i < est_times.size(); i++) {

        // Default params
        f_ekf best_diff = max_difference;
        int best_gt_idx = -1;

        // Increment while too small and is not within our difference threshold
        while (gt_pointer < gt_times.size()
               && gt_times.at(gt_pointer) < (est_times.at(i)+offset)
               && flx::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset)) > max_difference) {
            gt_pointer++;
        }

        // If we are closer than max difference, see if we can do any better
        while (gt_pointer < gt_times.size() && flx::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset)) <= max_difference) {
            // Break if we found a good match but are getting worse, we are done
            if (flx::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset)) >= best_diff) {
                break;
            }
            // We have a closer match, save it and move on
            best_diff = flx::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset));
            best_gt_idx = gt_pointer;
            gt_pointer++;
        }

        // Did we get a valid match
        if (best_gt_idx != -1) {

            // Save estimate and gt states for the match
            est_times_temp.push_back(gt_times.at(best_gt_idx));
            est_poses_temp.push_back(est_poses.at(i));
            gt_times_temp.push_back(gt_times.at(best_gt_idx));
            gt_poses_temp.push_back(gt_poses.at(best_gt_idx));

            // If we have covariance then also append it
            // If the groundtruth doesn't have covariance say it is 100% certain
            if(!est_covori.empty()) {
                assert(est_covori.size()==est_covpos.size());
                est_covori_temp.push_back(est_covori.at(i));
                est_covpos_temp.push_back(est_covpos.at(i));
                if(gt_covori.empty()) {
                    gt_covori_temp.push_back(Eigen::Matrix<f_ekf,3,3>::Zero());
                    gt_covpos_temp.push_back(Eigen::Matrix<f_ekf,3,3>::Zero());
                } else {
                    assert(gt_covori.size()==gt_covpos.size());
                    gt_covori_temp.push_back(gt_covori.at(best_gt_idx));
                    gt_covpos_temp.push_back(gt_covpos.at(best_gt_idx));
                }
            }

        }
    }

    // Ensure that we have enough associations
    if(est_times.size() < 3) {
        printf(RED "[ALIGN]: Was unable to associate groundtruth and estimate trajectories\n" RESET);
        printf(RED "[ALIGN]: %d total matches....\n" RESET,(int)est_times.size());
        printf(RED "[ALIGN]: Do the time of the files match??\n" RESET);
        std::exit(EXIT_FAILURE);
    }
    assert(est_times_temp.size()==gt_times_temp.size());
    //printf("[TRAJ]: %d est poses and %d gt poses => %d matches\n",(int)est_times.size(),(int)gt_times.size(),(int)est_times_temp.size());

    // Overwrite with intersected values
    est_times = est_times_temp;
    est_poses = est_poses_temp;
    est_covori = est_covori_temp;
    est_covpos = est_covpos_temp;
    gt_times = gt_times_temp;
    gt_poses = gt_poses_temp;
    gt_covori = gt_covori_temp;
    gt_covpos = gt_covpos_temp;

}


