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


#include <string>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

#include "calc/ResultTrajectory.h"
#include "utils/Colors.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

// Will plot three error values in three sub-plots in our current figure
void plot_3errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz) {

    // Parameters that define the line styles
    std::map<std::string, std::string> params_value, params_bound;
    params_value.insert({"label","error"});
    params_value.insert({"linestyle","-"});
    params_value.insert({"color","blue"});
    params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle","--"});
    params_bound.insert({"color","red"});

    // Plot our error value
    matplotlibcpp::subplot(3,1,1);
    std::vector<f_ts> dxvalues(sx.values.begin(), sx.values.end());
    matplotlibcpp::plot(sx.timestamps, dxvalues, params_value);
    if(!sx.values_bound.empty()) {
        std::vector<f_ts> dxvaluesbound(sx.values_bound.begin(), sx.values_bound.end());
        matplotlibcpp::plot(sx.timestamps, dxvaluesbound, params_bound);
        for(size_t i=0; i<sx.timestamps.size(); i++) {
            sx.values_bound.at(i) *= -1;
        }
        std::vector<f_ts> dxvaluesbound2(sx.values_bound.begin(), sx.values_bound.end());
        matplotlibcpp::plot(sx.timestamps, dxvaluesbound2, "r--");
    }

    // Plot our error value
    matplotlibcpp::subplot(3,1,2);
    std::vector<f_ts> dyvalues(sy.values.begin(), sy.values.end());
    matplotlibcpp::plot(sy.timestamps, dyvalues, params_value);
    if(!sy.values_bound.empty()) {
        std::vector<f_ts> dyvaluesbound(sy.values_bound.begin(), sy.values_bound.end());
        matplotlibcpp::plot(sy.timestamps, dyvaluesbound, params_bound);
        for(size_t i=0; i<sy.timestamps.size(); i++) {
            sy.values_bound.at(i) *= -1;
        }
        std::vector<f_ts> dyvaluesbound2(sy.values_bound.begin(), sy.values_bound.end());
        matplotlibcpp::plot(sy.timestamps, dyvaluesbound2, "r--");
    }

    // Plot our error value
    matplotlibcpp::subplot(3,1,3);
    std::vector<f_ts> dzvalues(sz.values.begin(), sz.values.end());
    matplotlibcpp::plot(sz.timestamps, dzvalues, params_value);
    if(!sz.values_bound.empty()) {
        std::vector<f_ts> dzvaluesbound(sz.values_bound.begin(), sz.values_bound.end());
        matplotlibcpp::plot(sz.timestamps, dzvaluesbound, params_bound);
        for(size_t i=0; i<sz.timestamps.size(); i++) {
            sz.values_bound.at(i) *= -1;
        }
        std::vector<f_ts> dzvaluesbound2(sz.values_bound.begin(), sz.values_bound.end());
        matplotlibcpp::plot(sz.timestamps, dzvaluesbound2, "r--");
    }

}

#endif


int main(int argc, char **argv) {

    // Ensure we have a path
    if(argc < 4) {
        printf(RED "ERROR: Please specify a align mode, groudtruth, and algorithm run file\n" RESET);
        printf(RED "ERROR: ./error_singlerun <align_mode> <file_gt.txt> <file_est.txt>\n" RESET);
        printf(RED "ERROR: rosrun ov_eval error_singlerun <align_mode> <file_gt.txt> <file_est.txt>\n" RESET);
        std::exit(EXIT_FAILURE);
    }

    // Load it!
    boost::filesystem::path path_gt(argv[2]);
    std::vector<f_ts> times;
    std::vector<Eigen::Matrix<f_ekf,7,1>> poses;
    std::vector<Eigen::Matrix<f_ekf,3,3>> cov_ori, cov_pos;
    ov_eval::Loader::load_data(argv[2], times, poses, cov_ori, cov_pos);
    // Print its length and stats
    f_ekf length = ov_eval::Loader::get_total_length(poses);
    printf("[COMP]: %d poses in %s => length of %.2f meters\n",(int)times.size(),path_gt.stem().string().c_str(),double(length));

    // Create our trajectory object
    ov_eval::ResultTrajectory traj(argv[3], argv[2], argv[1]);

    //===========================================================
    // Absolute trajectory error
    //===========================================================

    // Calculate
    ov_eval::Statistics error_ori, error_pos;
    traj.calculate_ate(error_ori, error_pos);

    // Print it
    printf("======================================\n");
    printf("Absolute Trajectory Error\n");
    printf("======================================\n");
    printf("rmse_ori = %.3f | rmse_pos = %.3f\n",double(error_ori.rmse),double(error_pos.rmse));
    printf("mean_ori = %.3f | mean_pos = %.3f\n",double(error_ori.mean),double(error_pos.mean));
    printf("min_ori  = %.3f | min_pos  = %.3f\n",double(error_ori.min),double(error_pos.min));
    printf("max_ori  = %.3f | max_pos  = %.3f\n",double(error_ori.max),double(error_pos.max));
    printf("std_ori  = %.3f | std_pos  = %.3f\n",double(error_ori.std),double(error_pos.std));

    //===========================================================
    // Relative pose error
    //===========================================================

    // Calculate
    std::vector<f_ekf> segments = {8.0, 16.0, 24.0, 32.0, 40.0};
    std::map<f_ekf,std::pair<ov_eval::Statistics,ov_eval::Statistics>> error_rpe;
    traj.calculate_rpe(segments, error_rpe);

    // Print it
    printf("======================================\n");
    printf("Relative Pose Error\n");
    printf("======================================\n");
    for(const auto &seg : error_rpe) {
        printf("seg %d - median_ori = %.3f | median_pos = %.3f (%d samples)\n",(int)seg.first,double(seg.second.first.median),double(seg.second.second.median),(int)seg.second.second.values.size());
        //printf("seg %d - std_ori  = %.3f | std_pos  = %.3f\n",(int)seg.first,seg.second.first.std,seg.second.second.std);
    }

#ifdef HAVE_PYTHONLIBS

    // Parameters
    std::map<std::string, std::string> params_rpe;
    params_rpe.insert({"notch","true"});
    params_rpe.insert({"sym",""});

    // Plot this figure
    matplotlibcpp::figure_size(800, 600);

    // Plot each RPE next to each other
    f_ekf ct = 1;
    f_ekf width = 0.50;
    std::vector<f_ekf> xticks;
    std::vector<std::string> labels;
    for(const auto &seg : error_rpe) {
        xticks.push_back(ct);
        labels.push_back(std::to_string((int)seg.first));
        matplotlibcpp::boxplot(seg.second.first.values, ct, width, "blue", "-", params_rpe);
        ct = ct + 1;
    }

    // Display to the user
    matplotlibcpp::xlim(f_ekf(0.5),ct-f_ekf(0.5));
    matplotlibcpp::xticks(xticks,labels);
    matplotlibcpp::title("Relative Orientation Error");
    matplotlibcpp::ylabel("orientation error (deg)");
    matplotlibcpp::xlabel("sub-segment lengths (m)");
    matplotlibcpp::show(false);

    // Plot this figure
    matplotlibcpp::figure_size(800, 600);

    // Plot each RPE next to each other
    ct = 1;
    for(const auto &seg : error_rpe) {
        matplotlibcpp::boxplot(seg.second.second.values, ct, width, "blue", "-", params_rpe);
        ct = ct + 1;
    }

    // Display to the user
    matplotlibcpp::xlim(f_ekf(0.5),ct-f_ekf(0.5));
    matplotlibcpp::xticks(xticks,labels);
    matplotlibcpp::title("Relative Position Error");
    matplotlibcpp::ylabel("translation error (m)");
    matplotlibcpp::xlabel("sub-segment lengths (m)");
    matplotlibcpp::show(false);

#endif

    //===========================================================
    // Normalized Estimation Error Squared
    //===========================================================

    // Calculate
    ov_eval::Statistics nees_ori, nees_pos;
    traj.calculate_nees(nees_ori, nees_pos);

    // Print it
    printf("======================================\n");
    printf("Normalized Estimation Error Squared\n");
    printf("======================================\n");
    printf("mean_ori = %.3f | mean_pos = %.3f\n",double(nees_ori.mean),double(nees_pos.mean));
    printf("min_ori  = %.3f | min_pos  = %.3f\n",double(nees_ori.min),double(nees_pos.min));
    printf("max_ori  = %.3f | max_pos  = %.3f\n",double(nees_ori.max),double(nees_pos.max));
    printf("std_ori  = %.3f | std_pos  = %.3f\n",double(nees_ori.std),double(nees_pos.std));
    printf("======================================\n");



#ifdef HAVE_PYTHONLIBS

    if(!nees_ori.values.empty() && !nees_pos.values.empty()) {
        // Zero our time arrays
        f_ts starttime1 = (nees_ori.timestamps.empty())? f_ts(0) : nees_ori.timestamps.at(0);
        f_ts endtime1 = (nees_ori.timestamps.empty())? f_ts(0) : nees_ori.timestamps.at(nees_ori.timestamps.size()-1);
        for(size_t i=0; i<nees_ori.timestamps.size(); i++) {
            nees_ori.timestamps.at(i) -= starttime1;
            nees_pos.timestamps.at(i) -= starttime1;
        }

        // Plot this figure
        matplotlibcpp::figure_size(1000, 600);

        // Parameters that define the line styles
        std::map<std::string, std::string> params_neesp, params_neeso;
        params_neesp.insert({"label","nees position"});
        params_neesp.insert({"linestyle","-"});
        params_neesp.insert({"color","blue"});
        params_neeso.insert({"label","nees orientation"});
        params_neeso.insert({"linestyle","-"});
        params_neeso.insert({"color","blue"});


        // Update the title and axis labels
        matplotlibcpp::subplot(2,1,1);
        matplotlibcpp::title("Normalized Estimation Error Squared");
        matplotlibcpp::ylabel("NEES Orientation");
        std::vector<f_ts> dori(nees_ori.values.begin(), nees_ori.values.end());
        matplotlibcpp::plot(nees_ori.timestamps, dori, params_neeso);
        matplotlibcpp::xlim(f_ts(0.0),endtime1-starttime1);
        matplotlibcpp::subplot(2,1,2);
        matplotlibcpp::ylabel("NEES Position");
        matplotlibcpp::xlabel("dataset time (s)");
        std::vector<f_ts> dpos(nees_pos.values.begin(), nees_pos.values.end());
        matplotlibcpp::plot(nees_pos.timestamps, dpos, params_neesp);
        matplotlibcpp::xlim(f_ts(0.0),endtime1-starttime1);

        // Display to the user
        matplotlibcpp::tight_layout();
        matplotlibcpp::show(false);
    }

#endif


    //===========================================================
    // Plot the error if we have matplotlib to plot!
    //===========================================================

    // Calculate
    ov_eval::Statistics posx, posy, posz;
    ov_eval::Statistics orix, oriy, oriz;
    ov_eval::Statistics roll, pitch, yaw;
    traj.calculate_error(posx,posy,posz,orix,oriy,oriz,roll,pitch,yaw);


    // Zero our time arrays
    f_ts starttime2 = (posx.timestamps.empty())? f_ts(0) : posx.timestamps.at(0);
    f_ts endtime2 = (posx.timestamps.empty())? f_ts(0) : posx.timestamps.at(posx.timestamps.size()-1);
    for(size_t i=0; i<posx.timestamps.size(); i++) {
        posx.timestamps.at(i) -= starttime2;
        posy.timestamps.at(i) -= starttime2;
        posz.timestamps.at(i) -= starttime2;
        orix.timestamps.at(i) -= starttime2;
        oriy.timestamps.at(i) -= starttime2;
        oriz.timestamps.at(i) -= starttime2;
        roll.timestamps.at(i) -= starttime2;
        pitch.timestamps.at(i) -= starttime2;
        yaw.timestamps.at(i) -= starttime2;
    }

#ifdef HAVE_PYTHONLIBS

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure_size(1000, 600);
    plot_3errors(posx,posy,posz);

    // Update the title and axis labels
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::title("IMU Position Error");
    matplotlibcpp::ylabel("x-error (m)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::ylabel("y-error (m)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::ylabel("z-error (m)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(false);

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure_size(1000, 600);
    plot_3errors(orix,oriy,oriz);

    // Update the title and axis labels
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::title("IMU Orientation Error");
    matplotlibcpp::ylabel("x-error (deg)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::ylabel("y-error (deg)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::ylabel("z-error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(false);

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure_size(1000, 600);
    plot_3errors(roll,pitch,yaw);

    // Update the title and axis labels
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::title("Global Orientation RPY Error");
    matplotlibcpp::ylabel("roll error (deg)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::ylabel("pitch error (deg)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::ylabel("yaw error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::xlim(f_ts(0.0),endtime2-starttime2);

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(true);


#endif


    // Done!
    return EXIT_SUCCESS;

}


