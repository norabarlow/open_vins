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
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "utils/Statistics.h"
#include "utils/Loader.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif


int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "timing_comparison");

    // Ensure we have a path
    if(argc < 2) {
        ROS_ERROR("ERROR: Please specify a file to convert");
        ROS_ERROR("ERROR: rosrun ov_eval timing_comparison <timings_folder>");
        std::exit(EXIT_FAILURE);
    }

    // Get the algorithms we will process
    // Also create empty statistic objects for each of our datasets
    std::string path_algos(argv[1]);
    std::vector<boost::filesystem::path> path_algorithms;
    for(const auto& entry : boost::filesystem::directory_iterator(path_algos)) {
        if(boost::filesystem::is_directory(entry)) {
            path_algorithms.push_back(entry.path());
        }
    }
    std::sort(path_algorithms.begin(), path_algorithms.end());

    //===============================================================================
    //===============================================================================
    //===============================================================================

    // Summary information (%cpu, %mem, threads)
    std::map<std::string,std::vector<ov_eval::Statistics>> algo_timings;
    for(const auto& p : path_algorithms) {
        std::vector<ov_eval::Statistics> temp = {ov_eval::Statistics(),ov_eval::Statistics(),ov_eval::Statistics()};
        algo_timings.insert({p.stem().string(),temp});
    }

    // Loop through each algorithm type
    for(size_t i=0; i<path_algorithms.size(); i++) {

        // Debug print
        ROS_INFO("======================================");
        ROS_INFO("[COMP]: processing %s algorithm", path_algorithms.at(i).stem().c_str());

        // our total summed values
        std::vector<double> total_times;
        std::vector<Eigen::Vector3d> total_summed_values;

        // Loop through each sub-directory in this folder
        for(auto& entry : boost::filesystem::recursive_directory_iterator(path_algorithms.at(i))) {

            // skip if not a directory
            if(boost::filesystem::is_directory(entry))
                continue;

            // skip if not a text file
            if(entry.path().extension() != ".txt")
                continue;

            // Load the data from file
            std::vector<double> times;
            std::vector<Eigen::Vector3d> summed_values;
            std::vector<Eigen::VectorXd> node_values;
            ov_eval::Loader::load_timing(entry.path().string(), times, summed_values, node_values);
            ROS_INFO("\t-loaded %d timestamps from file!!",(int)times.size());

            // Append to our summed values
            total_times.insert(total_times.end(),times.begin(),times.end());
            total_summed_values.insert(total_summed_values.end(),summed_values.begin(),summed_values.end());

        }

        // append to the map
        std::string algo = path_algorithms.at(i).stem().string();
        for(size_t j=0; j<total_times.size(); j++) {
            algo_timings.at(algo).at(0).timestamps.push_back(total_times.at(j));
            algo_timings.at(algo).at(0).values.push_back(total_summed_values.at(j)(0));
            algo_timings.at(algo).at(1).timestamps.push_back(total_times.at(j));
            algo_timings.at(algo).at(1).values.push_back(total_summed_values.at(j)(1));
            algo_timings.at(algo).at(2).timestamps.push_back(total_times.at(j));
            algo_timings.at(algo).at(2).values.push_back(total_summed_values.at(j)(2));
        }


    }


    //===============================================================================
    //===============================================================================
    //===============================================================================



#ifdef HAVE_PYTHONLIBS

    // Plot line colors
    std::vector<std::string> colors = {"blue","red","black","green","cyan","magenta"};
    std::vector<std::string> linestyle = {"-","--","-."};
    assert(algo_timings.size() <= colors.size()*linestyle.size());

    // Parameters
    std::map<std::string, std::string> params_rpe;
    params_rpe.insert({"notch","false"});
    params_rpe.insert({"sym",""});

    // Plot this figure
    matplotlibcpp::figure_size(1500, 400);

    // Plot each RPE next to each other
    double width = 0.1/(algo_timings.size()+1);
    std::vector<double> yticks;
    std::vector<std::string> labels;
    int ct_algo = 0;
    double ct_pos = 0;
    for(auto &algo : algo_timings) {
        // Start based on what algorithm we are doing
        ct_pos = 1+1.5*ct_algo*width;
        yticks.push_back(ct_pos);
        labels.push_back(algo.first);
        // Plot it!!!
        matplotlibcpp::boxplot(algo.second.at(0).values, ct_pos, width, colors.at(ct_algo%colors.size()), linestyle.at(ct_algo/colors.size()), params_rpe, false);
        // Move forward
        ct_algo++;
    }

    // Add "fake" plots for our legend
    ct_algo = 0;
    for(const auto &algo : algo_timings) {
        std::map<std::string, std::string> params_empty;
        params_empty.insert({"label", algo.first});
        params_empty.insert({"linestyle", linestyle.at(ct_algo/colors.size())});
        params_empty.insert({"color", colors.at(ct_algo%colors.size())});
        std::vector<double> vec_empty;
        matplotlibcpp::plot(vec_empty, vec_empty, params_empty);
        ct_algo++;
    }

    // Display to the user
    matplotlibcpp::ylim(1.0-1*width, ct_pos+1*width);
    matplotlibcpp::yticks(yticks,labels);
    //matplotlibcpp::xlabel("CPU Percent Usage");
    matplotlibcpp::show(true);

#endif



    // Done!
    return EXIT_SUCCESS;

}







