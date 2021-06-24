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
#include "Loader.h"


using namespace ov_eval;


void Loader::load_data(std::string path_traj,
                       std::vector<f_ts> &times, std::vector<Eigen::Matrix<f_ekf,7,1>> &poses,
                       std::vector<Eigen::Matrix<f_ekf,3,3>> &cov_ori, std::vector<Eigen::Matrix<f_ekf,3,3>> &cov_pos) {

    // Try to open our trajectory file
    std::ifstream file(path_traj);
    if(!file.is_open()) {
        printf(RED "[LOAD]: Unable to open trajectory file...\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Loop through each line of this file
    std::string current_line;
    while(std::getline(file, current_line)) {

        // Skip if we start with a comment
        if(!current_line.find("#"))
            continue;

        // Loop variables
        int i = 0;
        std::istringstream s(current_line);
        std::string field;
        Eigen::Matrix<f_ekf,20,1> data;
        Eigen::Matrix<f_ts,20,1> ddata;

        // Loop through this line (timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33)
        while(std::getline(s,field,' ')) {
            // Skip if empty
            if(field.empty() || i >= data.rows())
                continue;
            // save the data to our vector
            data(i) = std::atof(field.c_str());
            ddata(i) = std::atof(field.c_str());
            i++;
        }

        // Only a valid line if we have all the parameters
        if(i >= 20) {
            // time and pose
            times.push_back(ddata(0));
            poses.push_back(data.block(1,0,7,1));
            // covariance values
            Eigen::Matrix<f_ekf,3,3> c_ori, c_pos;
            c_ori << data(8),data(9),data(10),
                    data(9),data(11),data(12),
                    data(10),data(12),data(13);
            c_pos << data(14),data(15),data(16),
                    data(15),data(17),data(18),
                    data(16),data(18),data(19);
            c_ori = f_ekf(0.5)*(c_ori+c_ori.transpose());
            c_pos = f_ekf(0.5)*(c_pos+c_pos.transpose());
            cov_ori.push_back(c_ori);
            cov_pos.push_back(c_pos);
        } else if(i >= 8) {
            times.push_back(ddata(0));
            poses.push_back(data.block(1,0,7,1));
        }

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (times.empty()) {
        printf(RED "[LOAD]: Could not parse any data from the file!!\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Assert that they are all equal
    if(times.size() != poses.size()) {
        printf(RED "[LOAD]: Parsing error, pose and timestamps do not match!!\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Assert that they are all equal
    if(!cov_ori.empty() && (times.size() != cov_ori.size() || times.size() != cov_pos.size())) {
        printf(RED "[LOAD]: Parsing error, timestamps covariance size do not match!!\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Debug print amount
    //std::string base_filename = path_traj.substr(path_traj.find_last_of("/\\") + 1);
    //printf("[LOAD]: loaded %d poses from %s\n",(int)poses.size(),base_filename.c_str());

}



void Loader::load_simulation(std::string path, std::vector<Eigen::Matrix<f_ekf,Eigen::Dynamic,1>> &values) {

    // Try to open our trajectory file
    std::ifstream file(path);
    if(!file.is_open()) {
        printf(RED "[LOAD]: Unable to open file...\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Loop through each line of this file
    std::string current_line;
    while(std::getline(file, current_line)) {

        // Skip if we start with a comment
        if(!current_line.find("#"))
            continue;

        // Loop variables
        std::istringstream s(current_line);
        std::string field;
        std::vector<f_ekf> vec;

        // Loop through this line (timestamp(s) values....)
        while(std::getline(s,field,' ')) {
            // Skip if empty
            if(field.empty())
                continue;
            // save the data to our vector
            vec.push_back(std::atof(field.c_str()));
        }

        // Create eigen vector
        Eigen::Matrix<f_ekf,Eigen::Dynamic,1> temp(int(vec.size()));
        for(size_t i=0; i<vec.size(); i++) {
            temp(i) = vec.at(i);
        }
        values.push_back(temp);

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (values.empty()) {
        printf(RED "[LOAD]: Could not parse any data from the file!!\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Assert that all rows in this file are of the same length
    int rowsize = values.at(0).rows();
    for(size_t i=0; i<values.size(); i++) {
        if(values.at(i).rows() != rowsize) {
            printf(RED "[LOAD]: Invalid row size on line %d (of size %d instead of %d)\n" RESET,(int)i,(int)values.at(i).rows(),rowsize);
            printf(RED "[LOAD]: %s\n" RESET,path.c_str());
            std::exit(EXIT_FAILURE);
        }
    }

}


void Loader::load_timing_flamegraph(std::string path, std::vector<std::string> &names,
                                    std::vector<f_ts> &times, std::vector<Eigen::Matrix<f_ts, Eigen::Dynamic, 1>> &timing_values) {

    // Try to open our trajectory file
    std::ifstream file(path);
    if(!file.is_open()) {
        printf(RED "[LOAD]: Unable to open file...\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Loop through each line of this file
    std::string current_line;
    while(std::getline(file, current_line)) {

        // We should have a commented line of the names of the categories
        // Here we will process them (skip the first since it is just the timestamps)
        if(!current_line.find("#")) {
            // Loop variables
            std::istringstream s(current_line);
            std::string field;
            names.clear();
            // Loop through this line
            bool skipped_first = false;
            while(std::getline(s,field,',')) {
                // Skip if empty
                if(field.empty())
                    continue;
                // Skip the first ever one
                if(skipped_first)
                    names.push_back(field);
                skipped_first = true;
            }
            continue;
        }

        // Loop variables
        std::istringstream s(current_line);
        std::string field;
        std::vector<f_ts> vec;

        // Loop through this line (timestamp(s) values....)
        while(std::getline(s,field,',')) {
            // Skip if empty
            if(field.empty())
                continue;
            // save the data to our vector
            vec.push_back(f_ts(std::atof(field.c_str())));
        }

        // Create eigen vector
        Eigen::Matrix<f_ts, Eigen::Dynamic, 1> temp(int(vec.size()-1));
        for(size_t i=1; i<vec.size(); i++) {
            temp(i-1) = vec.at(i);
        }
        times.push_back(vec.at(0));
        timing_values.push_back(temp);

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (timing_values.empty()) {
        printf(RED "[LOAD]: Could not parse any data from the file!!\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Assert that all rows in this file are of the same length
    int rowsize = names.size();
    for(size_t i=0; i<timing_values.size(); i++) {
        if(timing_values.at(i).rows() != rowsize) {
            printf(RED "[LOAD]: Invalid row size on line %d (of size %d instead of %d)\n" RESET,(int)i,(int)timing_values.at(i).rows(),rowsize);
            printf(RED "[LOAD]: %s\n" RESET,path.c_str());
            std::exit(EXIT_FAILURE);
        }
    }

}


void Loader::load_timing_percent(std::string path, std::vector<f_ts> &times,
                                 std::vector<Eigen::Matrix<f_ts, 3, 1>> &summed_values, std::vector<Eigen::Matrix<f_ts, Eigen::Dynamic, 1>> &node_values) {

    // Try to open our trajectory file
    std::ifstream file(path);
    if(!file.is_open()) {
        printf(RED "[LOAD]: Unable to open timing file...\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Loop through each line of this file
    std::string current_line;
    while(std::getline(file, current_line)) {

        // Skip if we start with a comment
        if(!current_line.find("#"))
            continue;

        // Loop variables
        std::istringstream s(current_line);
        std::string field;
        std::vector<f_ts> vec;

        // Loop through this line (timestamp(s) values....)
        while(std::getline(s,field,' ')) {
            // Skip if empty
            if(field.empty())
                continue;
            // save the data to our vector
            vec.push_back(std::atof(field.c_str()));
        }

        // Create eigen vector
        Eigen::Matrix<f_ts, Eigen::Dynamic, 1> temp(int(vec.size()));
        for(size_t i=0; i<vec.size(); i++) {
            temp(i) = vec.at(i);
        }

        // Skip if there where no threads
        if(temp(3)==0.0)
            continue;

        // Save the summed value
        times.push_back(temp(0));
        summed_values.push_back(temp.block(1,0,3,1));
        node_values.push_back(temp.block(4,0,temp.rows()-4,1));

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (times.empty()) {
        printf(RED "[LOAD]: Could not parse any data from the file!!\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Assert that they are all equal
    if(times.size() != summed_values.size() || times.size() != node_values.size()) {
        printf(RED "[LOAD]: Parsing error, pose and timestamps do not match!!\n" RESET);
        printf(RED "[LOAD]: %s\n" RESET,path.c_str());
        std::exit(EXIT_FAILURE);
    }


}



f_ekf Loader::get_total_length(const std::vector<Eigen::Matrix<f_ekf,7,1>> &poses) {

    // Loop through every pose and append its segment
    f_ekf distance = 0.0;
    for (size_t i=1; i<poses.size(); i++) {
        distance += (poses[i].block(0,0,3,1) - poses[i-1].block(0,0,3,1)).norm();
    }

    // return the distance
    return distance;

}


