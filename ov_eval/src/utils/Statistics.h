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
#ifndef OV_EVAL_STATISTICS_H
#define OV_EVAL_STATISTICS_H


#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>

#include "types.h"

namespace ov_eval {


    /**
     * @brief Statistics object for a given set scalar time series values.
     *
     * Ensure that you call the calculate() function to update the values before using them.
     * This will compute all the final results from the values in @ref values vector.
     */
    struct Statistics {

    public:

        /// Root mean squared for the given values
        f_ekf rmse = 0.0;

        /// Mean of the given values
        f_ekf mean = 0.0;

        /// Median of the given values
        f_ekf median = 0.0;

        /// Standard deviation of given values
        f_ekf std = 0.0;

        /// Max of the given values
        f_ekf max = 0.0;

        /// Min of the given values
        f_ekf min = 0.0;

        /// 99th percentile
        f_ekf ninetynine = 0.0;

        /// Timestamp when these values occured at
        std::vector<f_ts> timestamps;

        /// Values (e.g. error or nees at a given time)
        std::vector<f_ekf> values;

        /// Bound of these values (e.g. our expected covariance bound)
        std::vector<f_ekf> values_bound;


        /// Will calculate all values from our vectors of information
        void calculate() {

            // Sort the data for easy finding of values
            std::vector<f_ekf> values_sorted = values;
            std::sort(values_sorted.begin(), values_sorted.end());

            // If we don't have any data, just return :(
            if(values_sorted.empty())
                return;

            // Now that its been sorted, can easily grab min and max
            min = values_sorted.at(0);
            max = values_sorted.at(values_sorted.size()-1);

            // Compute median
            // ODD:  grab middle from the sorted vector
            // EVEN: average the middle two numbers
            if (values_sorted.size()==1) {
                median = values_sorted.at(values_sorted.size()-1);
            } else if(values_sorted.size() % 2 == 1) {
                median = values_sorted.at(values_sorted.size() / 2);
            } else if(values_sorted.size() > 1) {
                median = 0.5 * (values_sorted.at(values_sorted.size()/2-1) + values_sorted.at(values_sorted.size()/2));
            } else {
                median = 0.0;
            }

            // Compute mean and rmse
            mean = 0;
            for (size_t i = 0; i < values_sorted.size(); i++) {
                assert(!flx::isnan(values_sorted.at(i)));
                mean += values_sorted.at(i);
                rmse += values_sorted.at(i) * values_sorted.at(i);
            }
            mean /= values_sorted.size();
            rmse = flx::sqrt(rmse / values_sorted.size());

            // Using mean, compute standard deviation
            std = 0;
            for (size_t i = 0; i < values_sorted.size(); i++) {
                std += flx::pow(values_sorted.at(i) - mean, f_ekf(2));
            }
            std = flx::sqrt(std / (values_sorted.size() - 1));

            // 99th percentile
            // TODO: is this correct?
            // TODO: http://sphweb.bumc.bu.edu/otlt/MPH-Modules/BS/BS704_Probability/BS704_Probability10.html
            ninetynine = mean + 2.326*std;

        }

        /// Will clear any old values
        void clear() {
            timestamps.clear();
            values.clear();
            values_bound.clear();
        }

    };



}

#endif //OV_EVAL_STATISTICS_H
