#!/bin/bash

DATASET_HOME=/home/xero/Documents/xr/datasets
DATASET=$DATASET_HOME/TUM_VI/exported/euroc/${RES}_16/dataset-${LOC}_${RES}_16

GT_PATH=$DATASET/mav0/mocap0
GT=$GT_PATH/data.txt
EST=$HOME/traj_estimate_${RES}_${LOC}_${RATE}.txt
TIM=$HOME/traj_timing_${RES}_${LOC}_${RATE}.txt

#rosrun ov_eval format_convert $GT_PATH/data.csv
#rosrun ov_eval plot_trajectories posyaw $EST $GT
rosrun ov_eval error_singlerun posyaw $GT $EST > error_${RES}_${LOC}_${RATE}
rosrun ov_eval timing_flamegraph $TIM > timing_${RES}_${LOC}_${RATE}
