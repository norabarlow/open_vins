#!/bin/bash

export HOME=$PWD

for res in 512; do
    export RES=$res
    for loc in room2; do
        export LOC=$loc
        export THRESH=0.70
        if [ $loc == room2 ] ; then
            export THRESH=0.75
        fi
        for rate in `seq 1 1`; do
            launch=src/open_vins/ov_msckf/launch/pgeneva_ros_tum_${RES} && \
            modified=${launch}_modified.launch && \
            export RATE=rate${rate} && \
            cp ${launch}.launch ${modified} && \
            sed -i "s/RRR/${rate}/" ${modified} && \
            sed -i "s/NNN/${LOC}/" ${modified} && \
            sed -i "s/THRESH/${THRESH}/" ${modified} && \
            roslaunch ov_msckf pgeneva_ros_tum_${RES}_modified.launch && \
            mv $HOME/traj_estimate.txt $HOME/traj_estimate_${RES}_${LOC}_${RATE}.txt && \
            mv $HOME/traj_timing.txt $HOME/traj_timing_${RES}_${LOC}_${RATE}.txt && \
	    ./display.sh
        done
    done
done
