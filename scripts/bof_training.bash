#!/bin/bash


groups_=(100 500 1000 1500 2000 3000 4000 5000)
surf_threshold_=(100)

for group in ${groups_[@]}
do
    for thresh in ${surf_threshold_[@]}
    do
	rosrun dolphin_slam bof_training _surf_threshold:=$thresh _bof_groups:=$group
    done
done



