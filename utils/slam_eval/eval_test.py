#!/usr/bin/python

import pandas as pd
from pandas import Series, DataFrame
import os
import parse_timing_file
import matplotlib.pyplot as plt
import numpy as np
import datetime

# Import scripts from TUM RGBD Evaluation
import evaluate_rpe

# Set up paths for dataset directory and SLAM output directory
dataset_root =  "/home/atrevor/data/tum_rgbd/";
benchmark_root = "/home/atrevor/benchmark/icp_seq/";
fr1_names = ["360", "desk", "desk2", "floor", "plant", "room", "rpy", "teddy", "xyz"];

# Load all ground truth trajectories
fr1_gt_trajectories = dict();
for fr1_name in fr1_names:
	gt_filename = dataset_root + "rgbd_dataset_freiburg1_" + fr1_name + "/groundtruth.txt";
	gt_traj = evaluate_rpe.read_trajectory(gt_filename);
	fr1_gt_trajectories[fr1_name] = gt_traj;

# Load trajectories from our experiment, and parse the filenames into a pandas dataframe
traj_root = benchmark_root + "trajectories/";
timing_root = benchmark_root + "timing/";
slam_filenames = list();
slam_trajectories = list();
dataset_name = list();
icp_leafsize = list();
icp_corrdist = list();
slam_rpe = list();
translational_error = list();
rotational_error = list();
translational_error_rmse = list();
translational_error_mean = list();
translational_error_std = list();
start_times = list();
end_times = list();
time_mean = list();

for trajectory_file in os.listdir(traj_root):
	slam_traj = evaluate_rpe.read_trajectory(traj_root + trajectory_file);
	slam_filenames.append(trajectory_file);
	slam_trajectories.append(slam_traj);
	split_filename = trajectory_file.split('_');
	dataset_name.append(split_filename[4]);
	icp_leafsize.append(np.double(split_filename[6]));
	corrdist = split_filename[8];
	corrdist = corrdist.replace('.txt','');
	icp_corrdist.append(np.double(corrdist));
	#print ("gt: " + split_filename[4] + " traj: " + trajectory_file)
	#print("gt len: " + str(len(fr1_gt_trajectories[split_filename[4]])) + " slamm len: " + str(len(slam_traj)))
	print("Processing " + trajectory_file + "...\n");
	rpe = evaluate_rpe.evaluate_trajectory(fr1_gt_trajectories[split_filename[4]], slam_traj, 0, True, 1.00, "f", 0.00, 1.00);
	slam_rpe.append(rpe);
	trans_error = np.array(rpe)[:,4];
	rot_error = np.array(rpe)[:,5];
	translational_error.append(trans_error);
	rotational_error.append(rot_error);
	translational_error_rmse.append(np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)));
	translational_error_mean.append(np.mean(trans_error));
	translational_error_std.append(np.std(trans_error));

	#Now parse timing file
	print("Processing timing: " +  timing_root + trajectory_file)
	(frame_start_times, frame_end_times) = parse_timing_file.read_timing_file(timing_root + trajectory_file);
	start_times.append(frame_start_times);
	end_times.append(frame_end_times);
	start_time_arr = np.array(frame_start_times);
	end_time_arr = np.array(frame_end_times);
	diff_arr = end_time_arr - start_time_arr;
	diff_seconds_arr = map(datetime.timedelta.total_seconds, diff_arr)
	mean_frame_time = np.array(diff_seconds_arr).mean();
	time_mean.append(mean_frame_time);



data = {'file_name' : slam_filenames, 
		'dataset_name' : dataset_name, 
		'slam_trajectory' : slam_trajectories, 
		'leaf_size' : icp_leafsize, 
		'corr_dist' : icp_corrdist,
		'rpe_result' : slam_rpe,
		'translational_error' : translational_error,
		'rotational_error' : rotational_error,
		'translational_error_rmse' : translational_error_rmse,
		'translational_error_mean' : translational_error_mean,
		'translational_error_std' : translational_error_std,
		'frame_start_times' : start_times,
		'frame_end_times' : end_times,
		'time_mean' : time_mean}

df_full = DataFrame(data, index=slam_filenames);
dataset_dfs = dict();
for name in df_full['dataset_name'].unique():
	set_df = df_full[df_full['dataset_name']==name][['translational_error_rmse', 'leaf_size', 'time_mean']]
	set_df.sort('leaf_size', inplace=True)
	dataset_dfs[name] = set_df

