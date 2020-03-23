#!/usr/bin/python

import matplotlib.pyplot as plt

def plot_accuracy_and_time(df):
	fig, ax1 = plt.subplots();
	plt.title("Sequential ICP RPE")
	#ax1 = df.plot(x='leaf_size', y='translational_error_rmse')
	ax1.plot(df['leaf_size'], df['translational_error_rmse'], color='b')
	ax1.set_xlabel('Voxels Grid Leaf Size (meters)')
	ax1.set_ylabel('RPE Translational Error per Frame RMSE (m)')
	for tl in ax1.get_yticklabels():
		tl.set_color('b')

	ax2 = ax1.twinx()
	#ax2 = df.plot(x='leaf_size', y='time_mean')
	ax2.plot(df['leaf_size'], df['time_mean'], color='r')
	ax2.set_ylabel('Mean Computation Time per Frame (seconds)')
	for tl in ax2.get_yticklabels():
		tl.set_color('r')
	plt.show();