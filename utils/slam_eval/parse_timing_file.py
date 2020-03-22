#!/usr/bin/python

import dateutil.parser as dt_parser

def read_timing_file(filename):
	file = open(filename)
	data = file.read()
	lines = data.split('\n')
	start_times = list()
	end_times = list()
	for line in lines:
		time_strs = line.split(' ')
		if (len(time_strs) >= 2):
			start_time_str = time_strs[0]
			end_time_str = time_strs[1]
			start_time = dt_parser.parse(start_time_str)
			end_time = dt_parser.parse(end_time_str)
			start_times.append(start_time)
			end_times.append(end_time)
	return ((start_times, end_times));
