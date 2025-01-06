'''
MIT License

Copyright (c) 2025 Snehesh Shrestha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
import glob
import numpy as np
import pandas as pd
import os

# from ms to 00:00:05.000 format
def ms_to_time(ms):
    seconds = ms // 1000
    ms %= 1000
    hours = seconds // 3600
    seconds %= 3600
    minutes = seconds // 60
    seconds %= 60
    return "%02d:%02d:%02d.%03d" % (hours, minutes, seconds, ms)

data_path = '../../data/'

# read csv
# dataset = pd.read_csv("natsgld_metadata_v1.0_sample.csv")
dataset = pd.read_csv(data_path + "natsgld_metadata_v1.1.csv")

in_path = data_path + "NS/"
outpath = data_path + 'videos/'

#iterate through each row
for index, row in dataset.iterrows():
    sn = row['ID']
    dbsn = row['DBSN']
    pid = row['PID']
    sid = row['SID']
    onset = row['ST']
    offset = row['ET']
    # action = str(row['GTA']).replace(" ", "_")
    # object = str(row['GTO']).replace(" ", "_")
    
    # if action == 'nan':
    #     action = 'none'
    # if object == 'nan':
    #     object = 'none'
    
    # ffmpeg -ss 00:00:05.000 -i 40.mp4 -t 00:03:05.000
    st = ms_to_time(onset)
    et = ms_to_time(offset - onset)
    infile = in_path + str(pid) + "_iphone_trimmed.mp4"
    outfile = outpath + "P" + str(pid) + "-" + '{:02d}'.format(sid) + "-" + '{:04d}'.format(dbsn) + ".mp4" # + '-' + str(action) + "-" + str(object)

    if not os.path.exists(outfile):
        # -seek_timestamp 1 -strict -2  -q
        cmd = "ffmpeg -v 0 -i " + infile + " -ss " + st + " -t " + et + " " + outfile
        print(cmd)
        # print(f"P {pid} {onset} - {offset} \n{cmd}") #, action, object
        os.system(cmd)
        # exit()
    else:
        print(f"Skipping DONE PID {pid} SID {sid}")
