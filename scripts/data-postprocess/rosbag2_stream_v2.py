#!/usr/bin/env python
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
from __future__ import division

import roslib
import rosbag
import rospy
import genpy
import sys, getopt, os, errno
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from message_filters import *
# import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import shlex, subprocess
import gc
import time

import glob
import re  # for sorting alphanumeric


def sorted_nicely(l):
    # Sorts the given iterable in the way that is expected. Required arguments: l -- The iterable to be sorted.
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)


start_time = -1
tol = genpy.Duration(0.033333333333333)
output_frame = [None, None, None]
last_frame = [None, None, None]

global output_counter
output_counter = 0

global output
output = []

def process_frame(msg, t, cam):
    global output_counter

    timespan = genpy.Duration(output_counter * 0.03333333333333333333)
    #print cam, #, t, timespan, start_time+timespan, t - (start_time + timespan))

    if output_counter > 0:
        if t <= start_time + timespan:
            output_frame[cam] = (t, msg)
        else:
            #print ',',
            #print("**** Finally passed the tolerance... ***")
            output.append(np.vstack((np.fromstring(i[1].data, np.uint8) for i in output_frame)))
            output_counter += 1

            output_frame[cam] = (t, msg)
    else:
        output_frame[cam] = (t, msg)



# data_path = '/media/natsgld-sample/data/40/'  #Example
# data_path = './'
data_path = ''
vid_list_file = data_path + 'zed_files'
vid_out_file = data_path + 'zed'
vid_list_array = []
files = sorted_nicely(glob.glob(data_path + 'zed_*.bag'))
cnt = 0
fps = 30
ext = '.mp4'
start_time = rospy.Time()
script_start_time = time.time()

# # ------------------------------------------------------------
# #           Extract images into MP4
# # ------------------------------------------------------------

def make_video(filename):
    global fps
    global output

    pix_fmt = 'bgra'
    # size = str(msg.width) + "x" + str(msg.height*3)
    size = '2560x2160'
    FFMPEG_BIN = '/usr/local/bin/ffmpeg'

    out_file = filename[:-4] + ext

    command = [FFMPEG_BIN,
               # '-y', # overwrite output file
               '-f', 'rawvideo',
               '-pix_fmt', pix_fmt,
               # '-vcodec', 'libx264',
               '-framerate', str(fps),
               '-an',  # dont expect audio
               '-s', size,  # size of each frame
               # '-pix_fmt', pix_fmt, # pixel format
               '-i', '-',  # input from pipe
               '-pix_fmt', 'yuv420p',
               '-codec', 'h264_nvenc',
               '-preset', 'lossless',
               # '-vf', 'fps=30',
               out_file]

    ffmpeg = subprocess.Popen(command, stdin=subprocess.PIPE)
    # ffmpeg = subprocess.Popen(['avconv', '-r', str(fps), '-an', '-f', 'rawvideo', '-s', size, '-pix_fmt', pix_fmt, '-i', '-', out_file],stdin=subprocess.PIPE)

    for img in output:
        ffmpeg.stdin.write(img.tostring())
    ffmpeg.stdin.close()
    ffmpeg.terminate()
    output = []
    vid_list_array.append(out_file)


print('-----------------------------------------------------------')
for filename in files:
    print('     PROCESSING ZED file ' + filename)
    bag = rosbag.Bag(str(filename))
    for topic, msg, t in bag.read_messages():  # connection_filter == filter_image_msgs):
        if len(msg.data) > 0:
            # for now hardcoded the topics but should be configurable
            if topic == "/zed1/zed1/stereo/image_rect_color":
                process_frame(msg, t, 0)
            elif topic == "/zed2/zed2/stereo/image_rect_color":
                process_frame(msg, t, 1)
            elif topic == "/zed3/zed3/stereo/image_rect_color":
                process_frame(msg, t, 2)

            if output_counter == 0 and not None in output_frame:
                start_time = max(i[0] for i in output_frame)
                # print("**** START TIME = ", start_time)
                # print('')
                output.append(np.vstack((np.fromstring(i[1].data, np.uint8) for i in output_frame)))
                output_counter = 1

    cnt = cnt + 1
    if (cnt % 5 == 0):
        print('-----------------------------------------------------------')
        print('   --- > GENERATING ' + filename[:-4] + ext)
        print('-----------------------------------------------------------')
        make_video(filename)
        print('-----------------------------------------------------------')



# Processing any remaining
if not cnt % 5 == 0:
    print('-----------------------------------------------------------')
    print('   *** > GENERATING remaining into ' + filename[:-4] + ext)
    print('-----------------------------------------------------------')
    make_video(filename)
    print('-----------------------------------------------------------')


# ------------------------------------------------------------
#           Generate List and Merge into ONE mp4
# ------------------------------------------------------------
vid_list_f = open(vid_list_file + '.txt', 'w+')
for filename in vid_list_array:

    if '/' in filename:
        just_filename = filename[filename.rfind('/')+1:]
    else:
        just_filename = filename

    vid_list_f.write('file \'' + just_filename + '\'\n')
vid_list_f.close()

print('-------------------------------------------------------------------')
print('*    Merging all the MP4 files into ' + vid_out_file + '...')
print('-------------------------------------------------------------------')
os.system('time ffmpeg -f concat -i ' + vid_list_file + '.txt' + ' -c copy -c:v h264_nvenc -preset lossless ' + vid_out_file + ext)

# for filename in vid_list_array:
#     os.remove(filename)

# print('-------------------------------------------------------------------')
# print('*   Optimized video file format for fast loading... DONE')
# print('-------------------------------------------------------------------')
# os.system('ffmpeg -i ' + vid_out_file + mp4_optimization_params + vid_out_file[0:-4] + '_fast' + ext)

# ## TODO: ADD ROS Topics from Unity etc to save to json file

# # # Clean up
# # time.sleep(2)
# for vid in vid_list_array:
#     os.remove(vid)
# os.remove(vid_list_file)

# print('-------------------------------------------------------------------')
# print('*    Stacking and Optimizing all the MP4 files into zed.mp4...')
# print('-------------------------------------------------------------------')
# # mp4_optimization_params = ' -f mp4 -c:v libx264 -movflags +faststart -g ' + str(fps) +' -tune zerolatency -profile:v baseline -pix_fmt yuv420p '
# # VBR ffmpeg -i iphone.mov -c:v h264_nvenc -movflags +faststart -g 30 -zerolatency 1 -rc:v vbr_hq -profile:v high -pix_fmt yuv420p -weighted_pred 1 iphone_vbr_hq.mp4
# # mp4_optimization_params = ' -f mp4 -c:v h264_nvenc -movflags +faststart -g ' + str(fps) +' -zerolatency 1 -rc:v vbr_hq -profile:v high -pix_fmt yuv420p -weighted_pred 1 '
# mp4_optimization_params = ' -f mp4 -c:v h264_nvenc -zerolatency 1 -rc constqp -qp 19 -preset hq -rc-lookahead 32 -g 300 -pix_fmt yuv420p '
# os.system(
#     'time ffmpeg -i zed_0.mp4 -i zed_1.mp4 -i zed_2.mp4 -filter_complex "[0:v][1:v][2:v]vstack=inputs=3[v]" -map "[v]" ' + mp4_optimization_params + ' zed.mp4')

print('-------------------------------------------------------------------')
print('COMPLETED: Total Time Taken = ' + str(time.time() - script_start_time))
print('-------------------------------------------------------------------')

# Exit
exit()
