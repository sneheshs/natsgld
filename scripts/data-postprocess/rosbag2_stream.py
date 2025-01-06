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
import sys, getopt, os, errno
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from message_filters import *
#import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import shlex, subprocess
import gc
import time

import glob
import re #for sorting alphanumeric

def sorted_nicely( l ):
    # Sorts the given iterable in the way that is expected. Required arguments: l -- The iterable to be sorted.
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key = alphanum_key)


prev_time_list = [0,0,0]
tol = rospy.Duration(secs=(0.034))

def process_frame(msg, cam_list, cam):
    t = msg.header.stamp
    #TODO: cam_list will be empty after nth zed file.
    # Maintain a last frame in each cam and remove if the next first frame in the new zed bag
    # is fine and not missing a frame.
    #print(t.to_sec())
    frame = np.fromstring(msg.data, np.uint8)
    ##print(frame.shape)
    if not cam_list:
        cam_list.append(frame)
        prev_time_list[cam] = msg.header.stamp
    else:
        dur = (t - prev_time_list[cam])
        #if cam==2:
            #print("Cam: " + str(cam) + "|Duration: " + str(dur.to_sec()))
        if (dur <= tol):
            ##if new_bag and cam_list:
                 ##new_bag = False
                 ##cam_list = []
            cam_list.append(frame)
            prev_time_list[cam] = msg.header.stamp
        else:
            missed_frames = (dur.to_sec()//(0.0333)) -1 # python2 floating limitation sadness
            print("Cam " + str(cam+1) + " Missed " + str(missed_frames) +" frames.... duplicating ....")
            i = 0
            while(i < missed_frames):
                last_frame = cam_list[-1]
                prev_time_list[cam] = prev_time_list[cam] + rospy.Duration(secs=(1/30))
                cam_list.append(last_frame)
                i = i+1
            cam_list.append(frame)
            prev_time_list[cam] = msg.header.stamp
        ##if (dur > tol and dur <= rospy.Duration(secs=(0.067))):
            ##print("Inserting after duplicates")
            ##print("Now cam " +str(cam)+" size: " + str(len(cam_list)))
            ##cam_list.append(frame)
            ##prev_time_list[cam] = msg.header.stamp
    #print("cam_list size: " + str(len(cam_list)))


    ##combined_frames_set.append(np.vstack((np.fromstring(i.data, np.uint8) for i in frame)))

# data_path = '/media/natsgld-sample/data/sessions/'
data_path = ''

vid_list_file = data_path + 'zed_'
vid_out_file = data_path + 'zed_'
vid_list_array = [[],[],[]]


# # ------------------------------------------------------------
# #           Get new bag file without ZED images
# # ------------------------------------------------------------
# print('-----------------------------------------------------------')
# print('     EXTRACTING NG Messages ...')
# print('-----------------------------------------------------------')
# files = sorted_nicely(glob.glob(data_path + 'zed_*.bag'))
# for file in files:
#     cmd = 'rosbag filter ' + file + ' ' + file[:-4]+'_tmp.bag \"\'ng\' in topic\"'
#     os.system(cmd)

# files = sorted_nicely(glob.glob('*_tmp.bag'))
# with rosbag.Bag('ng_msgs.bag', 'w') as outbag:
#    for file in files:
#        print 'Writing ' + file + "..."
#        for topic, msg, t in rosbag.Bag(file).read_messages():
#            outbag.write(topic, msg, t)

# for file in files:
#     os.remove(file)


start_time = time.time()

# # ------------------------------------------------------------
# #           Extract images into MP4
# # ------------------------------------------------------------
cnt     = 0

fps     = 30
files = sorted_nicely(glob.glob(data_path + 'zed_*.bag'))
first_cam = []
second_cam = []
third_cam = []
ext = '.mp4'
def make_video(filename, cam, num):
    global fps
    pix_fmt = 'bgra'
    #size = ''
    #size = str(msg.width) + "x" + str(msg.height*3)
    size = '2560x720'
    FFMPEG_BIN = '/usr/local/bin/ffmpeg'

    out_file = filename[:-4] + "_" + str(num) + ext
    #combined_frames = np.concatenate((first_cam,second_cam,third_cam))
    #combined_frames_set.append(np.vstack((np.fromstring(i.data, np.uint8) for i in frame)))
    command = [FFMPEG_BIN,
              #'-y', # overwrite output file
              '-f', 'rawvideo',
              '-pix_fmt', pix_fmt,
              #'-vcodec', 'libx264',
              '-framerate', str(fps),
              '-an', # dont expect audio
              '-s', size, # size of each frame
              #'-pix_fmt', pix_fmt, # pixel format
              '-i', '-', # input from pipe
              '-pix_fmt', 'yuv420p',
              '-codec', 'h264_nvenc',
              '-preset', 'lossless',
              #'-vf', 'fps=30',
              out_file]
    
    ffmpeg = subprocess.Popen(command, stdin=subprocess.PIPE)
    #ffmpeg = subprocess.Popen(['avconv', '-r', str(fps), '-an', '-f', 'rawvideo', '-s', size, '-pix_fmt', pix_fmt, '-i', '-', out_file],stdin=subprocess.PIPE)

    for img in cam:
        ffmpeg.stdin.write(img.tostring())
    ffmpeg.stdin.close()
    ffmpeg.terminate()
    cam = []
    vid_list_array[num].append(out_file)


for filename in files:
     print('-----------------------------------------------------------')
     print('     PROCESSING ZED file ' + filename)
     print('-----------------------------------------------------------')
     bag = rosbag.Bag(str(filename))
     for topic, msg, t in bag.read_messages(): # connection_filter == filter_image_msgs):
         if len(msg.data) > 0:
             # for now hardcoded the topics but should be configurable
             if topic == "/zed1/zed1/stereo/image_rect_color":
                 process_frame(msg,first_cam,0)
                 #msgarr.append(msg)
             elif topic == "/zed2/zed2/stereo/image_rect_color":
                 process_frame(msg,second_cam, 1)
             elif topic == "/zed3/zed3/stereo/image_rect_color":
                 process_frame(msg, third_cam, 2)

                 # for img_arr in combined_frames_set:
                 #     p_avconv.stdin.write(img_arr.tostring())

                 # combined_frames_set = []
                 # gc.collect()
                 # p_avconv.stdin.close()
                 # p_avconv.terminate()

                 # vid_list_array.append(out_file)

     print("first cam: " + str(len(first_cam)))
     print("second cam: " + str(len(second_cam)))
     print("third cam: " + str(len(third_cam)))
     cnt = cnt + 1
     if (cnt % 5 == 0):
         for i, cam in enumerate([first_cam, second_cam, third_cam]):
             make_video(filename, cam, i)
         first_cam = []
         second_cam = []
         third_cam = []
         gc.collect()

if(first_cam and second_cam and third_cam):
    for i, cam in enumerate([first_cam, second_cam, third_cam]):
        make_video('last.bag',cam, i)

#     # DELETE BAG FILES WHEN DONE
#     #os.remove(str(filename))


# ------------------------------------------------------------
#           Generate List and Merge into ONE mp4
# ------------------------------------------------------------
for i,vid_list in enumerate(vid_list_array):
    vid_list_f = open(vid_list_file + str(i) + '.txt', 'w+')
    #files = sorted_nicely(glob.glob(data_path + '*.mp4'))
    for filename in vid_list:
        vid_list_f.write('file \'' + filename + '\'\n')
    vid_list_f.close()

    print('-------------------------------------------------------------------')
    print('*    Merging all the MP4 files into ' + vid_out_file + str(i)  + '...')
    print('-------------------------------------------------------------------')
    os.system('time ffmpeg -f concat -i ' + vid_list_file + str(i) + '.txt' + ' -c copy -c:v h264_nvenc -preset lossless ' + vid_out_file + str(i) + ext)
    
    for filename in vid_list:
        os.remove(filename)

    #print('-------------------------------------------------------------------')
    #print('*   Optimized video file format for fast loading... DONE')
    #print('-------------------------------------------------------------------')
    #os.system('ffmpeg -i ' + vid_out_file + mp4_optimization_params + vid_out_file[0:-4] + '_fast' + ext)

# ## TODO: ADD ROS Topics from Unity etc to save to json file

# # # Clean up
# # time.sleep(2)
# for vid in vid_list_array:
#     os.remove(vid)
# os.remove(vid_list_file)

print('-------------------------------------------------------------------')
print('*    Stacking and Optimizing all the MP4 files into zed.mp4...')
print('-------------------------------------------------------------------')
#mp4_optimization_params = ' -f mp4 -c:v libx264 -movflags +faststart -g ' + str(fps) +' -tune zerolatency -profile:v baseline -pix_fmt yuv420p '
#VBR ffmpeg -i iphone.mov -c:v h264_nvenc -movflags +faststart -g 30 -zerolatency 1 -rc:v vbr_hq -profile:v high -pix_fmt yuv420p -weighted_pred 1 iphone_vbr_hq.mp4
#mp4_optimization_params = ' -f mp4 -c:v h264_nvenc -movflags +faststart -g ' + str(fps) +' -zerolatency 1 -rc:v vbr_hq -profile:v high -pix_fmt yuv420p -weighted_pred 1 '
mp4_optimization_params = ' -f mp4 -c:v h264_nvenc -zerolatency 1 -rc constqp -qp 19 -preset hq -rc-lookahead 32 -g 300 -pix_fmt yuv420p '
os.system('time ffmpeg -i zed_0.mp4 -i zed_1.mp4 -i zed_2.mp4 -filter_complex "[0:v][1:v][2:v]vstack=inputs=3[v]" -map "[v]" ' + mp4_optimization_params + ' zed.mp4')

print ('Total Time Taken = ' + str(time.time() - start_time))
# Exit
exit()
