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
import json
import os
import collections
import functools
from collections import OrderedDict

def frame_compare(x,y):
    num_x = int(str.split(str(x[0]), "e")[1])
    num_y = int(str.split(str(y[0]), "e")[1])

    return num_x - num_y


def get_frame_rect(camera_ind):
    w = 1280
    h = 720
    row = int((ind - 1) / 2)
    col = (ind + 1) % 2
    x = w * (col + 1)
    y = h * row

    return x, y, x+w, y+h


import cv2
path = '/media/natsgld/data/40/'
vidcap = cv2.VideoCapture(path + 'p40.mp4')
# vidcap.set(cv2.CAP_PROP_POS_AVI_RATIO,1)
# print("Length = " + str(vidcap.get(cv2.CAP_PROP_POS_MSEC)) + "ms")

#vidcap.set(cv2.CAP_PROP_POS_MSEC, 0.0)

print("Timestamp = " + str(vidcap.get(cv2.CAP_PROP_POS_MSEC)) )
print("Frame = " + str(vidcap.get(cv2.CAP_PROP_FRAME_COUNT)) )



def display_frame(keypoints):
    
    #print("Timestamp = " + str(vidcap.get(cv2.CAP_PROP_POS_MSEC)) )

    success, img = vidcap.read()
    display_keypoints(img, keypoints)
    cv2.imshow('image', img)
    cv2.waitKey(1)


cam_num = 4

def display_keypoints(img, keypoints):
    for keypoint in keypoints:
        x = int(round(float(keypoint[0])))
        y = int(round(float(keypoint[1])))

        if cam_num == 6:
            cv2.circle(img, (1280 + x, 720 * 2 + y), 3, (0, 255, 0), thickness=1, lineType=8, shift=0)
        elif cam_num == 5:
            cv2.circle(img, (x, 720 * 2 + y), 3, (0, 255, 0), thickness=1, lineType=8, shift=0)
        elif cam_num == 4:
            cv2.circle(img, (1280 + x, 720 + y), 3, (0, 255, 0), thickness=1, lineType=8, shift=0)
        elif cam_num == 3:
            cv2.circle(img, (x, 720 + y), 3, (0, 255, 0), thickness=1, lineType=8, shift=0)
        elif cam_num == 2:
            cv2.circle(img, (1280 + x, y), 3, (0, 255, 0), thickness=1, lineType=8, shift=0)
        else:
            cv2.circle(img, (x, y), 3, (0, 255, 0), thickness=1, lineType=8, shift=0)



for data_path in glob.glob('/media/natsgld-sample/data/40/'):

    output = OrderedDict() #{}
    f = data_path + 'zed_trimmed_key_points.json'
    
    if not os.path.isfile(f):
        print("No file, Skipping " + f + '!!!')
        continue     

    infile = open(f, 'r')
    print ("Reading " + f + "...")
    data = infile.read()
    result = json.loads(data, object_pairs_hook=OrderedDict)


    for cam in sorted(result.keys()):
        if not cam == 'cam' + str(cam_num):
            continue

        print ("Processing " + cam + "...")
        output[str(cam)] = []

        frames = collections.OrderedDict(sorted(result[cam].items(), key = functools.cmp_to_key(frame_compare)))

        ind = 0
        for frame in frames.keys():
            #print(frame)
            for person, keypoints in result[cam][frame].items():
                if person == 'person 0':
                    kp = []
                    for keypoint in keypoints:
                        kp.append(round(float(keypoint[0])))
                        kp.append(round(float(keypoint[1])))

                    display_frame(keypoints)
                    #print(str(ind) + ' - ' + str(frame) + " -- ")

                    output[cam].append(kp)
                else:
                    keypoints = []
                    #print(person, end=', ')

            ind += 1


    # f = f[:-5] + '_condensed.json'
    # print ("Writing " + f + "...")
    # with open(f, 'w') as outfile:
    #     json.dump(output, outfile)
