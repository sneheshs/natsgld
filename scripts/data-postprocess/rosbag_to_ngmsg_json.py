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
import os       #system command
import glob     #getting file list
import re       #for sorting alphanumeric

import rosbag
import rospy

import uuid     #get random alpha numeric
import json


def sorted_nicely( l ):
    # Sorts the given iterable in the way that is expected. Required arguments: l -- The iterable to be sorted.
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key = alphanum_key)

def get_hash(output):
    hashval = str(uuid.uuid4().hex)
    
    # Guarentees unique
    while hashval in output:
        hashval = str(uuid.uuid4().hex)

    return hashval

def compute_time(t):
    return (t.to_nsec() - START_TIME)/1000000


# /ng/baxter_animation_control
# /ng/baxter_face_animation
# /ng/gui_admin
# /ng/user_emotion
# /ng/baxter_animation_status
# /ng/clapper
# /ng/baxter_camera_angle
folders = sorted_nicely(glob.glob('/media/natsgld/data/session_*'))

START_TIME = 0

for path in folders:
    print ('Converting ' + path + ' ROSBAG NG msgs to JSON............')
    file = path + '/ng_msgs.bag'

    #Skip if file doesnt exist
    if not os.path.isfile(file):
        print('Skipping ' + file + ', files doesnt exist!!!')
        continue
    
    #Skip if already done
    if os.path.isfile(file[:-3] + 'json'):
        print('Skipping ' + file[:-3] + 'json' + ', ALREADY EXISTS!!!')
        continue
    
    with open(file[:-3] + 'json', 'w') as outfile:
        output = {}
        last_action = []
        last_speech = []
        last_emotion = []
        last_event = []
        for topic, msg, t in rosbag.Bag(file).read_messages():
            if len(msg.data) > 0:

                ##### START TIME
                if topic == '/ng/clapper':
                    START_TIME = t.to_nsec()

                ##### ACTION
                if topic == '/ng/baxter_animation':
                    if not last_action == []:
                        output[get_hash(output)] = last_action

                    last_action = [ "action", compute_time(t), compute_time(t)+1000, msg.data ]

                if topic == '/ng/baxter_face_animation':
                    if not last_action == []:
                        output[get_hash(output)] = last_action
                    
                    last_action = [ "action", compute_time(t), compute_time(t)+1000, msg.data ]

                elif topic == '/ng/gui_admin':
                    if msg.data == 'NEXT' or msg.data == "SKIP":
                        if not last_action == []:
                            last_action[2] = compute_time(t)
                            output[get_hash(output)] = last_action
                            last_action = []
                        else:
                            print('WARNING... Weird NEXT or SKIP without NEW ACTION')

                ##### EVENT
                elif topic == '/ng/baxter_animation_control':
                    if last_event == []:
                        last_event = [ "event", compute_time(t), compute_time(t)+1000, msg.data ]
                    else:
                        last_event[2] = compute_time(t)
                        output[get_hash(output)] = last_event
                        last_event = [ "event", compute_time(t), compute_time(t)+1000, msg.data ]
                
                ##### EMOTION
                elif topic == '/ng/user_emotion':
                    last_emotion = [ "emotion", compute_time(t)-1000, compute_time(t)+1000, msg.data ]
                    output[get_hash(output)] = last_emotion
                    last_emotion = []

                ##### VIDEO BANNER
                elif topic == '/ng/banner_video':
                    last_emotion = [ "event", compute_time(t)-1000, compute_time(t)+3000, 'VIDEO - ' + msg.data ]
                    output[get_hash(output)] = last_emotion
                    last_emotion = []

        # Make sure you save the last messages that might not have been saved
        if not last_action == []:
            output[get_hash(output)] = last_action
        if not last_event == []:
            output[get_hash(output)] = last_event
        if not last_emotion == []:
            output[get_hash(output)] = last_emotion

        ##### DONE, SAVE JSON FILE
        json.dump(output, outfile)
