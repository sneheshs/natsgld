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
import time

def sorted_nicely( l ):
    # Sorts the given iterable in the way that is expected. Required arguments: l -- The iterable to be sorted.
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key = alphanum_key)

def get_hash(data):
    hashval = str(uuid.uuid4().hex)
    
    # Guarentees unique
    while hashval in data:
        hashval = str(uuid.uuid4().hex)

    return hashval

def compute_time(t):
    return int((t.to_nsec() - START_TIME)/1000000)


# /ng/baxter_animation_control
# /ng/baxter_face_animation
# /ng/gui_admin
# /ng/user_emotion
# /ng/baxter_animation_status
# /ng/clapper
# /ng/baxter_camera_angle
folders = sorted_nicely(glob.glob('/media/natsgld/data/Phase2/session_*'))
print(folders)

START_TIME = 0

labels = {
    "event": [1, "#874DA1"],
    "action": [2, "#AB092A"],
    "speech": [3, "#D44A00"],
    "emotion": [4, "#0965AB"],
    "scene": [5, "#60A917"],
    "robotface": [6, "#6A00FF"],
    "oz": [7, "#D80073"],
    "instructions": [8, "#647687"],
    "status": [9, "#825A2C"]
}

DEFAULT_LENGTH = 400

for path in folders:
    print ('Converting ' + path + ' ROSBAG NG msgs to DAT............')
    file = path + '/ng_msgs.bag'

    #Skip if file doesnt exist
    if not os.path.isfile(file):
        print('Skipping ' + file + ', files doesnt exist!!!')
        continue
    
    #Skip if already done
    output_file = file[:-3] + 'dat'
    # if os.path.isfile(output_file):
    #     print('Skipping ' + output_file + ', ALREADY EXISTS!!!')
    #     continue
    
    with open(output_file, 'w') as outfile:
        data = {}
        
        last_scene_id = ""
        last_event_id = ""

        for topic, msg, t in rosbag.Bag(file).read_messages():
            if len(msg.data) > 0:

                ##### START TIME
                if topic == '/ng/clapper':
                    START_TIME = t.to_nsec()

                ##### ACTION
                if topic == '/ng/baxter_animation':
                    if not last_scene_id == "":
                        data[last_scene_id][2] =  compute_time(t)
                    
                    if not last_event_id == "":
                        data[last_event_id][2] =  compute_time(t)

                    last_scene_id = get_hash(data)
                    data[last_scene_id] =  [ labels["scene"][0], compute_time(t), compute_time(t)+DEFAULT_LENGTH, msg.data.replace('Animation', '') ]

                # BAXTER REPONSE
                if topic == '/ng/baxter_face_animation':
                    data[get_hash(data)] = [ labels["robotface"][0], compute_time(t), compute_time(t)+DEFAULT_LENGTH, msg.data ]

                # # BAXTER ACTION STATUS
                # if topic == '/ng/baxter_animation_status':
                #     data[get_hash(data)] = [ labels["status"][0], compute_time(t), compute_time(t)+DEFAULT_LENGTH, msg.data ]

                # HUMAN RESPONSE
                if topic == '/ng/gui_admin':
                    #if msg.data == 'NEXT' or msg.data == "SKIP":
                    data[get_hash(data)] = [ labels["oz"][0], compute_time(t), compute_time(t)+DEFAULT_LENGTH, msg.data ]

                ##### EVENT
                if topic == '/ng/baxter_animation_control':
                    if not last_event_id == "":
                        data[last_event_id][2] =  compute_time(t)

                    last_event_id = get_hash(data)
                    data[last_event_id] = [ labels["event"][0], compute_time(t), compute_time(t)+DEFAULT_LENGTH, msg.data ]

                ##### EMOTION
                if topic == '/ng/user_emotion':
                    data[get_hash(data)] = [ labels["emotion"][0], compute_time(t), compute_time(t)+DEFAULT_LENGTH, msg.data ]

                # ##### VIDEO BANNER
                # if topic == '/ng/banner_video':
                #     data[get_hash(data)] = [ labels["instructions"][0], compute_time(t), compute_time(t)+DEFAULT_LENGTH, msg.data ]

        ##### DONE, SAVE JSON FILE
        output = {
            "header": {
                "schema_version": "2.0",
                "date_modified": int(time.time() * 1000000),
                "version": 1,
                "labels": labels
            },
            "data": data
        }

        json.dump(output, outfile)

        #exit()

