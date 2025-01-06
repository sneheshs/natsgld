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
    return int((t.to_nsec() - START_TIME)/1000000)


# /ng/baxter_animation_control
# /ng/baxter_face_animation
# /ng/gui_admin
# /ng/user_emotion
# /ng/baxter_animation_status
# /ng/clapper
# /ng/baxter_camera_angle


folders = sorted_nicely(glob.glob('/media/natsgld/data/session_*'))

VID_FIRST_TIME = 0
VID_STARTED = 1
VID_ENDED = 2

START_TIME = 0

for path in folders:
    print('Converting ' + path + ' ROSBAG NG msgs to JSON............')
    file = path + '/ng_msgs.bag'

    #Skip if file doesnt exist
    if not os.path.isfile(file):
        print('Skipping ' + file + ', files doesnt exist!!!')
        continue
    
    #Skip if already done
    out_filename = file[:-4] + '_labels.json'
    if os.path.isfile(out_filename):
        print('Skipping ' + out_filename + ', ALREADY EXISTS!!!')
        continue
    
    with open(out_filename, 'w') as outfile:
        output = {}
        last_action = []
        last_speech = []
        last_emotion = []

        event = []
        event_started = VID_FIRST_TIME

        for topic, msg, t in rosbag.Bag(file).read_messages():
            if len(msg.data) > 0:

                ##### START TIME
                if topic == '/ng/clapper':
                    print(t, topic, msg.data)

                    if msg.data == 'CLAP':
                        START_TIME = t.to_nsec()

                ##### VIDEO BANNER
                elif topic == '/ng/banner_video':

                    data = msg.data.split('|')
                    if data[0] == 'loadVideo':

                        if event_started == VID_STARTED:
                            print('Ended - And Restarted', data[1])
                            event[2] = compute_time(t)
                            output[get_hash(output)] = event

                        event = ["event", compute_time(t), -1, data[1].replace('_', ' ')]
                        event_started = VID_STARTED
                        print('Started', data[1])

                ##### ACTION
                elif topic == '/ng/baxter_animation':

                    if event_started == VID_STARTED:
                        print('Ended')
                        event[2] = compute_time(t)

                    elif event_started == VID_FIRST_TIME:
                        print('First Time', 'Opening Scene - ' + msg.data)
                        event = ['event', START_TIME, compute_time(t), 'Opening Scene - ' + msg.data]

                    else: # Should not happen but if it does, use previous event end time as start time
                        print('*** WARNING ***  Scene Change without video update', compute_time(t), msg.data)
                        event = ['event', event[2], compute_time(t), msg.data]

                    output[get_hash(output)] = event
                    event_started = VID_ENDED

        ##### DONE, SAVE JSON FILE
        json.dump(output, outfile)
