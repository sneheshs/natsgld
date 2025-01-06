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
from flask import Flask, render_template, request
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys 
import os
import json
import config as conf
import session

import threading

from play_storyline import *

# GLOBAL
global slp
global robot_control_commands
global robot_face_controls
global user_emotions
global user_actions
global user_communications
#global sub_animation_status

global shutdown
shutdown = False

# FLASK APP
app = Flask(__name__)

#for debugging purposes only!
app.debug = False;

@app.route('/')
def execute():   
    global scene
    if scene == "kitchen_scene":
        i = 2
    elif scene == "cleaning_scene":
        i = 3
    elif scene == "placeholder":
        i = 4
    else:
        i = 2
    return render_template('main.html', commands=ACTIONS_LIST[i])

@app.route('/<cmd>')
def command(cmd=None):
    # #Start logging commands
    # f = open("keylogging.txt", "a")
    # f.write(str(time.time())+","+cmd + "\n")

    action_onclick(cmd)                                                                                   
    response = cmd
    return response, 200, {'Content-Type': 'text/plain'}   

@app.route('/next')
def new_section():
    global bUpdate
    bUpdate = True
    return render_template('main.html', commands=ACTIONS_LIST[3])

@app.route('/refresh')
def doUpdate(cmd=None):
    global bUpdate
    global new_scene
    if bUpdate:
        bUpdate = False
        return "Update"
    elif new_scene:
        new_scene = False
        return "NewScene"
    else:
        return ""

global divupdate_firsttime

@app.route('/divupdate')
def divupdate():
    global slp
    global divupdate_firsttime

    if divupdate_firsttime:
        return ""
    elif slp.waiting_for_AA:
        return "Waiting for User Input..."
    else:
        return ".. action in progress .."



def shutdown_server():
    environ = request.environ
    if not 'werkzeug.server.shutdown' in environ:
        raise RuntimeError('Not running the development server')
    environ['werkzeug.server.shutdown']()

def action_onclick(cmd):
    global slp
    global robot_control_commands
    global robot_face_controls
    global user_emotions
    global user_actions
    global user_communications

    global shutdown
    global divupdate_firsttime

    print(cmd)

    if cmd == "START":
        shutdown = False
        divupdate_firsttime = False
        slp.gui_admin(cmd)
        slp.play_story()

    elif cmd == "NEXT":
        slp.gui_admin(cmd)
        slp.next()

    elif cmd == "SKIPSTEP":
        slp.gui_admin(cmd)
        slp.skip_step()

    elif cmd == "SKIP":
        slp.gui_admin(cmd)
        slp.skip_to_next_scene()

    elif cmd == "QUIT":
        shutdown = True
        slp.gui_admin(cmd)
        slp = None
        shutdown_server()

    elif cmd in robot_control_commands:
        slp.baxter_animation_control(cmd)

    elif cmd in robot_face_controls:
        slp.baxter_face_animation(cmd)

    elif cmd in user_emotions:
        slp.user_emotion(cmd)

    elif cmd in user_actions:
        slp.user_action(cmd)

    elif cmd in user_communications:
        slp.user_communication(cmd)


global bUpdate
global scene
global new_scene
scene = ""
bUpdate = False
def callback(data):
    global bUpdate
    global sub
    bUpdate = True
    # This make sure the page doesn't keep updating every 5 seconds and
    # server side is not busy handing this callback
    # BUT the catch is, at the moment this only works the first time Flask server is launched
    # if the browser is refreshed once this has happened, you have restart the flask for this to work again
    # sub.unregister()


def callback_scene(data):
    global new_scene
    global scene
    scene = data.data
    new_scene = True


def callback_animation_status(data):
    print(data)


def thread_waiting_for_AA(some_val):
    global shutdown
    global waiting_for_AA
    #print("Thread Started")
    b = False
    while not shutdown:
        b = False
        time.sleep(1.5)
        #print("_", end='')
        while True:
            if shutdown:
                break
            elif slp is None:
                break
            elif not slp.waiting_for_AA:
                time.sleep(1.5)
                #print("..", end='')
                b = True
            else:
                break

        if b and not shutdown:
            print("\nWAITING FOR YOUR INPUT")



if __name__ == '__main__':

    # -------------- SESSION --------------
    with open('session_'+str(session.sessionID)+'.json') as f:
        data = json.load(f)
    ACTIONS_LIST = data[str(session.sessionID)]["ParticipantBB"]
    # ACTIONS = data[str(session.sessionID)]["ParticipantBB"][2]

    global robot_control_commands
    global robot_face_controls
    global user_emotions
    global user_actions
    global user_communications
    robot_control_commands = []
    robot_face_controls = []
    user_emotions = []
    user_actions = []
    user_communications = []

    for robot_cmds in ACTIONS_LIST[2]['Robot Control']:
        robot_control_commands.append(robot_cmds.split('|')[1])

    for robot_face in ACTIONS_LIST[2]['Robot Face']:
        robot_face_controls.append(robot_face.split('|')[1])

    for user_emos in ACTIONS_LIST[2]['User Emotions']:
        user_emotions.append(user_emos.split('|')[1])

    for user_acts in ACTIONS_LIST[2]['User Actions']:
        user_actions.append(user_acts.split('|')[1])

    for user_comms in ACTIONS_LIST[2]['User Communication']:
        user_communications.append(user_comms.split('|')[1])


    # -------------- STORYLINE --------------

    global divupdate_firsttime
    divupdate_firsttime = True

    global slp
    slp = storyline_player('kitchen_storyline.csv')

    # Module to check if waiting for researcher's input or robot is doing something
    x = threading.Thread(target=thread_waiting_for_AA, args=(1,))
    x.start()

    # -------------- ROS --------------
    # rospy.init_node('Participant_BB_UI', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # global sub_animation_status
    # sub = rospy.Subscriber("/zed1/zed1/left/image_rect_color", Image, callback)
    # sub_animation_status = rospy.Subscriber("/ng/baxter_animation_status", String, callback_animation_status)

    # -------------- FLASK --------------
    app.run()



