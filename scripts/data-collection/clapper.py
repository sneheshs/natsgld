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

import pygame as pg
import time
import rospy
from std_msgs.msg import String
from pygame.locals import *
from pynput.keyboard import Key, Listener #for key press
import threading

global DoneKeyCount
DoneKeyCount = 3

global bNotPressed
def on_press(key):
    global pub
    global bNotPressed
    global DoneKeyCount

    # val = "" + format(key)
    # print("PRESSED ", val, rospy.get_time())

    if key == Key.space:
        bNotPressed = False
        DoneKeyCount -= 1

    if DoneKeyCount == 0:
        exit()

# OLD METHOD
# import sys,tty,termios
# class _Getch:
#     def __call__(self):
#         fd=sys.stdin.fileno()
#         old_set=termios.tcgetattr(fd)
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch=sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_set)
#         return ch

# OLD METHOD
# def wait_key():
#     inkey=_Getch()
#     k=''
#     while not k==' ':
#         k=inkey()

def wait_key():
    global bNotPressed
    bNotPressed = True
    while bNotPressed:
        time.sleep(0.001)

# SCREEN COLORS
color1 = (255, 0, 0)
color2 = (0, 0, 255)

# ROS INIT
global pub_clapper
pub_clapper = rospy.Publisher('/ng/clapper', String, queue_size=10)
rospy.init_node('ngclapper', anonymous=True)
rate = rospy.Rate(100)  # 10hz

def runScreenFlipper():
    global pub_clapper

    pg.init()

    # PAINT THE WINDOW RED and SEND ROS MSG STARTED
    info = pg.display.Info()
    screen = pg.display.set_mode([info.current_w, info.current_h])
    screen.fill(color1)
    pub_clapper.publish("START")
    pg.display.flip()

    # CHANGE WINDOW TO BLUE and SEND 1st CLAPPED -> Begin Video
    screen.fill(color2)
    wait_key()

    # KEY PRESSED so publish
    pub_clapper.publish("CLAP")
    pg.display.flip()

    # CHANGE WINDOW BACK TO RED and SEND 2nd CLAP -> End Video
    screen.fill(color1)
    wait_key()

    # KEY PRESSED
    pub_clapper.publish("DONE")
    pg.display.flip()

    # DONE - Close window and exit
    wait_key()
    pg.quit()
    exit()

t = threading.Thread(target=runScreenFlipper)
t.start()

with Listener(on_press=on_press) as listener: listener.join()
