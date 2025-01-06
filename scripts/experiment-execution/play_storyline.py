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
import pandas as pd
import rospy
from std_msgs.msg import String
import time

class storyline_player:

    curIndex = 0
    new_scene_just_loaded = False
    new_scene_just_loaded1 = False
    bContinue = True

    waiting_for_AA = False

    def __init__(self, storyline_csv):
        self.ros_init()
        self.df = pd.read_csv(storyline_csv, skiprows=1)
        self.df.fillna('-', inplace=True)

    def nextRow(self):
        self.curIndex = self.curIndex + 1

    def getCurrentRow(self):
        return self.df.iloc[self.curIndex]

    def storyline_status_handler(self, msg):
        self.waiting_for_AA = False

        status = msg.data.split(',')[1]

        last_status = status
        print('> ' + status)

        row = self.getCurrentRow()

        if status.strip().upper().startswith('COMPLETED') and row['wait_status'].strip() == '-':
            self.waiting_for_AA = True
            # print('\n*>>> WAITING FOR USER\n')

        if (status.strip().upper().startswith(row['wait_status'].strip().upper())):
            # print("\n ** MATHCHED ** \n")
            self.nextRow()

            # row = self.getCurrentRow()
            # if row['wait_status'].strip() == '-':
            #     print('\n->> WAITING FOR USER\n')

            self.play_story()

    def ros_init(self):
        self.pub_animation          = rospy.Publisher('/ng/baxter_animation', String, queue_size=10)
        self.pub_animation_control  = rospy.Publisher('/ng/baxter_animation_control', String, queue_size=10)
        self.pub_camera_angle       = rospy.Publisher('/ng/baxter_camera_angle', String, queue_size=10)
        self.pub_face_animation     = rospy.Publisher('/ng/baxter_face_animation', String, queue_size=10)

        self.sub_animation_status   = rospy.Subscriber('/ng/baxter_animation_status', String, self.storyline_status_handler)

        self.pub_user_emotion       = rospy.Publisher("/ng/user_emotion", String, queue_size=10)
        self.pub_user_action        = rospy.Publisher("/ng/user_action", String, queue_size=10)
        self.pub_user_communication = rospy.Publisher("/ng/user_communication", String, queue_size=10)

        self.pub_gui_admin          = rospy.Publisher("/ng/gui_admin", String, queue_size=10)
        self.pub_banner_video       = rospy.Publisher("/ng/banner_video", String, queue_size=10)

        rospy.init_node('storyline_player', anonymous=True)
        rate = rospy.Rate(10)  # 10hz


    def play_story(self):
        #for ind, row in self.df.iterrows():
        #    print(row)
        row = self.getCurrentRow()

        self.waiting_for_AA = False
        wait_check = False

        if not row['animation'].strip() == '-':
            print(row['animation'])
            self.pub_animation.publish(row['animation'].strip())
            wait_check = True
            self.new_scene_just_loaded = True
            self.new_scene_just_loaded1 = True

        if not row['face_animation'].strip() == '-':
            print(row['face_animation'])
            wait_check = True
            self.pub_face_animation.publish(row['face_animation'].strip())

        if not row['animation_control'].strip() == '-':
            # print(row['animation_control'])
            self.pub_animation_control.publish(row['animation_control'].strip())

        if not row['camera_angle'].strip() == '-':
            if self.new_scene_just_loaded:
                time.sleep(1)
                self.new_scene_just_loaded = False

            # print(row['camera_angle'])
            self.pub_camera_angle.publish(row['camera_angle'].strip())

        if not row['banner_video'].strip() == '-':
            if self.new_scene_just_loaded1:
                time.sleep(1)
                self.new_scene_just_loaded1 = False

            print(row['banner_video'])
            self.pub_banner_video.publish(row['banner_video'].strip())

        if wait_check and row['wait_status'].strip() == '-':
            #print('\n->> WAITING FOR USER\n')
            self.waiting_for_AA = True


    def next(self):
        #self.pub_user_action(cmd) #Publish that user did what they they were supposed to in this scene

        #print('PLAYING NEXT \n', self.getCurrentRow())

        row = self.getCurrentRow()
        if row['wait_status'].strip() == '-':
            self.nextRow()
            self.play_story()

    def skip_step(self):
        self.nextRow()
        print('SKIPPING \n', self.getCurrentRow())
        self.play_story()

    def skip_to_next_scene(self):
        print('SKIPPING \n', self.getCurrentRow())
        self.nextRow()
        row = self.getCurrentRow()
        while(row['animation'].strip() == '-'):
            print('SKIPPING \n', self.getCurrentRow())
            self.nextRow()
            row = self.getCurrentRow()
        self.play_story()

    def quit(self):
        self.bContinue = False

    def gui_admin(self, cmd):
        self.pub_gui_admin.publish(cmd)

    def baxter_animation_control(self, cmd):
        self.pub_animation_control.publish(cmd)

    def baxter_face_animation(self, cmd):
        self.pub_face_animation.publish(cmd)

    def user_emotion(self, cmd):
        self.pub_user_emotion.publish(cmd)

    def user_action(self, cmd):
        self.pub_user_action.publish(cmd)

    def user_communication(self, cmd):
        self.pub_user_communication.publish(cmd)

def main():
    #Init
    slp = storyline_player('kitchen_storyline.csv')

    #Start
    slp.play_story()

    #User Input
    inp = ''

    while slp.bContinue:
        inp = input("Press any key to continue. Press q to quit.")
        if inp == 'q':
            slp.quit()
        else:
            slp.next()

if __name__ == "__main__":
    main()
