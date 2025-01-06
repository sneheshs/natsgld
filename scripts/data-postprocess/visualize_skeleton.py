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
import cv2
import numpy as np
import json
from collections import OrderedDict

cap = cv2.VideoCapture(
    '/media/natsgld/data/feva/static/data/P40/P40.mp4')

if (cap.isOpened() == False):
  print("Error opening video stream or file")

radius = 2
green = (0, 255, 0)
red = (0, 0, 255)
black = (0, 0, 0)
white = (255, 255, 255)
thickness = 1
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1

data = json.load(open('/media/natsgld/data/feva/static/data/P40/p40_keypoints.json'),
                 object_pairs_hook=OrderedDict)
cam1 = data['cam1']

frame_delay = 1

jump_to_frames = True
frame_to_jump_to = [828, 1345, 1381]
# frame_to_jump_to = [1345, 1381, 1451, 1452, 4014, 4016, 4017, 4018, 4037, 4050, 4051, 4052, 4128, 4214, 4246, 4249, 5217, 6633, 6648, 8638, 8639, 8643, 10912, 11145, 12054, 12258, 15869, 15873, 15874, 15875, 15878, 15879, 15880, 15881, 15882, 15883, 15887, 15888, 15889, 15890, 15891, 15892, 15893, 15894, 15895, 15896, 15897, 15898, 15899, 15900, 15901, 15902, 15903, 15904, 15905, 15906, 15907, 15908, 15909, 15910, 15911, 15912, 15913, 15914, 15915, 15916, 15917, 15918, 15919, 15920, 15921, 15922, 15923, 15924, 15925, 15926, 15927, 15928, 15929, 15930, 15931, 15932, 15933, 15934, 15935, 15936, 15937, 15938, 15939, 15940, 15941, 15942, 15943, 15944, 15945, 15946, 15947]
quit = False

fn = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    if jump_to_frames:
      if fn not in frame_to_jump_to:
        print fn,
        if cv2.waitKey(1) == ord('q'):
            exit()
        fn += 1
        continue
      print ("")

    if ret == True:
      kp = cam1[fn]
      for i in range(0, len(kp), 2):
          joint = (kp[i], kp[i+1])

          # Pose Map https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md
          if i/2 == 4:
            frame = cv2.circle(frame, joint, radius, red, thickness*2)
          else:
            frame = cv2.circle(frame, joint, radius, green, thickness)

      frame = cv2.putText(frame, 'Frame ' + str(fn), (20, 30), font, 
                   fontScale, white, 2, cv2.LINE_AA)

      cv2.imshow('Frame', frame)

      if jump_to_frames:
          print("Spacebar to unpause, Q to quit")
          k = 0
          cont = False
          while(not quit and not cont):
              k = cv2.waitKey(10)
              if k == ord('q'):
                  quit = True
              elif k == 32:
                  cont = True
      else:
          # Press Q to exit, Space to pause
          k = cv2.waitKey(frame_delay)
          if k == ord('q'):
            break
          elif k == 32:
              while(True):
                  k = cv2.waitKey(10)
                  if k == 32:
                      break
                  elif k == ord('q'):
                      quit = True
                      break

      if quit:
          break

    else: 
      break
    
    fn+=1

cap.release()
cv2.destroyAllWindows()
