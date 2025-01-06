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

'''
Description
--------------------
If you look in load_data_sample.py, it is pretty simple to load the data using numpy
data = np.load...
then data[0] is your first command, robot response, etc. Then data[1] is your next data and so on.

The fields you will care about are:
Speech -> Text of the speech
Gesture -> Array of human 2d pose sequence of the gestures
HasSpeech -> 1 or 0 if this command has speech or not
HasGesture -> has gesture or not
Start_Frame -> Image name of the what the robot sees when the command was given
Stop_Frame -> Image name of what the robot sees when the task was completed
BaxterState -> Robot motor angle sequences for each task completed
BBox_One_Hot_Encoding -> one hot encoding of the objects that the command referred to and the robot interacted with
BBoxes -> Bounding boxes of all the objects visible (format object instance there are total of 17 objects, and x1,y1,x2,y2) 
'''


import numpy as np

dataset = np.load('../../data/states_and_features/natsgld_states_and_features_v1.0_sample.npz', allow_pickle=True)['NatComm'].item()
# Pariticipants
#dict_keys([64, 70, 40, 45, 49, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63])
print(dataset.keys())

p40 = dataset[40]

# Show all keys
print(p40.keys())
# OUTPUT: dict_keys(['tdUEHP6Z8TATRsvicJS7cy2JhUJpIeCe', 'tznNogt30s9RhnohZmbjuDqliNAvSoMQ', ... ])

# See a sample data
print(p40['tdUEHP6Z8TATRsvicJS7cy2JhUJpIeCe'])
#
# FORMAT: [ DBSN, SID, Start Time, End Time, Sudo Speech, Has Speech, Has Gesture, Action, Object, { Speech UID: [ Start time, End time, Speech, [ Glove Embedding for each word ] ]}, { 'gesture_keypoints' : [ OpenPose keypoints ] }, { 'gesture_info': [...] }
#
# OUTPUT: [1129, 27, 416300, 418749, 'turn back', 1, 1, 'on', 'gas', {'Lq9pA5GDkCvZjOts1YaP7pAlvOoiblUV': [416700, 417800, 'so turn back', [array([ ...], dtype=float32), array([ ... ], dtype=float32), array([ ...], dtype=float32)]]}, {'gesture_keypoints': array([[[..., [653, 245, 680, ..., 604, 671, 596]]]), 'gesture_info': [1.0, 1.0, 'RH|', 0.0, nan, 0.0, nan]}]

