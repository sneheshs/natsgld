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
    num_x = int(str.split(x[0], "e")[1])
    num_y = int(str.split(y[0], "e")[1])

    return num_x - num_y



for data_path in glob.glob('/media/natsgld-sample/data/'):

    output = OrderedDict() #{}
    f = data_path + 'zed_trimmed_key_points.json'
    
    if not os.path.isfile(f):
        print("No file, Skipping " + f + '!!!')
        continue

    if os.path.isfile(f[:-5] + '_condensed.json'):
        print("Already done, Skipping " + f + '!!!')
        continue       

    infile = open(f, 'r')
    print ("Reading " + f + "...")
    data = infile.read()
    result = json.loads(data, object_pairs_hook=OrderedDict)

    for cam in sorted(result.keys()):
        person_count = {}
        print ("Processing " + cam + "...")
        output[str(cam)] = []

        frames = collections.OrderedDict(sorted(result[cam].items(), key = functools.cmp_to_key(frame_compare)))

        pre_frame = -1

        for frame in frames.keys():
            while not int(frame[5:]) == pre_frame + 1:
                output[cam].append([])
                pre_frame += 1
                #print("********** -----------> Empty frame found " + frame[5:])
            
            new_frame_person0_found = False
            for person, keypoints in result[cam][frame].items():
                if person == 'person 0':
                    new_frame_person0_found = True
                    kp = []
                    for keypoint in keypoints:
                        kp.append(round(float(keypoint[0])))
                        kp.append(round(float(keypoint[1])))
                    output[cam].append(kp)
                else:
                    #print(frame[5:] + '-' + person.split(' ')[1], end=' ')
                    #print(person.split(' ')[1], end=' ')
                    if person.split(' ')[1] not in person_count:
                        person_count[person.split(' ')[1]] = 1
                    else:
                        person_count[person.split(' ')[1]] = person_count[person.split(' ')[1]] + 1
            
            if not new_frame_person0_found:
                output[cam].append([])
                #print("********** -----------> Person 0 not found in " + frame)

            pre_frame = int(frame[5:])

        print (person_count)

    f = f[:-5] + '_condensed.json'
    
    print ("Writing " + f + "...")
    with open(f, 'w') as outfile:
        json.dump(output, outfile)
