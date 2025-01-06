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
import os
import glob
import re
import json


# Add folders you wish to skip
folder_to_skip = [] 


def sorted_nicely( l ):
    # Sorts the given iterable in the way that is expected. Required arguments: l -- The iterable to be sorted.
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key = alphanum_key)


def create_copy(in_file, out_file):
    if not os.path.isfile(out_file):
        if os.path.isfile(in_file):
            print('--- Creating ' + out_file)
            os.system('cp ' + in_file + ' ' + out_file)
        else:
            print ("ERROR: File missing: " + in_file)
    else:
        print("Skipping...Already done!!!")


for folder in sorted_nicely(glob.glob('/media/natsgld-sample/data/')):
    print ("Processing " + folder)
    
    if folder in folder_to_skip :
        print("Skipping...")
        continue

    words = folder.split('_')
    pID = words[-2]

    in_file_keypoint = 'zed_trimmed_key_points_condensed.json'
    in_file_mp4 = 'output.mp4'
    out_file_keypoint = 'p' + pID + '_keypoints.json'
    out_file_mp4 = 'p' + pID + '.mp4'

    create_copy(folder + '/' + in_file_keypoint, folder + '/' + out_file_keypoint)
    create_copy(folder + '/' + in_file_mp4, folder + '/' + out_file_mp4)





