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

def sorted_nicely( l ):
    # Sorts the given iterable in the way that is expected. Required arguments: l -- The iterable to be sorted.
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key = alphanum_key)

print "Extracting All Messages except ZED images"
files = sorted_nicely(glob.glob('zed_*.bag'))
for file in files:
    cmd = 'rosbag filter ' + file + ' ' + file[:-4]+'_tmp.bag \"\'ng\' in topic\"'
    os.system(cmd)

files = sorted_nicely(glob.glob('*_tmp.bag'))
with rosbag.Bag('ng_msgs.bag', 'w') as outbag:
   for file in files:
       print 'Writing ' + file + "..."
       for topic, msg, t in rosbag.Bag(file).read_messages():
           outbag.write(topic, msg, t)

for file in files:
    os.remove(file)
