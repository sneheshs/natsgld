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


def merge_two_json_files(json_label, json_speech, json_out):
    infile = open(json_label, 'r')
    label_text = infile.read()
    infile.close()
    infile = open(json_speech, 'r')
    speech_text = infile.read()
    infile.close()

    labels = json.loads(label_text)
    speech = json.loads(speech_text)

    merged_data = {}

    for id in labels:
        merged_data[id] = labels[id]
    
    for id in speech:
        # Remove blank entries
        if any(len(x) < 1 for x in speech[id]):
            print("--- BLANK ", speech[id])
            continue

        merged_data[id] = speech[id]

    with open(json_out, 'w') as of:
        json.dump(merged_data, of)



labels_file = 'ng_msgs.json'
speech_file = 'Speech_dataset.json'

for folder in sorted_nicely(glob.glob('/media/natsgld-sample/data/session_*')):
    print ("Processing " + folder)
    
    if folder in folder_to_skip :
        print("Skipping...")
        continue

    words = folder.split('_')
    pID = words[-2]
    out_file = 'p' + pID + '_dataset.json'

    if not os.path.isfile(folder + '/' + out_file):
        if os.path.isfile(folder + '/' + labels_file) and os.path.isfile(folder + '/' + speech_file):
            print('--- Creating ' + out_file)
            merge_two_json_files(folder + '/' + labels_file, folder + '/' + speech_file, folder + '/' + out_file)
        else:
            print ("ERROR: Label or Speech file is missing: " + labels_file + ' or ' + speech_file)
    else:
        print("Skipping...Already done!!!")




