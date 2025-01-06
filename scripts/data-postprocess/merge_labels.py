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


def sorted_nicely( l ):
    # Sorts the given iterable in the way that is expected. Required arguments: l -- The iterable to be sorted.
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key = alphanum_key)


def load_json(json_label_file):
    contents = json.load(open(json_label_file, 'r'))
    
    header = contents["header"]
    data = contents["data"]

    return header, data


def merge_two_json_files(primary_json, secondary_json, json_out):
    header1, data1 = load_json(primary_json)
    header2, data2 = load_json(secondary_json)

    merged_header = header1
    merged_data = data1

    # for l in merged_header['labels']:
    #     print(l, merged_header['labels'][l])

    #Highest label enum
    sn = 0
    for l in header1['labels']:
        if sn < header1['labels'][l][0]:
            sn =  header1['labels'][l][0]

    # Create mapping of label enum from old to new of json2
    new_label_mapping = {}
    for label in header2['labels']:
        if not label in merged_header['labels']:
            sn += 1
            new_label_mapping[header2['labels'][label][0]] = [sn, label]
            merged_header['labels'][label] = header2['labels'][label]
            merged_header['labels'][label][0] = sn
            #print(new_label_mapping, merged_header)
        else:
            if not header2['labels'][label][0] == merged_header['labels'][label][0]:
                new_label_mapping[header2['labels'][label][0]] = [merged_header['labels'][label][0], label]

    
    # TODO: Add check to make sure no duplicate IDs 
    for id in data2:
        merged_data[id] = data2[id]

        if merged_data[id][0] in new_label_mapping:
            #print(merged_data[id][0], new_label_mapping[merged_data[id][0]], new_label_mapping[merged_data[id][0]][0])
            merged_data[id][0] = new_label_mapping[merged_data[id][0]][0]
    
    # for l in merged_header['labels']:
    #     print(l, merged_header['labels'][l])
    # print(new_label_mapping)
    
    out_contents = {
        "header": merged_header,
        "data": merged_data
    }
    with open(json_out, 'w') as of:
        json.dump(out_contents, of)


for folder in sorted_nicely(glob.glob('/media/natsgld-sample/data/feva/static/data/p*')):
    print ("Processing " + folder)
    
    project = folder[folder.rfind('/')+1:]

    j1 = folder + '/' + project + '.dat'
    #j2 = folder + '/ng_msgs.dat'
    j2 = glob.glob('/media/natsgld-sample/data/session_' + project[-2:] + '_*')[0] + '/ng_msgs.dat'
    out_file = folder + '/' + project + '_full.dat'

    if not os.path.isfile(out_file):
        if os.path.isfile(j1) and os.path.isfile(j2):
            print('--- Creating ' + out_file)
            merge_two_json_files(j1, j2, out_file)
        else:
            print('Missing file:\n', j1, os.path.isfile(j1), '\n', j2, os.path.isfile(j2))
            print ("ERROR: One of the .dat file is missing")
    else:
        print("Skipping...Already done!!!")