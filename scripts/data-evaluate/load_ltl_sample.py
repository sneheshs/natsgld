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

import numpy as np

dataset = np.load('../../data/states_and_features/natsgld_LTL_v1.0_sample.npz', allow_pickle=True)

print(dataset.files)
# ['data', 'fields'])

print(dataset['fields'])
# array(['DBSN', 'PID', 'SID', 'Speech', 'Gestures', 'Gestures_len', 'LTL', 'GTA', 'GTO'], dtype='<U12')


data = dataset['data']
print(data.shape)
# (734, 9)

# See a sample data
print(data[0])

'''
FORMAT:
[ 
    DBSN,         # Database Sequence Number
    PID,          # Participant ID
    SID,          # Session ID
    Speech,       # Text of the speech
    Gesutrues,    # Array of human 2d pose sequence of the gestures
    Gestures_len, # Length of gestures (frames) 
    LTL,          # Linear Temporal Logic
    GTA,          # Ground Truth Action
    GTO           # Ground Truth Object
]

array([1, 40, 0, 'turn on the burner under the water',
       tensor([[0.4857, 0.5485, 0.7624,  ..., 0.6309, 0.5376, 0.8944],
               [0.4857, 0.5457, 0.7569,  ..., 0.6310, 0.5376, 0.8997],
               [0.4857, 0.5484, 0.7587,  ..., 0.6355, 0.5376, 0.9033],
               ...,
               [0.4808, 0.5512, 0.7859,  ..., 0.6115, 0.5350, 0.8691],
               [0.4808, 0.5511, 0.7738,  ..., 0.6116, 0.5350, 0.8658],
               [0.4808, 0.5512, 0.7821,  ..., 0.6163, 0.5375, 0.8570]]),
       94,
       'X ( F ( C_StovePot U StovePot ) & G ( C_StovePotKnob U StovePotKnob ) & F ( StovePotKnob U Stove_On ) )',
       'on', 'gas'], dtype=object)
'''
