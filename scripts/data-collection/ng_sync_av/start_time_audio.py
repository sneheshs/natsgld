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

from pydub import AudioSegment
import scipy.io.wavfile
import numpy as np
import os
import argparse

import matplotlib.pyplot as plt
import numpy as np

def audio_start_time(path):
    sound = AudioSegment.from_mp3(path)
    dest = path[:-3] + 'wav'
    sound.export(dest, format="wav")
    rate, audData = scipy.io.wavfile.read(dest)

    audData = np.abs(audData)

    # plt.plot(audData)
    # plt.show()

    peaks = []
    # tol = 20000
    # window_size = 20
    # for i in range(0, len(audData) - window_size, 20):
    #     if np.average(audData[i+window_size:i+window_size*2]) > np.average(audData[i:i+window_size]) + tol:
    #         peak = float(i) / float(rate)
    #         peaks.append(peak)
    #         print(peak)

    sorted_vals = np.sort(audData)
    temp = []
    for i in range(1, 20):
        temp.append(float(np.where(audData == sorted_vals[-i])[0])/float(rate))

    temp = np.sort(temp)
    peaks.append(temp[0])
    for i in range(1, len(temp)):
        if temp[i] > peaks[-1] - 0.01 and temp[i] < peaks[-1] + 0.01:
            peaks[-1] = min(temp[i], peaks[-1])
        else:
            peaks.append(temp[i])


    # plt.plot(peaks)
    # plt.show()
    # clap_time = float(np.argmax(audData)) / float(rate) # OLD stuff

    os.remove(dest)

    return peaks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-audio", type=str, help="audio file path")
    args = parser.parse_args()

    start_time_audio = audio_start_time(args.audio)

    with open(args.audio[:-3]+'txt', "w+") as f:
        for sta in start_time_audio:
            f.write(str(round(sta, 3) + 0.3) + '\n')

if __name__ == "__main__":
    main()
