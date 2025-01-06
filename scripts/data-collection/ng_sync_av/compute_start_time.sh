#!/usr/bin/env bash

# MIT License
#
# Copyright (c) 2025 Snehesh Shrestha
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#python start_time_audio.py -audio ${data_path}/mic.mp3

root_path="/media/natsgld-sample/data/"
session_dirs=$(find $root_path -maxdepth 1 -name "session*" -type d)
for data_path in $session_dirs; do
    for file in $(ls -v ${data_path}); do
        if [[ ${file: -4} == ".mp4" ]]; then
            python start_time_video.py -video ${data_path}/${file}
            figlet Done!
        fi
        if [[ ${file: -4} == ".mov" ]]; then
            python start_time_video.py -video ${data_path}/${file}
            figlet Done!
        fi
    done
done