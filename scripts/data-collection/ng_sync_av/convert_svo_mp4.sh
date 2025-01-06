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

tmux kill-server

# AUTO PARALLEL all SVO
data_path='/media/natsgld-sample/data/'
tmux new -d -s SVO_TO_MP4 "figlet Starting SVO to MP4 conversion... Please wait.; sleep 3"

for file in $(ls -v ${data_path}); do
    if [[ ${file: -4} == ".svo" ]]; then
        tmux select-pane -t 1
        tmux split-window -v -d "../zed-export-svo-mp4/build/ZED_SVO_Export ${data_path}/${file} ${data_path}/${file%.*}.mp4 0; figlet Done!; sleep 0.5"
    fi
done
tmux attach


