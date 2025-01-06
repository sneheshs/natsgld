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

import argparse
# from pydub import AudioSegment
import subprocess
import re
import time
import os
import glob

def get_video_length(path):
    process = subprocess.Popen(['ffmpeg', '-i', path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    stdout, stderr = process.communicate()
    str_out = str(stdout, "utf-8")
    matches = re.search("Duration:\s{1}\d+:\d+:\d+\.\d+", str_out).group()
    matches = matches[10:]
    # matches = matches.split(":")
    # print(matches)

    # hours = Decimal(matches[0])
    # minutes = Decimal(matches[1])
    # seconds = Decimal(matches[2])

    # total = 0
    # total += 60 * 60 * hours
    # total += 60 * minutes
    # total += seconds
    return matches


# def sync_video_audio(audio_path, video_path, audio_start, video_start):
 

#     # video trim
#     duration = get_video_length(video_path)
#     duration_arr = duration.split(":")
#     duration_sec = 0.0
#     hour = float(duration_arr[0]) * 60 * 60
#     min = float(duration_arr[1]) * 60
#     sec = float(duration_arr[2])

#     duration_sec = hour + min + sec
#     print(duration_sec)

#     video_end_time = float(duration_sec - video_start)
#     video_end_time_mil = float(video_end_time - int(video_end_time))

#     end_time = time.strftime('%H:%M:%S', time.gmtime(int(video_end_time)))
#     end_time = str(end_time) + "." + str(round(video_end_time_mil, 3)).split(".")[1]

#     video_start_time = time.strftime('%H:%M:%S', time.gmtime(video_start))

#     video_start_time = str(video_start_time) + "." + str(round(video_start - int(video_start), 3)).split(".")[1]
#     video_offset = "00:00:00"
#     os.system("ffmpeg -ss %s -i %s -ss %s -t %s -c copy video-extract.mp4" % (video_offset, video_path, video_start_time, end_time))

#     cmd = 'ffmpeg -i video-extract.mp4 -i audio-extract.wav -c:v copy -c:a aac -strict experimental output.mp4'
#     subprocess.call(cmd, shell=True)  # "Muxing Done
#     print('Muxing Done')


def trim_video(video_path, video_start_time, end_time):    
    #video_offset = "00:00:00.000"

    if not os.path.isfile(video_path[:-4] + '_trimmed.mp4'):
        #cmd1 = 'ffmpeg -ss %s -i %s -ss %s -t %s -c copy %s' % (video_offset, video_path, video_start_time, end_time, video_path[:-4]+'_trimmed.mp4')
        #cmd = 'ffmpeg -seek_timestamp 1 ' + ' -ss ' + str(video_start_time) + ' -i ' + str(video_path) + ' -strict -2 -t ' + str(end_time) + ' -c copy ' + video_path[:-4] + '_trimmed.mp4'


        #cmd = 'ffmpeg -seek_timestamp 1 ' + ' -ss ' + str(video_start_time) + ' -i ' + str(video_path) + ' -strict -2 -t ' + str(end_time) + ' -c copy ' + video_path[:-4] + '_trimmed.mp4'
        mp4_optimization_params = ' -f mp4 -c:v h264_nvenc -zerolatency 1 -rc constqp -qp 19 -preset hq -rc-lookahead 32 -g 300 -pix_fmt yuv420p '
        cmd = 'ffmpeg -seek_timestamp 1 ' + ' -ss ' + str(video_start_time) + ' -i ' + str(video_path) + ' -strict -2 -t ' + str(end_time) + mp4_optimization_params + video_path[:-4] + '_trimmed.mp4'
 
        print("COMMAND: " + cmd)
        os.system(cmd)
    else:
        print("Skipping Trimmed MP4 generation...")


def get_start_end_time(data_path, vid_type):
    vidfile = data_path + vid_type[:-4] + '.txt'

    if not os.path.isfile(vidfile):
        print('Skipping Trimming video, ' + vidfile + ' does not exisit!!!')
        return '', 0, 0
        #exit()

    arr = open(vidfile, "r").read().split("\n")
    
    video_path = data_path + vid_type
    video_start = float(arr[0])

    # video trim
    #print('START TIME: ', str(video_start))

    duration = get_video_length(video_path)
    #print('VIDEO DURATION: ', str(duration))

    duration_arr = duration.split(":")
    duration_sec = 0.0
    hour = float(duration_arr[0]) * 60 * 60
    min = float(duration_arr[1]) * 60
    sec = float(duration_arr[2])
    duration_sec = hour + min + sec
    #print('TOTAL SECONDS: ', str(duration_sec))

    video_end_time = float(duration_sec - video_start)
    video_end_time_mil = float(video_end_time - int(video_end_time))
    end_time = time.strftime('%H:%M:%S', time.gmtime(int(video_end_time)))
    end_time = str(end_time) + "." + str(round(video_end_time_mil, 3)).split(".")[1]

    video_start_time = time.strftime('%H:%M:%S', time.gmtime(video_start))
    video_start_time = str(video_start_time) + "." + str(round(video_start - int(video_start), 3)).split(".")[1]
    
    return video_path, video_start_time, end_time


def trim_this_video(data_path, vid_type):
    video_path, video_start_time, end_time = get_start_end_time(data_path, vid_type)
    if video_path == '' and video_start_time == 0 and end_time == 0:
        return
    
    trim_video(video_path, video_start_time, end_time)


def main(data_path):
    parser = argparse.ArgumentParser()

    trim_this_video(data_path, "iphone.mov")
    trim_this_video(data_path, "zed.mp4")

    # EXTRACT AUDIO
    #if not os.path.isfile(data_path + 'iphone_trimmed.mp3') or not os.path.isfile(data_path + 'iphone_trimmed.aac'):
    if not os.path.isfile(data_path + 'iphone_trimmed.aac'):
        cmd = 'ffmpeg -i ' + data_path + 'iphone_trimmed.mp4 -acodec aac ' + data_path + 'iphone_trimmed.aac'
        os.system(cmd)
    else:
        print("Skipping MP3/ AAC extraction...")

    # MUX Audio and Video
    if not os.path.isfile(data_path + 'output.mp4'):
        cmd = 'ffmpeg -i ' + data_path + 'zed_trimmed.mp4 -i ' + data_path + 'iphone_trimmed.aac -c:v copy -c:a aac -strict experimental ' + data_path + 'output.mp4'

        ### TRIMMING IN THE EARLIER BLOCK
        ### THEN USING THE MP3 and ZED_TRIMMED merge in the PREVIOUS LINE
        # #ffmpeg -ss 00:00:13.367 -i '/media/natsgld-sample/data/test/zed.mp4 -ss 00:00:13.367 -i '/media/natsgld-sample/data/test/zed2.mp4 -strict -2 -t 00:03:37.473 -i '/media/natsgld-sample/data/test/iphone_trimmed.mp3 -filter_complex "[0:v][1:v]vstack[t]" -map "[t]" -map 2:a -c:a aac -strict experimental -shortest '/media/natsgld-sample/data/test/output_final.mp4
        # _, video_start_time, end_time = get_start_end_time(data_path, "zed_0.mp4")
        # #+ ' -pix_fmt yuv420p' \
        # cmd ='ffmpeg' + ' -ss ' + str(video_start_time) + ' -i ' + data_path + 'zed_0.mp4' \
        #      + ' -ss ' + str(video_start_time) + ' -i ' + data_path + 'zed_1.mp4' \
        #      + ' -ss ' + str(video_start_time) + ' -i ' + data_path + 'zed_2.mp4' \
        #      + ' -strict -2' + ' -t ' + str(end_time) \
        #      + ' -i ' + data_path + 'iphone_trimmed.mp3' \
        #      + ' -filter_complex "[0:v][1:v]vstack[v1];[v1][2:v]vstack[t]" -map "[t]" -map 3:a -c:a aac -movflags +faststart -g 30 -strict experimental -shortest ' \
        #      + data_path + 'output_final_fast.mp4'
        # ### -tune zerolatency makes it not work for some reason
        # ### so far -g 30 helps and -movflags +faststart moves index to the front
        print(cmd)
        os.system(cmd)
    else:
        print("Skipping MUXing audio video...")

    # #DONE: Convert to a faster format
    # print('-------------------------------------------------------------------')
    # print('*   Optimizing video file format for fast loading...')
    # print('-------------------------------------------------------------------')
    # fps = 30
    # ext = 'mp4'
    # vid_out_file = data_path + 'output.mp4'
    # mp4_optimization_params = ' -f mp4 -c:v libx264 -movflags +faststart -g ' + str(fps) +' -tune zerolatency -profile:v baseline -pix_fmt yuv420p '
    # cmd = 'ffmpeg -i ' + vid_out_file + mp4_optimization_params + vid_out_file[0:-4] + '_fast.' + ext
    # os.system(cmd)

if __name__ == "__main__":

    data_path='/media/natsgld-sample/data/'
    for data_path in glob.glob('/media/natsgld-sample/data/'):
        print("---------- Processing " + data_path)
        main(data_path)
