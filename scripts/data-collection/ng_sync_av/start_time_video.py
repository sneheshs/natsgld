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

import cv2 as cv
import numpy as np
import argparse
from PIL import Image
import os

drawing = False
point1 = ()
point2 = ()


def click_crop(event, x, y, flags, params):
    global point1, point2, drawing

    if event == cv.EVENT_LBUTTONDOWN:
        if drawing is False:
            drawing = True
            point1 = (x, y)
            point2 = (x + 30, y + 10)

        else:
            drawing = False

            # elif event == cv.EVENT_LBUTTONDOWN:
            #     if drawing is True:
            #         point2 = (x, y)


# choose Came from 6 frames
def chooseCam(frame, k):
    row = int((k - 1) / 2)
    col = (k + 1) % 2
    result = frame[720 * row:720 * (row + 1), 1280 * col:1280 * (col + 1), :]
    return result


def color_changed(old_frame, frame):
    old_rgb = np.mean(old_frame[3], axis=0)
    new_rgb = np.mean(frame[3], axis=0)

    print(old_rgb, ' --> ', new_rgb)

    if (new_rgb[0] > 160 and new_rgb[2] < 60):
        return True

    else:
        return False


def video_start_time(path):
    if not os.path.exists(path):
        print("Skipping... ERROR file not found: " + path)
        exit()

    cap = cv.VideoCapture(path)
    if not cap.isOpened():
        print("Skipping... ERROR opening file.")
        exit()

    frame_num = 0

    ret, old_frame = cap.read()
    if not ret:
        print("Skipping... ERROR loading file: " + path)
        exit()

    old_frame = chooseCam(old_frame, 1)
    old_frame = cv.resize(old_frame, (1280, 720))
    frame_num += 1

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = chooseCam(frame, 1)
        frame = cv.resize(frame, (1280, 720))
        fps = cap.get(cv.CAP_PROP_FPS)
        timestamps = [cap.get(cv.CAP_PROP_POS_MSEC)]

        cv.namedWindow("Start Time")
        cv.setMouseCallback("Start Time", click_crop)
        mask = np.zeros_like(frame)

        if point1:
            mask = cv.circle(mask, point1, 1, (0, 255, 0))
        if point2:
            mask = cv.circle(mask, point2, 1, (0, 255, 0))

        if point1 and point2:
            mask = cv.rectangle(mask, point1, point2, (0, 255, 0))
            old_crop = old_frame[point1[1]:point2[1], point1[0]:point2[0]]
            old_crop = cv.resize(old_crop, (point2[0] - point1[0], point2[1] - point1[1]))
            cv.imshow("im", old_crop)
            frame_crop = frame[point1[1]:point2[1], point1[0]:point2[0]]

            if color_changed(old_crop, frame_crop) is True:
                # sec = int(frame_num / 30)
                # mil_sec = (frame_num % 30)*0.0333
                #
                # start_time = str(sec)+"."+str(round(mil_sec,4)).split(".")[1]
                return timestamps, frame_num

        img = cv.add(frame, mask)
        cv.imshow("Start Time", img)

        frame_num += 1
        old_frame = frame

        key = cv.waitKey(1)
        if key == 27:
            break

    cap.release()
    cv.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-video", type=str, help="path to video")
    args = parser.parse_args()
    
    if not os.path.isfile(args.video):
        print('Skipping start time generation, ' + args.video + ' file doesnt exist!!!')
        exit()

    if os.path.isfile(args.video[:-3]+'txt'):
        print('Skipping start time generation, ' + args.video[:-3]+'txt' + ' already exisits...')
        exit()

    start_ret, frame_num = video_start_time(args.video)
    start_time = start_ret[0]
    # print(start_time/1000, frame_num)
    f = open(args.video[:-3]+'txt', "w+")
    f.write(str(start_time / 1000) + "\n" + str(frame_num) + "\n")
    f.close


if __name__ == "__main__":
    main()
