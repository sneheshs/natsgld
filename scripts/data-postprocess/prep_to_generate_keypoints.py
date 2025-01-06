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
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="Input filename")

    args = parser.parse_args()
    inputfile = args.input
    file = str(inputfile)
    path = file[:file.rfind('/')+1]

    mp4_optimization_params = ' -f mp4 -c:v h264_nvenc -zerolatency 1 -rc constqp -qp 19 -preset hq -rc-lookahead 32 -g 300 -pix_fmt yuv420p '


    print('************ ' + path + '**************')

    for ind in range(1, 7):
        print('>>>>>>>>>>> Processing camera ' + str(ind))
        
        out_file = path + 'tmp'+ str(ind) + '_output' + str(ind) + '.mp4'

        #os.system('python3 segment_video.py -f '+ file +' -cam ' + str(ind) + ' -name tmp' + str(ind))
        row = int((ind - 1) / 2)
        col = (ind + 1) % 2
        x = 1280 * col
        y = 720 * row
        #print( str(x), str(y), "1280, 720")

        if not os.path.isfile(out_file):
        	os.system('ffmpeg -i ' + file + ' -filter:v "crop=1280:720:' + str(x) + ':' + str(y) + '" ' + mp4_optimization_params + out_file)
        else:
            print("... Skipping already exists!")


