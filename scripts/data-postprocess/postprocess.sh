# MIT License

# Copyright (c) 2025 Snehesh Shrestha

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

CURRENT_SESSIONS=$(ls -d session_* | grep '_[0-9]*_[0-9]')

max=0
for session in $CURRENT_SESSIONS
do
    #${string:position:length}
    n=${session:8:2}
    num=$(python -c "print int('$n')")
    ((num>max)) && max=$num
done

NEW_FOLDER=session_$(expr $max + 1)_$(date +%m%d%Y)/
echo SETTING UP NEW SESSION $NEW_FOLDER
cp -r TEMPLATE_session_ID_DATE $NEW_FOLDER
mv zed_*.bag $NEW_FOLDER
cd $NEW_FOLDER
ls

./rosbag2_#1_stream_v2.py &
sleep 1
./rosbag2_#2_json.py
 
