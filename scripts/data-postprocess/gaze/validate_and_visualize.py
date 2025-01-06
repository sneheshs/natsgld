import cv2
import numpy as np
import glob
import csv

GENERATE_VIDEO_FILES = True

## Imported from https://github.com/Ahmednull/L2CS-Net
class GazeResultContainer:
    pitch: np.ndarray
    yaw: np.ndarray
    bboxes: np.ndarray
    landmarks: np.ndarray
    scores: np.ndarray

def draw_gaze(a,b,c,d,image_in, pitchyaw, thickness=2, color=(255, 255, 0),sclae=2.0):
    """Draw gaze angle on given image with a given eye positions."""
    image_out = image_in
    (h, w) = image_in.shape[:2]
    length = c
    pos = (int(a+c / 2.0), int(b+d / 2.0))
    if len(image_out.shape) == 2 or image_out.shape[2] == 1:
        image_out = cv2.cvtColor(image_out, cv2.COLOR_GRAY2BGR)
    dx = -length * np.sin(pitchyaw[0]) * np.cos(pitchyaw[1])
    dy = -length * np.sin(pitchyaw[1])
    cv2.arrowedLine(image_out, tuple(np.round(pos).astype(np.int32)),
                   tuple(np.round([pos[0] + dx, pos[1] + dy]).astype(int)), color,
                   thickness, cv2.LINE_AA, tipLength=0.18)
    return image_out

def draw_bbox(frame: np.ndarray, bbox: np.ndarray):
    
    x_min=int(bbox[0])
    if x_min < 0:
        x_min = 0
    y_min=int(bbox[1])
    if y_min < 0:
        y_min = 0
    x_max=int(bbox[2])
    y_max=int(bbox[3])

    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0,255,0), 1)

    return frame

def render(frame: np.ndarray, results):

    # Draw bounding boxes
    for bbox in results.bboxes:
        frame = draw_bbox(frame, bbox)

    # Draw Gaze
    for i in range(results.pitch.shape[0]):

        bbox = results.bboxes[i]
        pitch = results.pitch[i]
        yaw = results.yaw[i]
        
        # Extract safe min and max of x,y
        x_min=int(bbox[0])
        if x_min < 0:
            x_min = 0
        y_min=int(bbox[1])
        if y_min < 0:
            y_min = 0
        x_max=int(bbox[2])
        y_max=int(bbox[3])

        # Compute sizes
        bbox_width = x_max - x_min
        bbox_height = y_max - y_min

        draw_gaze(x_min,y_min,bbox_width, bbox_height,frame,(pitch,yaw),color=(0,0,255))

    return frame

def load_csv(metadata_path):
    dbsn_dict = {}
    with open(metadata_path, mode='r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            dbsn_value = row['DBSN'].strip()
            sid_value = row['SID'].strip()
            dbsn_dict[sid_value] = dbsn_value
    return dbsn_dict

def find_filename(video_path, sid, pid):
    sid = str(sid).zfill(2)
    search_string = video_path + f"P{pid}-{sid}-*.mp4"
    files = glob.glob(search_string)
    print(files)
    if len(files) == 0:  # No matching file
        # raise FileNotFoundError(f"No files found matching: {search_string}")
        return None
    elif len(files) > 1:  # Multiple matches
        print(f"Warning: Multiple files found, using the first one: {files[0]}")
    return files[0]  # Return the first match


## NATSGLD Implementatoin
video_path = "../../../data/videos/"
data = np.load('../../../data/gaze_data.npz', allow_pickle=True)
metadata_path = "../../../data/natsgld_metadata_v1.1.csv"
dbsn_dict = load_csv(metadata_path)
gaze_data = data['gaze_data'].item()  # numpy.ndarray to dictionary

current_gaze = GazeResultContainer()

for pid in gaze_data.keys():
    print(f"PID: {pid}")
    for result in gaze_data[pid]["results"]:
        sid = result['sid']
        print(f"  SID: {sid}")
        data = result['data']
        print(result['data'][0])
        dbsn = dbsn_dict[sid]
        filename = find_filename(video_path, sid, pid)

        if filename is None:
            continue

        cap = cv2.VideoCapture(filename)

        if not cap.isOpened():
            raise IOError("Cannot open webcam")

        if GENERATE_VIDEO_FILES:
            # Output processed video file
            output_file = filename[:-4] + "_gaze.mp4"
            fps = cap.get(cv2.CAP_PROP_FPS)
            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(output_file, fourcc, fps if fps > 0 else 10, (frame_width, frame_height))

        for res in result['data']:

            current_gaze = res
            # print(current_gaze.yaw)

            # Get frame
            success, frame = cap.read()
            if not success:
                break
        
            frame = render(frame, current_gaze)

            if GENERATE_VIDEO_FILES:
                out.write(frame)

            cv2.imshow("Demo",frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                if GENERATE_VIDEO_FILES:
                    out.release()
                quit

        if GENERATE_VIDEO_FILES:
            out.release()
            cap.release()

        

