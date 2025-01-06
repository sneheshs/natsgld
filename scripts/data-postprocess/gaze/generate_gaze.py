import argparse
import pathlib
import numpy as np
import cv2
import time

import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from l2cs.utils import select_device
from l2cs.pipeline import Pipeline

import os
from glob import glob
import csv


# DEFAULT_WEBCAM: Default camera source index. '0' usually refers to the built-in webcam.
DEFAULT_WEBCAM = 0

CWD = pathlib.Path.cwd()

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(
        description='Gaze evalution using model pretrained with L2CS-Net on Gaze360.')
    parser.add_argument(
        '--device',dest='device', help='Device to run model: cpu or gpu:0',
        default="cpu", type=str)
    parser.add_argument(
        '--snapshot',dest='snapshot', help='Path of model snapshot.', 
        default='output/snapshots/L2CS-gaze360-_loader-180-4/_epoch_55.pkl', type=str)
    parser.add_argument(
        '--cam',dest='cam_id', help='Camera device id to use [0]',  
        default=0, type=int)
    parser.add_argument(
        '--arch',dest='arch',help='Network architecture, can be: ResNet18, ResNet34, ResNet50, ResNet101, ResNet152',
        default='ResNet50', type=str)
    parser.add_argument(
        "-i", "--input", type=str, required=True, help="Camera source (index) or path to an offline video file", default=str(DEFAULT_WEBCAM)), 
    parser.add_argument(
        "-o", "--output", type=str, help="Output directory for processed videos and data", default="./results")


    args = parser.parse_args()
    return args

def load_csv(metadata_path):
    sid_dict = {}
    with open(metadata_path, mode='r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            dbsn_value = row['DBSN'].strip()
            sid_value = row['SID'].strip()
            sid_dict[dbsn_value] = sid_value
    return sid_dict

def parse_file(video_path):
    # Extract PID and SID from filename
    filename = os.path.basename(video_path)
    parts = filename.split('-')
    pid = parts[0][1:]  # Remove 'P' -> '40'
    dbsn = os.path.splitext(parts[2])[0]  # Remove '.mp4' -> '1234'
    dbsn = str(int(dbsn))
    if dbsn not in sid_dict:
        print(f"[WARNING] dbsn '{dbsn}' not found in CSV. Skipping {video_path}")
        return None
    else: 
        sid = sid_dict[dbsn]
    return pid, sid, dbsn

def process_video(video_path, output_dir, arch, device, gaze_data, sid_dict):
    """
    Process a single video file for gaze estimation and store results.
    """
    pid, sid, dbsn = parse_file(video_path)

    # Initialize pipeline with PID and SID
    gaze_pipeline = Pipeline(
        weights=CWD / 'models' / 'L2CSNet_gaze360.pkl',
        arch='ResNet50',
        device = select_device(args.device, batch_size=1), 
        # pid = pid,
        # sid = sid
    )

    # Open video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Unable to open video {video_path}")
        return

    # Create or update PID entry in consolidated data
    if pid not in gaze_data:
        gaze_data[pid] = {"results": []}

    # Find or create an entry keyed by SID in the data
    sid_entry = next((entry for entry in gaze_data[pid]["results"] if entry["sid"] == sid), None)
    if sid_entry is None:
        sid_entry = {"sid": sid, "data": []}
        gaze_data[pid]["results"].append(sid_entry)

    # Output processed video file
    # output_file = os.path.join(output_dir, f"processed_P{pid}_S{sid}.mp4")
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter(output_file, fourcc, fps if fps > 0 else 10, (frame_width, frame_height))

    # Process frames
    with torch.no_grad():
        while True:
            success, frame = cap.read()
            if not success or frame is None:
                print(f"End of video {video_path}")
                break

            start_fps = time.time()
            current_results = gaze_pipeline.step(frame)

            if current_results:
                # We found valid results (face detected), so store them
                sid_entry["data"].append(current_results)
                last_results = current_results
            else:
                # No valid results returned; reuse last known results
                if last_results is not None:
                    sid_entry["data"].append(last_results)
                else:
                    # If no last_results yet, store None or skip
                    sid_entry["data"].append(None)


    # Release resources
    cap.release()

if __name__ == '__main__':
    args = parse_args()
    
    metadata_path = "/home/saketh/Desktop/natsgld-sample/data/natsgld_metadata_v1.1.csv"

    sid_dict =  load_csv(metadata_path)
    # Initialize output directory
    results = args.output
    os.makedirs(results, exist_ok=True)

    cudnn.enabled = True
    arch=args.arch
    cam = args.cam_id
    # snapshot_path = args.snapshot

    # Initialize storage for gaze data grouped by PID
    gaze_data = {}
    
    # Handle single video or directory
    if os.path.isdir(args.input):
        # Process all MP4 videos in the directory
        video_files = sorted(glob(os.path.join(args.input, "*.mp4")))
    else:
        # Process a single video
        video_files = [args.input]

    output_npz = os.path.join(results, "gaze_data.npz")
    if os.path.exists(output_npz):
        print("Loading Previous Data") 
        loaded_data = np.load(output_npz, allow_pickle=True)
        print(loaded_data)
        if 'gaze_data' in loaded_data:
            gaze_data = loaded_data['gaze_data'].item()
            print("Existing gaze_data loaded.")
        else:
            print("gaze_data key not found in the npz file. Starting fresh.")
    else:
        print("No existing gaze_data.npz found. Starting fresh.")

    # Process each video file
    for video_path in video_files:
        pid, sid, dbsn = parse_file(video_path)
        print(f"Processing Video: PID={pid}, SID={sid}, DBSN={dbsn}")
        # print(gaze_data)
        # Check if pid exists and sid exists under pid
        for pid_vals, details in gaze_data.items():
            for res in details['results']:
                sid_vals = res['sid']
                if pid in pid_vals and sid in sid_vals:                
                    print(f"Skipping {video_path} as PID={pid} and SID={sid} already exist.")
                    continue

        # If not skipped, process the video
        print(f"Processing Video: {video_path}")
        process_video(video_path, results, arch, args.device, gaze_data, sid_dict)

        # After processing, save consolidated data
        output_npz = os.path.join(results, "gaze_data.npz")
        np.savez_compressed(output_npz, gaze_data=gaze_data)
        print(f"Gaze Data saved in NPZ format: {output_npz}")