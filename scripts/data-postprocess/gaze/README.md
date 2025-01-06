# Gaze
This repository uses the [L2CS-Net](https://github.com/Ahmednull/L2CS-Net) pipeline to generate gaze data.


#### Demo Video [Download](../../../demo/NATSGLD_Demo_Gaze.mp4)
[![NatSGLD Gaze Demo](https://img.youtube.com/vi/Nd5ytJ4pWWc/0.jpg)](https://youtube.com/shorts/Nd5ytJ4pWWc)

## Setup Instructions

### Files to Copy
Copy `l2cs` and `models` folder into the scripts > data-postprocess > gaze.

### Modifications in `pipeline.py`
To ensure proper functionality, update the following line in `pipeline.py`:

#### Before:
```python
from face_detection import RetinaFace
```

#### After:
```python
from batch_face import RetinaFace
```

For the batch_face python module, you might need to install `batch_face` using `pip install batch_face`.

Additionally, apply the following fix based on this [pull request](https://github.com/Ahmednull/L2CS-Net/pull/23), which resolves issues related to handling empty results:

```python
# Predict gaze
if len(face_imgs) > 0:
    pitch, yaw = self.predict_gaze(np.stack(face_imgs))
else:
    pitch = np.empty((0, 1))
    yaw = np.empty((0, 1))
else:
    pitch = np.empty((0,1))
    yaw = np.empty((0,1))

else:
    pitch, yaw = self.predict_gaze(frame)

# Save data
results = GazeResultContainer(
    pitch=pitch,
    yaw=yaw,
    bboxes=np.stack(bboxes) if len(bboxes) > 0 else np.empty((0, 4)),
    landmarks=np.stack(landmarks) if len(landmarks) > 0 else np.empty((0, 5, 2)),
    scores=np.stack(scores) if len(scores) > 0 else np.empty((0,))
)

return results
```

## Generating Gaze Data

To generate gaze data from input videos, run the following command:

```bash
python3 generate_gaze.py -i data/videos
```

- Replace `data/videos` with the path to your video directory.
- The results will be saved in the `results` folder as `.npz` files.

## Visualizing Gaze Data

To validate and visualize the generated gaze data, run:

```bash
python3 validate_and_visualize.py
```

## Output
- **Generated Files:** Gaze data in `.npz` format, saved in the `results` folder.
- **Visualization:** Displays processed results for validation and analysis.

## References
- [L2CS-Net Repository](https://github.com/Ahmednull/L2CS-Net)
- [Relevant Pull Request for Fixes](https://github.com/Ahmednull/L2CS-Net/pull/23)
