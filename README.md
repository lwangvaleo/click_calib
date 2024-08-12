# Click-Calib: A Robust Extrinsic Calibration Method for Surround-View Systems

This is the official code repository for our paper Click-Calib.

![Click-Calib](assets/click-calib.png)

## Requirements

Please refer to requirements.txt

## How to use

### Step 1: Initialize extrinsic calibration

To ensure the optimization convergence, a initial guess of the Surround-View System (SVS) calibration needs to be provided. You can just take the nominal pose of each camera, or manually adjust each camera's pose to achieve a reasonably good BEV image. We will provide this manual adjustment tool later.

### Step 2: Select keypoints
Use the click_points.py script to click keypoints in each pair of adjacent camera images. Ensure both images have an equal number of selected keypoints. When you finish clicking, just close the GUI window and the selected keypoints will be printed out. To achieve good calibration, at least 10 points need be selected for each pair of adjacent cameras.

### Step 3: Optimize

Copy and paste the kepoints from click_points.py to optimize.py, then run optimize.py. The optimization process should take about 5 to 30 seconds. If it takes too long time or results in a large Mean Distance Error (MDE), this indicate a failure to converge. In such cases, check your initial extrinsics or other settings (e.g., number of selected keypoints).

### (Optional) Step 4: Generate BEV images

For qualitative evaluation, use generate_bev_img.py to create BEV images from SVS images. It overlays all pixels reprojected from each camera, so better calibration will yield better alignment while poor calibration will have more "ghosting" effect.

### (Optional) Step 5: Metric calculation

For quantitative evaluation, use eval.py to compute the MDE metric on your test frames.


## Citation
If you use this code for your research or other publication purposes, please cite:

**Click-Calib: A Robust Extrinsic Calibration Method for Surround-View Systems**

Bibtex:
```
@inproceedings{
    todo
}
```