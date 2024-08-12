# Copyright 2024 Valeo Brain Division and contributors
#
# Author: Lihao Wang <lihao.wang@valeo.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import numpy as np
import cv2
from projection import Camera, read_cam_from_json
from matplotlib import pyplot as plt

def create_bev_projection_maps(source_cam: Camera, bev_range: int, bev_size: int):
    """
    bev_range: in meters
    bev_size: in pixels
    """
    u_map = np.zeros((bev_size, bev_size, 1), dtype=np.float32)
    v_map = np.zeros((bev_size, bev_size, 1), dtype=np.float32)
    scale_pxl_to_meter = bev_range / bev_size

    bev_points_v = np.arange(bev_size)
    bev_points_world_z = np.zeros(bev_size)

    for u_px in range(bev_size):
        bev_points_u = np.ones(bev_size) * u_px
        bev_points_world_x = bev_range / 2 - bev_points_v * scale_pxl_to_meter
        bev_points_world_y = bev_range / 2 - bev_points_u * scale_pxl_to_meter
        bev_points_world = np.column_stack((bev_points_world_x, bev_points_world_y, bev_points_world_z))
        source_points = source_cam.project_3d_to_2d(bev_points_world)
        u_map.T[0][u_px] = source_points.T[0]
        v_map.T[0][u_px] = source_points.T[1]

    map1, map2 = cv2.convertMaps(u_map, v_map, dstmap1type=cv2.CV_16SC2, nninterpolation=False)
    return map1, map2

def generate_bev(source_cam: Camera, source_img: np.ndarray, bev_range: int, bev_size: int):
    map1, map2 = create_bev_projection_maps(source_cam, bev_range, bev_size)
    bev_image = cv2.remap(source_img, map1, map2, cv2.INTER_CUBIC)
    return bev_image

if __name__ == '__main__':
    bev_range = 25
    bev_size = 960
    calib_f_front = "../calibrations/optimized/00164_FV.json"
    calib_f_left = "../calibrations/optimized/00165_MVL.json"
    calib_f_right = "../calibrations/optimized/00166_MVR.json"
    calib_f_rear = "../calibrations/optimized/00167_RV.json"
    fisheye_img_front = cv2.imread("../images/fisheye/00164_FV.png")
    fisheye_img_left = cv2.imread("../images/fisheye/00165_MVL.png")
    fisheye_img_right = cv2.imread("../images/fisheye/00166_MVR.png")
    fisheye_img_rear = cv2.imread("../images/fisheye/00167_RV.png")
    cam_front = read_cam_from_json(calib_f_front)
    cam_left = read_cam_from_json(calib_f_left)
    cam_right = read_cam_from_json(calib_f_right)
    cam_rear = read_cam_from_json(calib_f_rear)
    bev_img_front = generate_bev(cam_front, fisheye_img_front, bev_range, bev_size)
    bev_img_left = generate_bev(cam_left, fisheye_img_left, bev_range, bev_size)
    bev_img_right = generate_bev(cam_right, fisheye_img_right, bev_range, bev_size)
    bev_img_rear = generate_bev(cam_rear, fisheye_img_rear, bev_range, bev_size)
    bev_img_all = (bev_img_front.astype(np.float32) + bev_img_left.astype(np.float32) +
                   bev_img_right.astype(np.float32) + bev_img_rear.astype(np.float32)) / 4
    bev_img_all = ((bev_img_all / bev_img_all.max()) * 255).astype(np.uint8)
    plt.imshow(cv2.cvtColor(bev_img_all, cv2.COLOR_BGR2RGB))
    plt.show()


