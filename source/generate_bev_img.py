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
from projection import Camera, create_bev_projection_maps, read_cam_from_json, bev_points_world_to_img
from matplotlib import pyplot as plt

def generate_bev_one_cam(source_cam: Camera, source_img: np.ndarray, bev_range: int, bev_size: int):
    map1, map2 = create_bev_projection_maps(source_cam, bev_range, bev_size)
    bev_image = cv2.remap(source_img, map1, map2, cv2.INTER_CUBIC)
    return bev_image

def generate_bev_all_cams(cam_front, cam_left, cam_right, cam_rear, img_front, img_left, img_right, img_rear,
                          overlay_opt='all', bev_range=25, bev_size=640):
    assert overlay_opt in ['fr', 'lr', 'all']

    bev_img_front = generate_bev_one_cam(cam_front, img_front, bev_range, bev_size)
    bev_img_left = generate_bev_one_cam(cam_left, img_left, bev_range, bev_size)
    bev_img_right = generate_bev_one_cam(cam_right, img_right, bev_range, bev_size)
    bev_img_rear = generate_bev_one_cam(cam_rear, img_rear, bev_range, bev_size)

    xy_world_front = cam_front.get_translation()[:2]
    xy_world_left = cam_left.get_translation()[:2]
    xy_world_right = cam_right.get_translation()[:2]
    xy_world_rear = cam_rear.get_translation()[:2]

    u_front_bev, v_front_bev = bev_points_world_to_img(bev_range, bev_size, xy_world_front)
    u_left_bev, v_left_bev = bev_points_world_to_img(bev_range, bev_size, xy_world_left)
    u_right_bev, v_right_bev = bev_points_world_to_img(bev_range, bev_size, xy_world_right)
    u_rear_bev, v_rear_bev = bev_points_world_to_img(bev_range, bev_size, xy_world_rear)

    bev_img_all = np.zeros(bev_img_front.shape).astype(np.uint8)
    if overlay_opt == 'lr':
        bev_img_all[0:v_front_bev, :] = bev_img_front[0:v_front_bev, :]
        bev_img_all[v_rear_bev:bev_size, :] = bev_img_rear[v_rear_bev:bev_size, :]
        bev_img_all[:, 0:u_left_bev] = bev_img_left[:, 0:u_left_bev]
        bev_img_all[:, u_right_bev:bev_size] = bev_img_right[:, u_right_bev:bev_size]
    elif overlay_opt == 'fr':
        bev_img_all[:, 0:u_left_bev] = bev_img_left[:, 0:u_left_bev]
        bev_img_all[:, u_right_bev:bev_size] = bev_img_right[:, u_right_bev:bev_size]
        bev_img_all[0:v_front_bev, :] = bev_img_front[0:v_front_bev, :]
        bev_img_all[v_rear_bev:bev_size, :] = bev_img_rear[v_rear_bev:bev_size, :]
    else:
        bev_img_all = (bev_img_front.astype(np.float32) + bev_img_left.astype(np.float32) +
                       bev_img_right.astype(np.float32) + bev_img_rear.astype(np.float32)) / 4
        bev_img_all = ((bev_img_all / bev_img_all.max()) * 255).astype(np.uint8)

    return bev_img_all

if __name__ == '__main__':
    bev_range = 25
    bev_size = 960
    overlay_opt = "all" # Which images to take for overlaying zones, available options: fr: front & rear, lr: left & right, all: all 4 cams
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
    bev_img_all = generate_bev_all_cams(cam_front, cam_left, cam_right, cam_rear, fisheye_img_front, fisheye_img_left,
                                        fisheye_img_right, fisheye_img_rear, overlay_opt, bev_range, bev_size)
    plt.imshow(cv2.cvtColor(bev_img_all, cv2.COLOR_BGR2RGB))
    plt.show()


