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
from utils import quat_to_mat, init_fisheye_cam, read_calib

def calc_mean_dist_error(calib,
                         cam_front,
                         cam_left,
                         cam_right,
                         cam_rear,
                         pos_z_front,
                         pos_z_left,
                         pos_z_right,
                         pos_z_rear,
                         pts_img_front_left,
                         pts_img_front_right,
                         pts_img_rear_left,
                         pts_img_rear_right):
    t_front = [calib[0], calib[1], pos_z_front]
    R_front = quat_to_mat(calib[2:6])
    cam_front.update_extr(t_front, R_front)

    t_left = [calib[6], calib[7], pos_z_left]
    R_left = quat_to_mat(calib[8:12])
    cam_left.update_extr(t_left, R_left)

    t_right = [calib[12], calib[13], pos_z_right]
    R_right = quat_to_mat(calib[14:18])
    cam_right.update_extr(t_right, R_right)

    t_rear = [calib[18], calib[19], pos_z_rear]
    R_rear = quat_to_mat(calib[20:24])
    cam_rear.update_extr(t_rear, R_rear)

    distance = 0
    pts_img_front_fl = pts_img_front_left["front"]
    pts_img_left_fl = pts_img_front_left["left"]
    pts_img_front_fr = pts_img_front_right["front"]
    pts_img_right_fr = pts_img_front_right["right"]
    pts_img_rear_rl = pts_img_rear_left["rear"]
    pts_img_left_rl = pts_img_rear_left["left"]
    pts_img_rear_rr = pts_img_rear_right["rear"]
    pts_img_right_rr = pts_img_rear_right["right"]

    assert len(pts_img_front_fl) == len(pts_img_left_fl)
    assert len(pts_img_front_fr) == len(pts_img_right_fr)
    assert len(pts_img_rear_rl) == len(pts_img_left_rl)
    assert len(pts_img_rear_rr)  == len(pts_img_right_rr)

    num_pts = len(pts_img_front_fl) + len(pts_img_front_fr) + len(pts_img_rear_rl) + len(pts_img_rear_rr)

    if len(pts_img_front_fl) > 0:
        pts_world_front_fl = cam_front.project_2d_to_3d_ground(pts_img_front_fl)
        pts_world_left_fl = cam_left.project_2d_to_3d_ground(pts_img_left_fl)
        distance += np.linalg.norm(pts_world_front_fl - pts_world_left_fl, axis=1).sum()

    if len(pts_img_front_fr) > 0:
        pts_world_front_fr = cam_front.project_2d_to_3d_ground(pts_img_front_fr)
        pts_world_right_fr = cam_right.project_2d_to_3d_ground(pts_img_right_fr)
        distance += np.linalg.norm(pts_world_front_fr - pts_world_right_fr, axis=1).sum()

    if len(pts_img_rear_rl) > 0:
        pts_world_rear_rl = cam_rear.project_2d_to_3d_ground(pts_img_rear_rl)
        pts_world_left_rl = cam_left.project_2d_to_3d_ground(pts_img_left_rl)
        distance += np.linalg.norm(pts_world_rear_rl - pts_world_left_rl, axis=1).sum()

    if len(pts_img_rear_rr) > 0:
        pts_world_rear_rr = cam_rear.project_2d_to_3d_ground(pts_img_rear_rr)
        pts_world_right_rr = cam_right.project_2d_to_3d_ground(pts_img_right_rr)
        distance += np.linalg.norm(pts_world_rear_rr - pts_world_right_rr, axis=1).sum()

    mean_dist_error = distance / num_pts
    return mean_dist_error


if __name__ == '__main__':
    calib_f_front = "../calibrations/optimized/00164_FV.json"
    calib_f_left = "../calibrations/optimized/00165_MVL.json"
    calib_f_right = "../calibrations/optimized/00166_MVR.json"
    calib_f_rear = "../calibrations/optimized/00167_RV.json"

    pts_img_front_left = {
        "front": np.array(
            [(186, 585), (194, 591), (325, 493), (333, 495), (418, 444), (463, 417), (502, 402), (547, 384),
             (210, 469), (211, 458), (226, 454), (428, 403), (546, 369)]),
        "left": np.array(
            [(1048, 539), (1047, 555), (1092, 591), (1091, 607), (1119, 639), (1135, 651), (1146, 677), (1162, 704),
             (1057, 309), (1063, 297), (1074, 317), (1159, 550), (1187, 660)])}

    pts_img_front_right = {
        "front": np.array(
            [(939, 475), (856, 433), (865, 432), (815, 412), (978, 499), (1175, 559), (1137, 534), (1121, 551),
             (1130, 549), (1126, 619)]),
        "right": np.array(
            [(158, 583), (138, 618), (137, 606), (124, 639), (174, 566), (246, 330), (221, 356), (212, 422),
             (216, 400), (222, 538)])}

    pts_img_rear_left = {
        "rear": np.array(
            [(788, 350), (810, 370), (818, 369), (858, 410), (866, 409), (825, 360), (921, 469), (931, 467),
             (1019, 582), (1028, 576), (1061, 476), (1114, 513), (1158, 546)]),
        "left": np.array(
            [(240, 212), (247, 217), (252, 210), (263, 219), (270, 212), (267, 194), (285, 218), (290, 210),
             (317, 222), (325, 213), (452, 109), (512, 97), (571, 89)])}

    pts_img_rear_right = {
        "rear": np.array(
            [(325, 454), (338, 453), (389, 399), (446, 361), (456, 360), (420, 396), (449, 372), (487, 339),
             (504, 324), (512, 324), (598, 280), (555, 300)]),
        "right": np.array(
            [(967, 197), (980, 208), (995, 198), (1019, 202), (1027, 211), (1019, 220), (1030, 220), (1043, 212),
             (1047, 207), (1054, 214), (1105, 222), (1078, 216)])}

    intr_front, quat_front, t_front = read_calib(calib_f_front)
    intr_left, quat_left, t_left = read_calib(calib_f_left)
    intr_right, quat_right, t_right = read_calib(calib_f_right)
    intr_rear, quat_rear, t_rear = read_calib(calib_f_rear)

    pos_x_front, pos_y_front, pos_z_front = t_front
    pos_x_left, pos_y_left, pos_z_left = t_left
    pos_x_right, pos_y_right, pos_z_right = t_right
    pos_x_rear, pos_y_rear, pos_z_rear = t_rear

    cam_front = init_fisheye_cam(intr_front, quat_front, t_front)
    cam_left = init_fisheye_cam(intr_left, quat_left, t_left)
    cam_right = init_fisheye_cam(intr_right, quat_right, t_right)
    cam_rear = init_fisheye_cam(intr_rear, quat_rear, t_rear)

    calib = np.array([pos_x_front,
                          pos_y_front,
                          *quat_front,
                          pos_x_left,
                          pos_y_left,
                          *quat_left,
                          pos_x_right,
                          pos_y_right,
                          *quat_right,
                          pos_x_rear,
                          pos_y_rear,
                          *quat_rear])
    mean_dist_error = calc_mean_dist_error(calib,
                                           cam_front,
                                           cam_left,
                                           cam_right,
                                           cam_rear,
                                           pos_z_front,
                                           pos_z_left,
                                           pos_z_right,
                                           pos_z_rear,
                                           pts_img_front_left,
                                           pts_img_front_right,
                                           pts_img_rear_left,
                                           pts_img_rear_right)

    print("Mean distance error:", mean_dist_error)