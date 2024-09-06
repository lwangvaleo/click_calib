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

import os
import numpy as np
from scipy.optimize import minimize
from utils import quat_to_mat, init_fisheye_cam, read_calib, write_calib

def optimizer(calib,
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

    assert len(pts_img_front_fl) == len(pts_img_left_fl) and len(pts_img_front_fl) > 0
    assert len(pts_img_front_fr) == len(pts_img_right_fr) and len(pts_img_front_fr) > 0
    assert len(pts_img_rear_rl) == len(pts_img_left_rl) and len(pts_img_rear_rl) > 0
    assert len(pts_img_rear_rr)  == len(pts_img_right_rr) and len(pts_img_rear_rr) > 0

    num_pts = len(pts_img_front_fl) + len(pts_img_front_fr) + len(pts_img_rear_rl) + len(pts_img_rear_rr)

    pts_world_front_fl = cam_front.project_2d_to_3d_ground(pts_img_front_fl)
    pts_world_left_fl = cam_left.project_2d_to_3d_ground(pts_img_left_fl)
    distance += np.linalg.norm(pts_world_front_fl - pts_world_left_fl, axis=1).sum()

    pts_world_front_fr = cam_front.project_2d_to_3d_ground(pts_img_front_fr)
    pts_world_right_fr = cam_right.project_2d_to_3d_ground(pts_img_right_fr)
    distance += np.linalg.norm(pts_world_front_fr - pts_world_right_fr, axis=1).sum()

    pts_world_rear_rl = cam_rear.project_2d_to_3d_ground(pts_img_rear_rl)
    pts_world_left_rl = cam_left.project_2d_to_3d_ground(pts_img_left_rl)
    distance += np.linalg.norm(pts_world_rear_rl - pts_world_left_rl, axis=1).sum()

    pts_world_rear_rr = cam_rear.project_2d_to_3d_ground(pts_img_rear_rr)
    pts_world_right_rr = cam_right.project_2d_to_3d_ground(pts_img_right_rr)
    distance += np.linalg.norm(pts_world_rear_rr - pts_world_right_rr, axis=1).sum()

    mde = distance / num_pts
    print(f"Mean distance error: {mde}")

    return mde


if __name__ == '__main__':
    calib_f_front = "../calibrations/original/00164_FV.json"
    calib_f_left = "../calibrations/original/00165_MVL.json"
    calib_f_right = "../calibrations/original/00166_MVR.json"
    calib_f_rear = "../calibrations/original/00167_RV.json"
    calib_save_root = "../calibrations/optimized"

    # Put your clicked keypoints here
    pts_img_front_left = {
        "front": np.array([(186, 585), (194, 591), (325, 493), (333, 495), (418, 444), (463, 417), (502, 402), (547, 384),
                           (210, 469), (211, 458), (226, 454), (428, 403), (546, 369)]),
        "left": np.array([(1048, 539), (1047, 555), (1092, 591), (1091, 607), (1119, 639), (1135, 651), (1146, 677), (1162, 704),
                          (1057, 309), (1063, 297), (1074, 317), (1159, 550), (1187, 660)])}

    pts_img_front_right = {
        "front": np.array([(939, 475), (856, 433), (865, 432), (815, 412), (978, 499), (1175, 559), (1137, 534), (1121, 551),
                           (1130, 549), (1126, 619)]),
        "right": np.array([(158, 583), (138, 618), (137, 606), (124, 639), (174, 566), (246, 330), (221, 356), (212, 422),
                           (216, 400), (222, 538)])}

    pts_img_rear_left = {
        "rear": np.array([(788, 350), (810, 370), (818, 369), (858, 410), (866, 409), (825, 360), (921, 469), (931, 467),
                          (1019, 582), (1028, 576), (1061, 476), (1114, 513), (1158, 546)]),
        "left": np.array([(240, 212), (247, 217), (252, 210), (263, 219), (270, 212), (267, 194), (285, 218), (290, 210),
                          (317, 222), (325, 213), (452, 109), (512, 97), (571, 89)])}

    pts_img_rear_right = {
        "rear": np.array([(325, 454), (338, 453), (389, 399), (446, 361), (456, 360), (420, 396), (449, 372), (487, 339),
                          (504, 324), (512, 324), (598, 280), (555, 300)]),
        "right": np.array([(967, 197), (980, 208), (995, 198), (1019, 202), (1027, 211), (1019, 220), (1030, 220), (1043, 212),
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

    calib_ini = np.array([pos_x_front,
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
    func_optimize = lambda calib: optimizer(calib,
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

    res_multi_cam = minimize(func_optimize, calib_ini, method='BFGS')
    final_calib = res_multi_cam.x.tolist()
    print("Optimized mean distance error:", res_multi_cam.fun)
    # Save to files
    final_t_front = final_calib[0:2] + [pos_z_front]
    final_quat_front = final_calib[2:6]
    final_t_left = final_calib[6:8] + [pos_z_left]
    final_quat_left = final_calib[8:12]
    final_t_right = final_calib[12:14] + [pos_z_right]
    final_quat_right = final_calib[14:18]
    final_t_rear = final_calib[18:20] + [pos_z_rear]
    final_quat_rear = final_calib[20:24]
    write_calib(intr_front, final_quat_front, final_t_front, os.path.join(calib_save_root, "00164_FV.json"))
    write_calib(intr_left, final_quat_left, final_t_left, os.path.join(calib_save_root, "00165_MVL.json"))
    write_calib(intr_right, final_quat_right, final_t_right, os.path.join(calib_save_root, "00166_MVR.json"))
    write_calib(intr_rear, final_quat_rear, final_t_rear, os.path.join(calib_save_root, "00167_RV.json"))