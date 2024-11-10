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

import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, RadioButtons, Button
import os
from scipy.spatial.transform import Rotation as SciRot
import cv2
from utils import init_fisheye_cam, read_calib, write_calib
from generate_bev_img import generate_bev_all_cams


if __name__ == '__main__':
    """
    How to use:
    Adjust the extrinsic parameters, especially rotation angles, for each camera to create a reasonably good BEV image. 
    Use the left-right / front-rear toggle button to switch between BEV images overlaid from different cameras. 
    Once finished, click 'Export to files' button to save calibrations to files. Then you can use the saved calibrations
    as the initial files for optimize.py.
    """
    ini_calib_dir = "../calibrations/fisheye/creteil_learning_car_manual_calib_ini"
    # Original calibrations are used only to get intrinsics and camera heights
    calib_ori_f_front = "../calibrations/original/00164_FV.json"
    calib_ori_f_left = "../calibrations/original/00165_MVL.json"
    calib_ori_f_right = "../calibrations/original/00166_MVR.json"
    calib_ori_f_rear = "../calibrations/original/00167_RV.json"
    img_front = cv2.imread("../images/fisheye/00164_FV.png")
    img_left = cv2.imread("../images/fisheye/00165_MVL.png")
    img_right = cv2.imread("../images/fisheye/00166_MVR.png")
    img_rear = cv2.imread("../images/fisheye/00167_RV.png")
    calib_export_dir = "../calibrations/initial"
    overlay_opt = 'lr'

    if not os.path.exists(calib_export_dir):
        os.makedirs(calib_export_dir)

    intr_front, _, t_front = read_calib(calib_ori_f_front)
    intr_left, _, t_left = read_calib(calib_ori_f_left)
    intr_right, _, t_right = read_calib(calib_ori_f_right)
    intr_rear, _, t_rear = read_calib(calib_ori_f_rear)

    # Replace translation xy by their nominal values
    pos_x_front, pos_y_front = 3.7, 0
    pos_x_left, pos_y_left = 2, 1
    pos_x_right, pos_y_right = 2, -1
    pos_x_rear, pos_y_rear = -1, 0
    pos_z_front = t_front[2]
    pos_z_left = t_left[2]
    pos_z_right = t_right[2]
    pos_z_rear = t_rear[2]
    t_front = [pos_x_front, pos_y_front, pos_z_front]
    t_left = [pos_x_left, pos_y_left, pos_z_left]
    t_right = [pos_x_right, pos_y_right, pos_z_right]
    t_rear = [pos_x_rear, pos_y_rear, pos_z_rear]

    # Replace Euler angles by their typical values
    rot_z1_front, rot_x_front, rot_z2_front = 180, 90, 90
    quat_front = SciRot.from_euler('zxz', [rot_z1_front, rot_x_front, rot_z2_front], degrees=True).as_quat()
    rot_z1_left, rot_x_left, rot_z2_left = 180, 180, -180
    quat_left = SciRot.from_euler('zxz', [rot_z1_left, rot_x_left, rot_z2_left], degrees=True).as_quat()
    rot_z1_right, rot_x_right, rot_z2_right = -180, 180, 0
    quat_right = SciRot.from_euler('zxz', [rot_z1_right, rot_x_right, rot_z2_right], degrees=True).as_quat()
    rot_z1_rear, rot_x_rear, rot_z2_rear = 180, 90, -90
    quat_rear = SciRot.from_euler('zxz', [rot_z1_rear, rot_x_rear, rot_z2_rear], degrees=True).as_quat()

    cam_front = init_fisheye_cam(intr_front, quat_front, t_front)
    cam_left = init_fisheye_cam(intr_left, quat_left, t_left)
    cam_right = init_fisheye_cam(intr_right, quat_right, t_right)
    cam_rear = init_fisheye_cam(intr_rear, quat_rear, t_rear)

    fig, ax = plt.subplots()
    plt.subplots_adjust(left=0.5, right=0.9, top=0.8, bottom=0.2)
    ax.axis('off')
    topview = generate_bev_all_cams(cam_front, cam_left, cam_right, cam_rear, img_front, img_left, img_right, img_rear, overlay_opt)
    im = ax.imshow(cv2.cvtColor(topview, cv2.COLOR_BGR2RGB))

    def update_calib(val):
        overlay_opt = 'lr' if menu_topview_opt.value_selected == 'left-right' else 'fr'
        pos_x_0 = float(text_pos_x_0.text)
        pos_y_0 = float(text_pos_y_0.text)
        rot_z1_0 = float(text_rot_z1_0.text)
        rot_x_0 = float(text_rot_x_0.text)
        rot_z2_0 = float(text_rot_z2_0.text)
        pos_x_1 = float(text_pos_x_1.text)
        pos_y_1 = float(text_pos_y_1.text)
        rot_z1_1 = float(text_rot_z1_1.text)
        rot_x_1 = float(text_rot_x_1.text)
        rot_z2_1 = float(text_rot_z2_1.text)
        pos_x_2 = float(text_pos_x_2.text)
        pos_y_2 = float(text_pos_y_2.text)
        rot_z1_2 = float(text_rot_z1_2.text)
        rot_x_2 = float(text_rot_x_2.text)
        rot_z2_2 = float(text_rot_z2_2.text)
        pos_x_3 = float(text_pos_x_3.text)
        pos_y_3 = float(text_pos_y_3.text)
        rot_z1_3 = float(text_rot_z1_3.text)
        rot_x_3 = float(text_rot_x_3.text)
        rot_z2_3 = float(text_rot_z2_3.text)
        t_front[0] = pos_x_0
        t_front[1] = pos_y_0
        R_front = SciRot.from_euler('zxz', [rot_z1_0, rot_x_0, rot_z2_0], degrees=True).as_matrix()
        cam_front.update_extr(t_front, R_front)
        t_left[0] = pos_x_1
        t_left[1] = pos_y_1
        R_left = SciRot.from_euler('zxz', [rot_z1_1, rot_x_1, rot_z2_1], degrees=True).as_matrix()
        cam_left.update_extr(t_left, R_left)
        t_right[0] = pos_x_2
        t_right[1] = pos_y_2
        R_right = SciRot.from_euler('zxz', [rot_z1_2, rot_x_2, rot_z2_2], degrees=True).as_matrix()
        cam_right.update_extr(t_right, R_right)
        t_rear[0] = pos_x_3
        t_rear[1] = pos_y_3
        R_rear = SciRot.from_euler('zxz', [rot_z1_3, rot_x_3, rot_z2_3], degrees=True).as_matrix()
        cam_rear.update_extr(t_rear, R_rear)
        topview = generate_bev_all_cams(cam_front, cam_left, cam_right, cam_rear, img_front, img_left, img_right, img_rear, overlay_opt)
        im.set_data(cv2.cvtColor(topview, cv2.COLOR_BGR2RGB))
        fig.canvas.draw_idle()

    def export_calib(event):
        quat_front = SciRot.from_matrix(cam_front.get_rotation()).as_quat().tolist()
        quat_left = SciRot.from_matrix(cam_left.get_rotation()).as_quat().tolist()
        quat_right = SciRot.from_matrix(cam_right.get_rotation()).as_quat().tolist()
        quat_rear = SciRot.from_matrix(cam_rear.get_rotation()).as_quat().tolist()
        write_calib(intr_front, quat_front, t_front, os.path.join(calib_export_dir, "00164_FV.json"))
        write_calib(intr_left, quat_left, t_left, os.path.join(calib_export_dir, "00165_MVL.json"))
        write_calib(intr_right, quat_right, t_right, os.path.join(calib_export_dir, "00166_MVR.json"))
        write_calib(intr_rear, quat_rear, t_rear, os.path.join(calib_export_dir, "00167_RV.json"))

    box_topview_opt = plt.axes([0.10, 0.7, 0.1, 0.1], facecolor='linen')
    menu_topview_opt = RadioButtons(box_topview_opt, ('left-right', 'front-rear'))
    box_export_calib = plt.axes([0.10, 0.2, 0.08, 0.04], facecolor='linen')
    button_export_calib = Button(box_export_calib, 'Export to files')

    box_pos_x_0 = plt.axes([0.10, 0.6, 0.05, 0.03], facecolor='linen')
    text_pos_x_0 = TextBox(box_pos_x_0, 'Front: pos_x', initial=f"{pos_x_front:.2f}")
    box_pos_y_0 = plt.axes([0.19, 0.6, 0.05, 0.03], facecolor='linen')
    text_pos_y_0 = TextBox(box_pos_y_0, 'pos_y', initial=f"{pos_y_front:.2f}")
    box_rot_z1_0 = plt.axes([0.28, 0.6, 0.05, 0.03], facecolor='linen')
    text_rot_z1_0 = TextBox(box_rot_z1_0, 'rot_z1', initial=f"{rot_z1_front:.1f}")
    box_rot_x_0 = plt.axes([0.37, 0.6, 0.05, 0.03], facecolor='linen')
    text_rot_x_0 = TextBox(box_rot_x_0, 'rot_x', initial=f"{rot_x_front:.1f}")
    box_rot_z2_0 = plt.axes([0.46, 0.6, 0.05, 0.03], facecolor='linen')
    text_rot_z2_0 = TextBox(box_rot_z2_0, 'rot_z2', initial=f"{rot_z2_front:.1f}")

    box_pos_x_1 = plt.axes([0.10, 0.5, 0.05, 0.03], facecolor='linen')
    text_pos_x_1 = TextBox(box_pos_x_1, 'Left: pos_x', initial=f"{pos_x_left:.2f}")
    box_pos_y_1 = plt.axes([0.19, 0.5, 0.05, 0.03], facecolor='linen')
    text_pos_y_1 = TextBox(box_pos_y_1, 'pos_y', initial=f"{pos_y_left:.2f}")
    box_rot_z1_1 = plt.axes([0.28, 0.5, 0.05, 0.03], facecolor='linen')
    text_rot_z1_1 = TextBox(box_rot_z1_1, 'rot_z1', initial=f"{rot_z1_left:.1f}")
    box_rot_x_1 = plt.axes([0.37, 0.5, 0.05, 0.03], facecolor='linen')
    text_rot_x_1 = TextBox(box_rot_x_1, 'rot_x', initial=f"{rot_x_left:.1f}")
    box_rot_z2_1 = plt.axes([0.46, 0.5, 0.05, 0.03], facecolor='linen')
    text_rot_z2_1 = TextBox(box_rot_z2_1, 'rot_z2', initial=f"{rot_z2_left:.1f}")

    box_pos_x_2 = plt.axes([0.10, 0.4, 0.05, 0.03], facecolor='linen')
    text_pos_x_2 = TextBox(box_pos_x_2, 'Right: pos_x', initial=f"{pos_x_right:.2f}")
    box_pos_y_2 = plt.axes([0.19, 0.4, 0.05, 0.03], facecolor='linen')
    text_pos_y_2 = TextBox(box_pos_y_2, 'pos_y', initial=f"{pos_y_right:.2f}")
    box_rot_z1_2 = plt.axes([0.28, 0.4, 0.05, 0.03], facecolor='linen')
    text_rot_z1_2 = TextBox(box_rot_z1_2, 'rot_z1', initial=f"{rot_z1_right:.1f}")
    box_rot_x_2 = plt.axes([0.37, 0.4, 0.05, 0.03], facecolor='linen')
    text_rot_x_2 = TextBox(box_rot_x_2, 'rot_x', initial=f"{rot_x_right:.1f}")
    box_rot_z2_2 = plt.axes([0.46, 0.4, 0.05, 0.03], facecolor='linen')
    text_rot_z2_2 = TextBox(box_rot_z2_2, 'rot_z2', initial=f"{rot_z2_right:.1f}")

    box_pos_x_3 = plt.axes([0.10, 0.3, 0.05, 0.03], facecolor='linen')
    text_pos_x_3 = TextBox(box_pos_x_3, 'Rear: pos_x', initial=f"{pos_x_rear:.2f}")
    box_pos_y_3 = plt.axes([0.19, 0.3, 0.05, 0.03], facecolor='linen')
    text_pos_y_3 = TextBox(box_pos_y_3, 'pos_y', initial=f"{pos_y_rear:.2f}")
    box_rot_z1_3 = plt.axes([0.28, 0.3, 0.05, 0.03], facecolor='linen')
    text_rot_z1_3 = TextBox(box_rot_z1_3, 'rot_z1', initial=f"{rot_z1_rear:.1f}")
    box_rot_x_3 = plt.axes([0.37, 0.3, 0.05, 0.03], facecolor='linen')
    text_rot_x_3 = TextBox(box_rot_x_3, 'rot_x', initial=f"{rot_x_rear:.1f}")
    box_rot_z2_3 = plt.axes([0.46, 0.3, 0.05, 0.03], facecolor='linen')
    text_rot_z2_3 = TextBox(box_rot_z2_3, 'rot_z2', initial=f"{rot_z2_rear:.1f}")

    menu_topview_opt.on_clicked(update_calib)
    button_export_calib.on_clicked(export_calib)

    text_pos_x_0.on_submit(update_calib)
    text_pos_y_0.on_submit(update_calib)
    text_rot_z1_0.on_submit(update_calib)
    text_rot_x_0.on_submit(update_calib)
    text_rot_z2_0.on_submit(update_calib)

    text_pos_x_1.on_submit(update_calib)
    text_pos_y_1.on_submit(update_calib)
    text_rot_z1_1.on_submit(update_calib)
    text_rot_x_1.on_submit(update_calib)
    text_rot_z2_1.on_submit(update_calib)

    text_pos_x_2.on_submit(update_calib)
    text_pos_y_2.on_submit(update_calib)
    text_rot_z1_2.on_submit(update_calib)
    text_rot_x_2.on_submit(update_calib)
    text_rot_z2_2.on_submit(update_calib)

    text_pos_x_3.on_submit(update_calib)
    text_pos_y_3.on_submit(update_calib)
    text_rot_z1_3.on_submit(update_calib)
    text_rot_x_3.on_submit(update_calib)
    text_rot_z2_3.on_submit(update_calib)

    plt.show()
