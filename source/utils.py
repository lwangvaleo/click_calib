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

from scipy.spatial.transform import Rotation as SciRot
import json
from projection import Camera, RadialPolyCamProjection

def quat_to_mat(quat):
    return SciRot.from_quat(quat).as_matrix()


def init_fisheye_cam(intr, quat, t):
    coef = [intr['k1'], intr['k2'], intr['k3'], intr['k4']]
    cam = Camera(
        rotation=quat_to_mat(quat),
        translation=t,
        lens=RadialPolyCamProjection(coef),
        size=(intr['width'], intr['height']),
        principle_point=(intr['cx_offset'], intr['cy_offset']),
        aspect_ratio=intr['aspect_ratio']
    )
    return cam


def read_calib(path):
    with open(path) as f:
        calib = json.load(f)
    intr = calib['intrinsic']
    quat = calib['extrinsic']['quaternion']
    t = calib['extrinsic']['translation']
    return intr, quat, t


def write_calib(intr, quat, t, save_path):
    calib = {}
    calib["extrinsic"] = {"quaternion": quat, "translation": t}
    calib["intrinsic"] = intr
    with open(save_path, "w") as f:
        json.dump(calib, f, indent=4)