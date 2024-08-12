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