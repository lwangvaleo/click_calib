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
import matplotlib.image as mpimg

def zoom(event):
    ax = event.inaxes
    if ax is None:
        return
    xdata, ydata = event.xdata, event.ydata
    x, y = ax.get_xlim(), ax.get_ylim()
    if event.button == 'up':      # Zoom in on scroll up
        ax.set_xlim(xdata - (xdata - x[0]) / 1.1, xdata + (x[1] - xdata) / 1.1)
        ax.set_ylim(ydata - (ydata - y[0]) / 1.1, ydata + (y[1] - ydata) / 1.1)
    elif event.button == 'down':  # Zoom out on scroll down
        ax.set_xlim(xdata - (xdata - x[0]) * 1.1, xdata + (x[1] - xdata) * 1.1)
        ax.set_ylim(ydata - (ydata - y[0]) * 1.1, ydata + (y[1] - ydata) * 1.1)
    ax.figure.canvas.draw()

def onclick(event):
    if event.inaxes == ax1:
        x, y = event.xdata, event.ydata
        pts_1.append((int(x), int(y)))
        pt_1_idx = len(pts_1)
        ax1.plot(x, y, 'ro', markersize=3)
        ax1.annotate(f"{pt_1_idx}", (x, y), color=(0.70, 1, 0.40), fontsize=6)
    elif event.inaxes == ax2:
        x, y = event.xdata, event.ydata
        pts_2.append((int(x), int(y)))
        pt_2_idx = len(pts_2)
        ax2.plot(x, y, 'ro', markersize=3)
        ax2.annotate(f"{pt_2_idx}", (x, y), color=(0.70, 1, 0.40), fontsize=6)
    fig.canvas.draw()

if __name__ == '__main__':
    pts_1 = []
    pts_2 = []
    img_1_path = "../images/fisheye/00164_FV.png"
    img_2_path = "../images/fisheye/00165_MVL.png"
    img1 = mpimg.imread(img_1_path)
    img2 = mpimg.imread(img_2_path)

    fig, (ax1, ax2) = plt.subplots(1, 2)
    ax1.imshow(img1)
    ax1.set_title('Cam_1', fontsize=10)
    ax1.axis('off')
    ax2.imshow(img2)
    ax2.set_title('Cam_2', fontsize=10)
    ax2.axis('off')

    fig.text(0.5, 0.94, 'Click-Calib', fontsize=12, fontweight='bold', ha='center')
    fig.suptitle('Click to select keypoints in both cameras. Scroll to zoom.\n'
                 'Keypoints with the same index in both images should match in world.\n'
                 'The number of selected keypoints must be equal in each camera.', fontsize=10, y=0.9, linespacing=2)

    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('scroll_event', zoom)
    plt.show()

    assert len(pts_1) == len(pts_2), "The number of points in two cameras must be the same!"
    print(f"Points in cam 1: {pts_1}")
    print(f"Points in cam 2: {pts_2}")


