import colorsys

import numpy as np


def random_bright_color(uint=False):
    base = np.random.rand(3)
    h, s, l = base[0], 0.5 + base[1] / 2.0, 0.4 + base[2] / 5.0
    r, g, b = [int(256 * i) for i in colorsys.hls_to_rgb(h, l, s)]

    if uint:
        return r, g, b
    else:
        return [x / 255 for x in (r, g, b)]


def pybulletimage2numpy(img):
    width, height = img[0:2]
    rgb = img[2]
    rgb = np.reshape(rgb, (height, width, 4))
    rgb = rgb[:, :, :3]
    return rgb


def pybulletsegmap2numpy(img):
    width, height = img[0:2]
    segmap = img[4]
    segmap = np.reshape(segmap, (height, width))
    segmap = segmap[:, :]
    return segmap


def segmap2color(segmap):
    output = np.zeros((segmap.shape[0], segmap.shape[1], 3), np.uint8)

    for inst in range(segmap.max() + 1):
        output[segmap == inst, :] = np.random.uniform(200, 255, 3)

    return output


def segmap2color_fixed(segmap):
    output = np.zeros((segmap.shape[0], segmap.shape[1], 3), np.uint8)
    output[segmap == 0, :] = (255, 0, 0)
    output[segmap == 1, :] = (0, 255, 0)

    return output
