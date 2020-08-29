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
