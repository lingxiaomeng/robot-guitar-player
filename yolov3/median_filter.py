import numpy as np
from numba import njit, prange
import cv2


@njit
def padding(img, pad):
    padded_img = np.zeros((img.shape[0] + 2 * pad, img.shape[1] + 2 * pad))
    padded_img[pad:-pad, pad:-pad] = img
    return padded_img


@njit(parallel=True)
def AdaptiveMedianFilter(img, s=51, sMax=101):
    if len(img.shape) == 3:
        raise Exception("Single channel image only")

    H, W = img.shape
    a = sMax // 2
    padded_img = padding(img, a)

    f_img = np.zeros(padded_img.shape)

    for i in prange(a, H + a + 1):
        for j in range(a, W + a + 1):
            value = Lvl_A(padded_img, i, j, s, sMax)
            f_img[i, j] = value

    return f_img[a:-a, a:-a]


@njit
def Lvl_A(mat, x, y, s, sMax):
    window = mat[x - (s // 2):x + (s // 2) + 1, y - (s // 2):y + (s // 2) + 1]
    Zmin = np.min(window)
    Zmed = np.median(window)
    Zmax = np.max(window)

    A1 = Zmed - Zmin
    A2 = Zmed - Zmax

    if A1 > 0 and A2 < 0:
        return Lvl_B(window, Zmin, Zmed, Zmax)
    else:
        s += 2
        if s <= sMax:
            return Lvl_A(mat, x, y, s, sMax)
        else:
            return Zmed


@njit
def Lvl_B(window, Zmin, Zmed, Zmax):
    h, w = window.shape

    Zxy = window[h // 2, w // 2]
    B1 = Zxy - Zmin
    B2 = Zxy - Zmax

    if B1 > 0 and B2 < 0:
        return Zxy
    else:
        return Zmed


if __name__ == "__main__":
    img = cv2.imread('./depth.jpg', cv2.IMREAD_GRAYSCALE)
    img = AdaptiveMedianFilter(img)
    cv2.imwrite('./depth_filtered.jpg', img)
