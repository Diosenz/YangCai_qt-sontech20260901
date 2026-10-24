"""Use installed OpenCV C image primitives; no installation or compilation."""
import ctypes as c
import os
import numpy as np

DLL_DIR = "E:/Libs/OPENCV3/opencv/build/x64/vc15/bin"
_dll_dir = os.add_dll_directory(DLL_DIR)
lib = c.CDLL(DLL_DIR + "/opencv_world345.dll")


class Mat(c.Structure):
    _fields_ = [("type", c.c_int), ("step", c.c_int), ("refcount", c.c_void_p), ("hdr_refcount", c.c_int), ("data", c.c_void_p), ("rows", c.c_int), ("cols", c.c_int)]


class Point(c.Structure):
    _fields_ = [("x", c.c_int), ("y", c.c_int)]


class Scalar(c.Structure):
    _fields_ = [("val", c.c_double * 4)]


lib.cvInitMatHeader.argtypes = [c.POINTER(Mat), c.c_int, c.c_int, c.c_int, c.c_void_p, c.c_int]
lib.cvInitMatHeader.restype = c.POINTER(Mat)
lib.cvResize.argtypes = [c.c_void_p, c.c_void_p, c.c_int]
lib.cvLine.argtypes = [c.c_void_p, Point, Point, Scalar, c.c_int, c.c_int, c.c_int]


def header(im):
    assert im.flags.c_contiguous
    m = Mat()
    code = 0 if im.dtype == np.uint8 else 5
    lib.cvInitMatHeader(c.byref(m), im.shape[0], im.shape[1], code, im.ctypes.data, im.strides[0])
    return m


def resize(im, width, height):
    im = np.ascontiguousarray(im, dtype=np.uint8)
    out = np.empty((height, width), dtype=np.uint8)
    a, b = header(im), header(out)
    lib.cvResize(c.byref(a), c.byref(b), 1)
    return out.astype(np.float32)


def lines(points, shape=(964, 1292), thickness=10):
    im = np.zeros(shape, dtype=np.uint8)
    m = header(im)
    pts = np.floor(np.asarray(points) + .5).astype(int)
    white = Scalar((255, 0, 0, 0))
    for a, b in zip(pts[:-1], pts[1:]):
        lib.cvLine(c.byref(m), Point(*a), Point(*b), white, thickness, 16, 0)
    return im
