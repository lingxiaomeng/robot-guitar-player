import pyrealsense2 as rs
import numpy as np
import cv2
import math
from detect import YoloDetector
from math import sqrt

np.set_printoptions(threshold=np.inf)


class ImageUtils:
    def __init__(self):
        self.H = self.get_H()
        self.H_inv = np.linalg.inv(self.H)
        self.detector = YoloDetector()

    def change_image(self, img):
        out = cv2.warpPerspective(img, self.H, (1200, 1000))
        return out

    def inv_image(self, img):
        out = cv2.warpPerspective(img, self.H_inv, (1280, 720))
        return out

    def get_image(self):
        # Configure depth and color streams

        pipeline = rs.pipeline()

        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start streaming
        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        align_to = rs.stream.color
        align = rs.align(align_to)

        try:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = frames.get_color_frame()

            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            if not depth_frame or not color_frame:
                return None

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            perspective_image = self.change_image(color_image)

            poss, label, im0 = self.detector.detect(perspective_image)

            im0 = self.inv_image(im0)
            # print(im0.shape)
            cv2.imwrite('./labeled_image.jpg', im0)
            if len(poss) > 0:
                x = (poss[0][0] + poss[0][2]) / 2
                y = (poss[0][1] + poss[0][3]) / 2
                print(f"perspective x: {x} y: {y}")
                x, y = self.perspective(x, y)
                print(f"x: {x} y: {y}")
                depth = depth_frame.get_distance(int(x), int(y))
                print(f"distance: {depth}")

                depth = self.get_depth(depth_image, x, y)
                depth = depth * depth_scale
                print(f"distance: {depth} depth_scale {depth_scale} depth_image {depth_image[int(y)][int(x)]}")
                depth_point = rs.rs2_deproject_pixel_to_point(
                    depth_intrin, [x, y], depth)

                print(f"point {depth_point}")
                h_depth_point = np.append(depth_point, 1)

                robot_point = self.transpose_pos(h_depth_point)
                print(robot_point)
                # images = np.hstack((color_image, depth_colormap))
                cv2.imwrite('./depth.jpg', depth_image * depth_scale * 255)
                return robot_point
                # Show images
                # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('RealSense', images)
                # cv2.waitKey(0)
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # return color_image, depth_image
            return None
        finally:
            pipeline.stop()
            # pass

    def get_H(self):
        K = np.array([[892.622005, 0.000000, 637.957594],
                      [0.000000, 891.284636, 357.766994],
                      [0.000000, 0.000000, 1.000000]])

        xa = np.array([1 / math.sqrt(2), 1 / math.sqrt(2), 0])
        za = np.array([math.sqrt(2) / 4, -math.sqrt(2) / 4, math.sqrt(3) / 2])
        # print np.cross(xa, za)

        R = np.array(
            [xa,
             np.cross(za, xa),
             za]).transpose()

        # R = np.linalg.inv(R + 1e-10)
        T = (np.array([[-0.3], [0.2], [0]]))

        d = 1
        dinv = 1 / d

        N = (np.array([[0], [0], [0.5]]))

        H = R + (T / d).dot(N.transpose())
        # print H
        K_top = np.array([[892.622005, 0.000000, 300],
                          [0.000000, 891.284636, 800],
                          [0.000000, 0.000000, 1.000000]])

        Homography = K_top.dot(H).dot(np.linalg.inv(K))
        return Homography

    def perspective(self, x, y):
        H = self.H_inv
        c = np.array([[x], [y], [1]])
        # print(H.dot(c))
        p = H.dot(c)
        px = p[0][0] / p[2][0]
        py = p[1][0] / p[2][0]
        return px, py

    def get_depth(self, depth_image, x, y):
        x = int(x)
        y = int(y)
        h, w = depth_image.shape
        depth = depth_image[y][x]
        if depth == 0:
            d1, d2, d3, d4 = 0, 0, 0, 0
            w1, w2, w3, w4 = 0, 0, 0, 0
            for i in range(0, x):
                d = depth_image[y][x - i]
                if d != 0:
                    d1 = d
                    w1 = i
                    break
            for i in range(0, w - x - 1):
                d = depth_image[y][x + i]
                if d != 0:
                    d2 = d
                    w2 = i
                    break
            for i in range(0, y):
                d = depth_image[y - i][x]
                if d != 0:
                    d3 = d
                    w3 = i
                    break
            for i in range(0, h - y - 1):
                d = depth_image[y + i][x]
                if d != 0:
                    d4 = d
                    w4 = i
                    break
            print(f"d:{d1} {d2} {d3} {d4}")
            print(f"w:{w1} {w2} {w3} {w4}")

            if w1 == w2 == w3 == w4 == 0:
                return 0
            else:
                w = w1 + w2 + w3 + w4
                d = d1 * (w - w1) + d2 * (w - w2) + d3 * (w - w3) + d4 * (w - w4)
                d = d / (3 * w)
                return d
        else:
            return depth

    def transpose_pos(self, pb):

        Tab = np.array([[sqrt(2) / 2, -sqrt(6) / 4, sqrt(2) / 4, 0],
                        [-sqrt(2) / 2, -sqrt(6) / 4, sqrt(2) / 4, 0],
                        [0, -1 / 2, -sqrt(3) / 2, 0.4717],
                        [0, 0, 0, 1]])

        Tac = np.array([[-sqrt(2) / 2, -sqrt(2) / 2, 0, 0.53992],
                        [sqrt(2) / 2, -sqrt(2) / 2, 0, -0.01248],
                        [0, 0, 1, 0.0337],
                        [0, 0, 0, 1]])

        Tca = np.linalg.inv(Tac)
        Tcb = Tca.dot(Tab)

        return Tcb.dot(pb.transpose())
