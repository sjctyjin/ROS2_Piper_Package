import argparse
import glob
import os
import sys

import cv2
import numpy as np

RS_IMPORT_ERROR = None
try:
    import pyrealsense2 as rs
except ImportError as exc:
    rs = None
    RS_IMPORT_ERROR = exc


DEFAULTS = {
    "h_low1": 0,
    "h_high1": 24,
    "s_low": 170,
    "v_low": 70,
    "min_area": 200,
    "max_area": 200000,
    "expand": 6,
}


def noop(_value):
    return


def add_label(image, text):
    cv2.rectangle(image, (10, 10), (300, 42), (0, 0, 0), thickness=-1)
    cv2.putText(image, text, (18, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    return image


def create_windows():
    cv2.namedWindow("HSV Controls", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Result", cv2.WINDOW_NORMAL)

    cv2.createTrackbar("H low", "HSV Controls", DEFAULTS["h_low1"], 179, noop)
    cv2.createTrackbar("H high", "HSV Controls", DEFAULTS["h_high1"], 179, noop)
    cv2.createTrackbar("S low", "HSV Controls", DEFAULTS["s_low"], 255, noop)
    cv2.createTrackbar("V low", "HSV Controls", DEFAULTS["v_low"], 255, noop)
    cv2.createTrackbar("Min area", "HSV Controls", DEFAULTS["min_area"], 500000, noop)
    cv2.createTrackbar("Max area", "HSV Controls", DEFAULTS["max_area"], 500000, noop)
    cv2.createTrackbar("Expand", "HSV Controls", DEFAULTS["expand"], 50, noop)


def read_params():
    h_low = cv2.getTrackbarPos("H low", "HSV Controls")
    h_high = cv2.getTrackbarPos("H high", "HSV Controls")
    s_low = cv2.getTrackbarPos("S low", "HSV Controls")
    v_low = cv2.getTrackbarPos("V low", "HSV Controls")
    min_area = cv2.getTrackbarPos("Min area", "HSV Controls")
    max_area = cv2.getTrackbarPos("Max area", "HSV Controls")
    expand = cv2.getTrackbarPos("Expand", "HSV Controls")

    if h_high < h_low:
        h_high = h_low
        cv2.setTrackbarPos("H high", "HSV Controls", h_high)
    if max_area < min_area:
        max_area = min_area
        cv2.setTrackbarPos("Max area", "HSV Controls", max_area)

    return {
        "h_low1": h_low,
        "h_high1": h_high,
        "s_low": s_low,
        "v_low": v_low,
        "min_area": min_area,
        "max_area": max_area,
        "expand": expand,
    }


def build_mask(frame_bgr, params):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([params["h_low1"], params["s_low"], params["v_low"]], dtype=np.uint8)
    upper = np.array([params["h_high1"], 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)

    clean_kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, clean_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, clean_kernel)

    if params["expand"] > 0:
        kernel_size = params["expand"] * 2 + 1
        expand_kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.dilate(mask, expand_kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_mask = np.zeros_like(mask)
    filtered_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if params["min_area"] <= area <= params["max_area"]:
            cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)
            filtered_contours.append(contour)

    return filtered_mask, filtered_contours


def print_params(params):
    print("\n# HSV parameters")
    print(f"self.h_low1 = {params['h_low1']}")
    print(f"self.h_high1 = {params['h_high1']}")
    print(f"self.s_low = {params['s_low']}")
    print(f"self.v_low = {params['v_low']}")
    print(f"self.min_area = {params['min_area']}")
    print(f"self.max_area = {params['max_area']}")
    print(f"self.expand = {params['expand']}\n")


def reset_params():
    cv2.setTrackbarPos("H low", "HSV Controls", DEFAULTS["h_low1"])
    cv2.setTrackbarPos("H high", "HSV Controls", DEFAULTS["h_high1"])
    cv2.setTrackbarPos("S low", "HSV Controls", DEFAULTS["s_low"])
    cv2.setTrackbarPos("V low", "HSV Controls", DEFAULTS["v_low"])
    cv2.setTrackbarPos("Min area", "HSV Controls", DEFAULTS["min_area"])
    cv2.setTrackbarPos("Max area", "HSV Controls", DEFAULTS["max_area"])
    cv2.setTrackbarPos("Expand", "HSV Controls", DEFAULTS["expand"])


def parse_args():
    parser = argparse.ArgumentParser(description="Realtime HSV tuner for Intel RealSense color stream")
    parser.add_argument("--serial", default="", help="Optional RealSense serial number")
    parser.add_argument("--width", type=int, default=640, help="Color stream width")
    parser.add_argument("--height", type=int, default=480, help="Color stream height")
    parser.add_argument("--fps", type=int, default=30, help="Color stream fps")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index for fallback mode")
    parser.add_argument("--device", default="", help="Explicit V4L2 device path, e.g. /dev/video4")
    return parser.parse_args()


class RealSenseColorSource:
    def __init__(self, args):
        if rs is None:
            raise RuntimeError(f"pyrealsense2 import failed: {RS_IMPORT_ERROR}")
        if not hasattr(rs, "pipeline"):
            raise RuntimeError("pyrealsense2 is installed but does not expose pipeline")

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if args.serial:
            self.config.enable_device(args.serial)
        self.config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
        self.pipeline.start(self.config)
        self.mode_name = "RealSense"

    def read(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return False, None
        return True, np.asanyarray(color_frame.get_data())

    def release(self):
        self.pipeline.stop()


class OpenCvColorSource:
    def __init__(self, args):
        self.cap = None
        self.mode_name = ""
        errors = []

        candidates = []
        if args.device:
            candidates.append(args.device)
        else:
            linux_devices = sorted(glob.glob("/dev/video*"))
            candidates.extend(linux_devices)
            candidates.extend([args.camera_index, 0, 1, 2, 3, 4, 5])

        deduped_candidates = []
        for candidate in candidates:
            if candidate not in deduped_candidates:
                deduped_candidates.append(candidate)

        for candidate in deduped_candidates:
            cap = self._open_candidate(candidate, args)
            if cap is None:
                errors.append(str(candidate))
                continue

            ok, frame = self._warmup_read(cap)
            if ok and frame is not None:
                self.cap = cap
                self.mode_name = f"OpenCV source {candidate}"
                break

            cap.release()
            errors.append(str(candidate))

        if self.cap is None:
            raise RuntimeError(
                "Cannot open any camera source. Tried: "
                + ", ".join(errors)
                + ". Try --device /dev/video4 or another /dev/videoX from ls /dev/video*"
            )

    @staticmethod
    def _open_candidate(candidate, args):
        if isinstance(candidate, str) and candidate.startswith("/dev/"):
            cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)

        if not cap.isOpened():
            cap.release()
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        return cap

    @staticmethod
    def _warmup_read(cap, retries=10):
        for _ in range(retries):
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 0:
                return True, frame
        return False, None

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


def create_color_source(args):
    try:
        return RealSenseColorSource(args)
    except Exception as exc:
        print(f"RealSense mode unavailable, fallback to OpenCV VideoCapture: {exc}")
        return OpenCvColorSource(args)


def main():
    args = parse_args()
    source = create_color_source(args)

    create_windows()
    print(f"Source: {source.mode_name}")
    print("Controls: q=quit, s=print params, r=reset defaults")

    try:
        while True:
            ok, frame_bgr = source.read()
            if not ok or frame_bgr is None:
                continue
            params = read_params()
            filtered_mask, contours = build_mask(frame_bgr, params)

            original = frame_bgr.copy()
            if contours:
                cv2.drawContours(original, contours, -1, (0, 255, 0), 2)

            result = cv2.bitwise_and(frame_bgr, frame_bgr, mask=filtered_mask)
            mask_preview = cv2.cvtColor(filtered_mask, cv2.COLOR_GRAY2BGR)

            add_label(original, "Original")
            add_label(mask_preview, "Mask")
            add_label(result, "Result")

            cv2.imshow("Original", original)
            cv2.imshow("Mask", mask_preview)
            cv2.imshow("Result", result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                print_params(params)
            if key == ord("r"):
                reset_params()

    except KeyboardInterrupt:
        pass
    finally:
        source.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
