#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import numpy as np
import torch
import cv2
from PySide6 import QtCore

# ==========================================================
# Paths
# ==========================================================
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(CURRENT_DIR)
SRC_DIR = os.path.join(PARENT_DIR, "src")
ETC_DIR = os.path.join(PARENT_DIR, "etc")
GEN_DIR = os.path.join(PARENT_DIR, "generated")

for d in [SRC_DIR, GEN_DIR]:
    if d not in sys.path:
        sys.path.insert(0, d)

# ==========================================================
# RoboComp imports
# ==========================================================
from genericworker import GenericWorker
import interfaces
from model import MNISTNet


class SpecificWorker(GenericWorker):

    # ==================================================
    # INIT
    # ==================================================
    def __init__(self, proxy_map, configData, startup_check=False):
        super().__init__(proxy_map, configData)

        self.Period = configData["Period"]["Compute"]

        # ------------------------------
        # Load DNN
        # ------------------------------
        self.device = torch.device("cpu")
        self.model = MNISTNet().to(self.device)
        model_path = os.path.join(ETC_DIR, "my_network.pt")
        state = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(state)
        self.model.eval()

        # ------------------------------
        # Output (sensor state)
        # ------------------------------
        self.last_digit = -1
        self.last_confidence = 0.0
        self.last_update_time = 0.0
        self.CONF_THRESH = 0.2
        self.MAX_AGE = 0.8

        # ------------------------------
        # Debug
        # ------------------------------
        self.show_debug = True
        if self.show_debug:
            cv2.namedWindow("MNIST-debug", cv2.WINDOW_NORMAL)
            cv2.namedWindow("MNIST-ROI-28x28", cv2.WINDOW_NORMAL)

        # ------------------------------
        # Timer
        # ------------------------------
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    # ==================================================
    # DNN inference
    # ==================================================
    def classify_28x28(self, img):
        x = torch.tensor(img, dtype=torch.float32)
        x = x.unsqueeze(0).unsqueeze(0) / 255.0
        with torch.no_grad():
            out = self.model(x)
            probs = torch.softmax(out, dim=1)
            conf, digit = torch.max(probs, dim=1)
        return int(digit.item()), float(conf.item())

    # ==================================================
    # ICE interface
    # ==================================================
    def MNIST_getNumber(self):
        result = interfaces.MNISTResult()
        now = time.time()

        if (self.last_digit != -1 and
                (now - self.last_update_time) <= self.MAX_AGE):
            result.digit = self.last_digit
            result.confidence = self.last_confidence
        else:
            result.digit = -1
            result.confidence = 0.0

        return result

    # ==================================================
    def _show_debug(self, debug, roi=None):
        if not self.show_debug:
            return
        cv2.imshow("MNIST-debug", debug)
        if roi is not None:
            cv2.imshow("MNIST-ROI-28x28", roi)
        cv2.waitKey(1)

    # --------------------------------------------------
    # Periodic compute
    # --------------------------------------------------
    def compute(self):
        try:
            # ==================================================
            # 1) Leer imagen
            # ==================================================
            image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
            frame = np.frombuffer(image.image, dtype=np.uint8)
            frame = frame.reshape(image.height, image.width, 3)

            debug = frame.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape

            # ==================================================
            # PREFILTRO BARATO (negro)
            # ==================================================
            small = cv2.resize(gray, (320, 160), interpolation=cv2.INTER_AREA)
            bw_small = cv2.inRange(small, 0, 60)

            contours_small, _ = cv2.findContours(
                bw_small, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if not contours_small:
                self._show_debug(debug)
                return

            # ==================================================
            # PIPELINE REAL — MARCO NEGRO
            # ==================================================
            gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
            bw = cv2.inRange(gray_blur, 0, 60)

            kernel = np.ones((3, 3), np.uint8)
            bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(
                bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            best_frame = None
            best_score = -1

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 0.005 * h * w:
                    continue

                x, y, bwc, bhc = cv2.boundingRect(cnt)
                aspect = bwc / float(bhc)

                if 0.6 <= aspect <= 1.6 and area > best_score:
                    best_score = area
                    best_frame = (x, y, bwc, bhc)

            if best_frame is None:
                self._show_debug(debug)
                return

            # ==================================================
            # ROI DEL MARCO
            # ==================================================
            x, y, bwc, bhc = best_frame
            cv2.rectangle(debug, (x, y), (x + bwc, y + bhc), (0, 255, 0), 2)

            margin = int(0.1 * min(bwc, bhc))
            fx1 = max(0, x + margin)
            fy1 = max(0, y + margin)
            fx2 = min(w, x + bwc - margin)
            fy2 = min(h, y + bhc - margin)

            frame_roi = gray_blur[fy1:fy2, fx1:fx2]
            if frame_roi.size == 0:
                self._show_debug(debug)
                return

            if np.std(frame_roi) < 15:
                self._show_debug(debug)
                return

            # ==================================================
            # BUSCAR DÍGITO
            # ==================================================
            frame_roi = cv2.normalize(
                frame_roi, None, 0, 255, cv2.NORM_MINMAX
            )

            digit_bw = cv2.adaptiveThreshold(
                frame_roi, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                11, 2
            )

            digit_contours, _ = cv2.findContours(
                digit_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            digit_roi = None
            best_digit_area = 0
            rh, rw = frame_roi.shape

            for cnt in digit_contours:
                area = cv2.contourArea(cnt)
                if area < 0.01 * rh * rw:
                    continue

                dx, dy, dw, dh = cv2.boundingRect(cnt)
                if dx < 3 or dy < 3 or dx + dw > rw - 3 or dy + dh > rh - 3:
                    continue

                if area > best_digit_area:
                    best_digit_area = area
                    digit_roi = frame_roi[dy:dy + dh, dx:dx + dw]

            if digit_roi is None:
                self._show_debug(debug)
                return

            digit_roi = cv2.resize(digit_roi, (28, 28), interpolation=cv2.INTER_AREA)

            # ==================================================
            # DNN
            # ==================================================
            digit, confidence = self.classify_28x28(digit_roi)

            if confidence >= self.CONF_THRESH:
                self.last_digit = digit
                self.last_confidence = confidence
                self.last_update_time = time.time()

            cv2.putText(
                debug,
                f"{digit} ({confidence:.2f})",
                (x, max(30, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2
            )

            self._show_debug(debug, digit_roi)

        except Exception as e:
            print(f"[ERROR] Exception in compute(): {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()

    # ==================================================
    def startup_check(self):
        QtCore.QTimer.singleShot(
            200,
            QtCore.QCoreApplication.instance().quit
        )

