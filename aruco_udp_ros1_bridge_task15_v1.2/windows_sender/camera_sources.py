# -*- coding: utf-8 -*-
"""Camera backends for the Windows ArUco sender.

Default backend is the MVS industrial-camera SDK from the original project.
An OpenCV backend is included only for bench testing with a USB camera/video.
"""
from __future__ import annotations

import ctypes
import logging
import sys
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

LOGGER = logging.getLogger("camera")


class CameraError(RuntimeError):
    pass


class OpenCVCameraSource:
    def __init__(self, device_index: int = 0, video_file: str = "") -> None:
        source = video_file if video_file else int(device_index)
        self._capture = cv2.VideoCapture(source)
        if not self._capture.isOpened():
            raise CameraError(f"cannot open OpenCV camera source: {source}")

    def read(self, timeout_ms: int = 1000) -> Tuple[Optional[np.ndarray], Optional[int]]:
        del timeout_ms
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None, None
        return frame, None

    def close(self) -> None:
        self._capture.release()


class MvsCameraSource:
    def __init__(self, mvimport_dir: str, device_index: int = 0) -> None:
        mv_path = str(Path(mvimport_dir).resolve())
        if mv_path not in sys.path:
            sys.path.insert(0, mv_path)

        try:
            from MvCameraControl_class import MvCamera  # type: ignore
            from CameraParams_const import (  # type: ignore
                MV_ACCESS_Exclusive,
                MV_GIGE_DEVICE,
                MV_USB_DEVICE,
                MV_UNKNOW_DEVICE,
                MV_1394_DEVICE,
                MV_CAMERALINK_DEVICE,
            )
            from CameraParams_header import (  # type: ignore
                MV_CC_DEVICE_INFO,
                MV_CC_DEVICE_INFO_LIST,
                MV_FRAME_OUT,
                MV_GrabStrategy_LatestImagesOnly,
                MV_TRIGGER_MODE_OFF,
            )
            from PixelType_header import (  # type: ignore
                PixelType_Gvsp_Mono8,
                PixelType_Gvsp_BayerGB8,
                PixelType_Gvsp_RGB8_Packed,
                PixelType_Gvsp_YUV422_Packed,
                PixelType_Gvsp_YUV422_YUYV_Packed,
            )
        except Exception as exc:
            raise CameraError(
                "failed to import MVS SDK Python bindings; verify MVS is installed "
                "and MvImport is complete"
            ) from exc

        self.MvCamera = MvCamera
        self.MV_ACCESS_Exclusive = MV_ACCESS_Exclusive
        self.MV_GIGE_DEVICE = MV_GIGE_DEVICE
        self.MV_TRIGGER_MODE_OFF = MV_TRIGGER_MODE_OFF
        self.MV_GrabStrategy_LatestImagesOnly = MV_GrabStrategy_LatestImagesOnly
        self.MV_FRAME_OUT = MV_FRAME_OUT
        self.pixel_types = {
            "mono8": PixelType_Gvsp_Mono8,
            "bayer_gb8": PixelType_Gvsp_BayerGB8,
            "rgb8": PixelType_Gvsp_RGB8_Packed,
            "yuv422": PixelType_Gvsp_YUV422_Packed,
            "yuyv": PixelType_Gvsp_YUV422_YUYV_Packed,
        }

        tlayer_type = (
            MV_GIGE_DEVICE
            | MV_USB_DEVICE
            | MV_UNKNOW_DEVICE
            | MV_1394_DEVICE
            | MV_CAMERALINK_DEVICE
        )
        device_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)
        if ret != 0:
            raise CameraError(f"MVS enumerate devices failed: 0x{ret:08x}")
        if device_list.nDeviceNum == 0:
            raise CameraError("MVS found no camera")
        if device_index < 0 or device_index >= device_list.nDeviceNum:
            raise CameraError(
                f"camera index {device_index} is invalid; found {device_list.nDeviceNum} device(s)"
            )

        self._cam = MvCamera()
        device_info = ctypes.cast(
            device_list.pDeviceInfo[device_index], ctypes.POINTER(MV_CC_DEVICE_INFO)
        ).contents
        self._is_gige = device_info.nTLayerType == MV_GIGE_DEVICE

        ret = self._cam.MV_CC_CreateHandleWithoutLog(device_info)
        if ret != 0:
            raise CameraError(f"MVS create handle failed: 0x{ret:08x}")
        ret = self._cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            self._cam.MV_CC_DestroyHandle()
            raise CameraError(f"MVS open device failed: 0x{ret:08x}")

        if self._is_gige:
            packet_size = self._cam.MV_CC_GetOptimalPacketSize()
            if packet_size > 0:
                ret = self._cam.MV_CC_SetIntValueEx("GevSCPSPacketSize", packet_size)
                if ret != 0:
                    LOGGER.warning("failed to set GigE packet size: 0x%08x", ret)

        # Free-running acquisition and latest-frame strategy minimize stale-frame latency.
        ret = self._cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            LOGGER.warning("failed to set TriggerMode=Off: 0x%08x", ret)
        ret = self._cam.MV_CC_SetGrabStrategy(MV_GrabStrategy_LatestImagesOnly)
        if ret != 0:
            LOGGER.warning("failed to set latest-frame strategy: 0x%08x", ret)
        ret = self._cam.MV_CC_StartGrabbing()
        if ret != 0:
            self.close()
            raise CameraError(f"MVS start grabbing failed: 0x{ret:08x}")
        self._started = True

        LOGGER.info(
            "opened MVS camera index=%d, count=%d, transport=%s",
            device_index,
            device_list.nDeviceNum,
            "GigE" if self._is_gige else "non-GigE",
        )

    def read(self, timeout_ms: int = 1000) -> Tuple[Optional[np.ndarray], Optional[int]]:
        frame_out = self.MV_FRAME_OUT()
        ctypes.memset(ctypes.byref(frame_out), 0, ctypes.sizeof(frame_out))
        ret = self._cam.MV_CC_GetImageBuffer(frame_out, int(timeout_ms))
        if ret != 0 or not frame_out.pBufAddr:
            return None, None

        info = frame_out.stFrameInfo
        width = int(info.nWidth)
        height = int(info.nHeight)
        pixel_type = int(info.enPixelType)
        frame_number = int(info.nFrameNum)
        try:
            if pixel_type in (self.pixel_types["mono8"], self.pixel_types["bayer_gb8"]):
                size = width * height
            elif pixel_type == self.pixel_types["rgb8"]:
                size = width * height * 3
            elif pixel_type in (self.pixel_types["yuv422"], self.pixel_types["yuyv"]):
                size = width * height * 2
            else:
                raise CameraError(f"unsupported MVS pixel type: {pixel_type}")

            raw = ctypes.string_at(frame_out.pBufAddr, size)
            data = np.frombuffer(raw, dtype=np.uint8).copy()
            if pixel_type == self.pixel_types["mono8"]:
                image = data.reshape(height, width)
            elif pixel_type == self.pixel_types["bayer_gb8"]:
                image = cv2.cvtColor(data.reshape(height, width), cv2.COLOR_BAYER_GB2BGR)
            elif pixel_type == self.pixel_types["rgb8"]:
                image = cv2.cvtColor(data.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
            elif pixel_type == self.pixel_types["yuyv"]:
                image = cv2.cvtColor(data.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2)
            else:
                image = cv2.cvtColor(data.reshape(height, width, 2), cv2.COLOR_YUV2BGR_Y422)
            return image, frame_number
        finally:
            self._cam.MV_CC_FreeImageBuffer(frame_out)

    def close(self) -> None:
        cam = getattr(self, "_cam", None)
        if cam is None:
            return
        if getattr(self, "_started", False):
            cam.MV_CC_StopGrabbing()
            self._started = False
        cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()
        self._cam = None
