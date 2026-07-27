# PROJECT FILE HEADER
# 文件：apps/camera/epiceye/epicraw_parser.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from enum import Enum
import numpy as np
import struct
import json


class FileType(Enum):
    NotSupported = -1
    EpicRaw1 = 0
    EpicRaw2 = 1


class DataType(Enum):
    Non = -1
    RGB24 = 0
    Depth32 = 1
    RGB48 = 2


def _array_from_buffer(
        buffer: bytes, width: int, height: int, raw_type: DataType
):
    if raw_type == DataType.RGB24:
        result = np.frombuffer(buffer, dtype=np.uint8)
        return result.reshape(height, width, 3)
    if raw_type == DataType.RGB48:
        result = np.frombuffer(buffer, dtype=np.uint16)
        return result.reshape(height, width, 3)
    if raw_type == DataType.Depth32:
        result = np.frombuffer(buffer, dtype=np.float32)
        return result.reshape(height, width)
    return None


def get_width_height_from_bytes(data: bytes):
    _, width, height = struct.unpack("<8s2i", data[:16])
    return width, height


def decode_point_cloud_from_bytes(data: bytes, undistort_lut):
    file_type = _get_file_type(data)
    if file_type == FileType.EpicRaw1:
        depth_data = EpicRaw1.get_depth(data)
        matrix = EpicRaw1.get_camera_matrix(data)
        return _decode_point_cloud_from_depth(depth_data, matrix, undistort_lut)
    if file_type == FileType.EpicRaw2:
        depth_data = EpicRaw2.get_depth(data)
        matrix = EpicRaw2.get_camera_matrix(data)
        return _decode_point_cloud_from_depth(depth_data, matrix, undistort_lut)
    return None


def _decode_point_cloud_from_depth(depth: np.ndarray, matrix: np.ndarray, undistort_lut):
    if undistort_lut is None:
        undistort_lut = np.stack(
            np.meshgrid(range(0, depth.shape[1]), range(0, depth.shape[0])), axis=2
        )

    u = undistort_lut[:, :, 0]
    v = undistort_lut[:, :, 1]

    x = (u - matrix[0][2]) * depth / matrix[0][0]
    y = (v - matrix[1][2]) * depth / matrix[1][1]
    point_map = np.dstack((x, y, depth))

    # to avoid numpy rules error, do not use bitwise_or here
    # invalid = depth < 0.1 | depth > 6000 is not version and platform compatible
    invalid = depth < 0.1
    point_map[invalid] = (np.nan, np.nan, np.nan)
    invalid = depth > 6000
    point_map[invalid] = (np.nan, np.nan, np.nan)

    return point_map


def get_depth_from_bytes(data: bytes):
    file_type = _get_file_type(data)
    if file_type == FileType.EpicRaw1:
        return EpicRaw1.get_depth(data)
    if file_type == FileType.EpicRaw2:
        return EpicRaw2.get_depth(data)
    return None


def get_image_from_epicraw_bytes(data: bytes):
    file_type = _get_file_type(data)
    if file_type == FileType.EpicRaw1:
        return EpicRaw1.get_image(data)
    if file_type == FileType.EpicRaw2:
        return EpicRaw2.get_image(data)
    return None


def get_matrix_from_epicraw_bytes(data: bytes):
    file_type = _get_file_type(data)
    if file_type == FileType.EpicRaw1:
        return EpicRaw1.get_camera_matrix(data)
    if file_type == FileType.EpicRaw2:
        return EpicRaw2.get_camera_matrix(data)
    return None


def get_distortion_from_epicraw_bytes(data: bytes):
    file_type = _get_file_type(data)
    if file_type == FileType.EpicRaw1:
        return EpicRaw1.get_distortion(data)
    if file_type == FileType.EpicRaw2:
        return EpicRaw2.get_distortion(data)
    return None


def _get_file_type(data: bytes):
    filetype_bytes = data[:8]
    filetype_str = struct.unpack('<8s', filetype_bytes)[0]
    if filetype_str == b"EPICRAW1":
        return FileType.EpicRaw1
    if filetype_str == b"EPICRAW2":
        return FileType.EpicRaw2
    if filetype_str[7] == 0:
        return FileType.EpicRaw1
    return FileType.NotSupported


class EpicRaw1:
    parameters_keys = [
        'projector_brightness', 'exposure_2d', 'exposure_3d', 'use_hdr',
        'exposure_hdr', 'use_PF', 'use_SF'
    ]

    @staticmethod
    def get_image(data):
        header_data = data[:168]
        _, width, height, _, depth_data_len, image_data_len = struct.unpack(
            f"<8sii144s2i", header_data
        )
        if image_data_len == 0:
            return None
        image_data = data[168 + depth_data_len:]
        return _array_from_buffer(image_data, width, height, DataType.RGB24)

    @staticmethod
    def get_depth(data: bytes):
        header_data = data[:168]
        _, width, height, _, depth_data_len, image_data_len = struct.unpack(
            f"<8sii144s2i", header_data
        )
        if depth_data_len == 0:
            return None
        depth_data = data[168: 168 + depth_data_len]
        return _array_from_buffer(depth_data, width, height, DataType.Depth32)

    @staticmethod
    def get_camera_matrix(data: bytes):
        header_data = data[:168]
        _, matrix_data, _ = struct.unpack(
            f"<16s72s80s", header_data
        )
        return np.frombuffer(matrix_data, np.float64).reshape(3, 3)

    @staticmethod
    def get_distortion(data: bytes):
        header_data = data[:168]
        _, distort_data, _ = struct.unpack(
            f"<88s40s40s", header_data
        )
        return np.frombuffer(distort_data, np.float64)

    @staticmethod
    def get_config_dict(data: bytes):
        header_data = data[:168]
        _, config_data, _ = struct.unpack(
            f"<128s28s12s", header_data
        )
        return {
            k: v for k, v in zip(
                EpicRaw1.parameters_keys,
                struct.unpack('<I2fIf2I', config_data)
            )
        }


class EpicRaw2:
    @staticmethod
    def get_image(data: bytes):
        header_data = data[:40]
        _, width, height, data_type, camera_matrix_len, \
            distortion_len, config_str_len, depth_data_len, \
            image_data_len = struct.unpack(f"<8s8i", header_data)
        if image_data_len == 0:
            return None
        data_offset = 40 + camera_matrix_len + distortion_len + config_str_len + depth_data_len
        image_data = data[data_offset:]
        return _array_from_buffer(image_data, width, height, DataType.RGB48)

    @staticmethod
    def get_depth(data: bytes):
        header_data = data[:40]
        _, width, height, data_type, camera_matrix_len, \
            distortion_len, config_str_len, depth_data_len, \
            image_data_len = struct.unpack(f"<8s8i", header_data)
        if depth_data_len == 0:
            return None
        data_offset = 40 + camera_matrix_len + distortion_len + config_str_len
        depth_data = data[data_offset: data_offset + depth_data_len]
        return _array_from_buffer(depth_data, width, height, DataType.Depth32)

    @staticmethod
    def get_camera_matrix(data: bytes):
        header_data = data[:40]
        _, width, height, data_type, camera_matrix_len, \
            distortion_len, config_str_len, depth_data_len, \
            image_data_len = struct.unpack(f"<8s8i", header_data)
        matrix_data = data[40: 40 + camera_matrix_len]
        return np.frombuffer(matrix_data, np.float64).reshape(3, 3)

    @staticmethod
    def get_distortion(data: bytes):
        header_data = data[:40]
        _, width, height, data_type, camera_matrix_len, \
            distortion_len, config_str_len, depth_data_len, \
            image_data_len = struct.unpack(f"<8s8i", header_data)
        distortion_data = data[40 + camera_matrix_len: 40 + camera_matrix_len + distortion_len]
        return np.frombuffer(distortion_data, np.float64)

    @staticmethod
    def get_config_dict(data: bytes):
        header_data = data[:40]
        _, width, height, data_type, camera_matrix_len, \
            distortion_len, config_str_len, depth_data_len, \
            image_data_len = struct.unpack(f"<8s8i", header_data)
        data_offset = 40 + camera_matrix_len + distortion_len
        config_data = data[data_offset: data_offset + config_str_len].decode('ascii')
        return json.loads(config_data)
