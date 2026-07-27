# PROJECT FILE HEADER
# 文件：apps/camera/epiceye/epiceye.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from .epicraw_parser import *
import os
import requests
import logging
import hashlib
import socket
import struct
from typing import List, Dict


EMPTY_ARRAY = None
API_URLS = {
    'get_info': "http://%s/api/EpicEye/Info",
    'get_config': "http://%s/api/CameraConfig?frameId=%s",
    'set_config': "http://%s/api/CameraConfig",
    'get_camera_matrix': "http://%s/api/CameraParameters/Intrinsic/CameraMatrix",
    'get_distortion': "http://%s/api/CameraParameters/Intrinsic/Distortion",
    'get_undistort_lut': "http://%s/api/UndistortLut",
    'trigger_frame': "http://%s/api/Frame?pointCloud=%s",
    'get_frame': "http://%s/api/Frame?frameId=%s",
    'get_depth': "http://%s/api/Depth?frameId=%s"
}

undistort_lut_dict = dict()
_http_session = requests.Session()
# A camera on the local network must not be routed through a desktop HTTP
# proxy.  Proxy environment variables are common in development shells and
# otherwise turn a healthy camera connection into a proxy error.
_http_session.trust_env = False


def _api_address(ip: str) -> str:
    """Return the EpicEye API host, adding the server port when omitted.

    Epic Hub is served on port 80, while current EpicEye Server releases
    expose the camera API on port 5000.  Callers historically pass only the
    camera IPv4 address, so keep that interface and add the API port here.
    Set EPICEYE_API_PORT=80 for legacy firmware, or pass an explicit
    ``host:port`` value to bypass the default.
    """
    address = str(ip).strip().rstrip('/')
    if address.startswith('http://'):
        address = address[len('http://'):]
    elif address.startswith('https://'):
        address = address[len('https://'):]

    # IPv4/hostname with an explicit port.  EpicEye devices in this project
    # use IPv4; bracketed IPv6 is preserved as an explicit address as well.
    if address.startswith('[') or ':' in address:
        return address

    port = os.environ.get('EPICEYE_API_PORT', '5000').strip()
    return f'{address}:{port}' if port else address


def _request_timeout():
    connect_timeout = float(os.environ.get("EPICEYE_CONNECT_TIMEOUT", "3"))
    read_timeout = float(os.environ.get("EPICEYE_READ_TIMEOUT", "30"))
    return connect_timeout, read_timeout


def init_logger_handler(name):
    log = logging.getLogger(name)
    log.propagate = False
    log.setLevel(logging.INFO)
    handler = logging.StreamHandler()
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    handler.setLevel(logging.INFO)
    log.addHandler(handler)
    return log

logger = init_logger_handler(f"EPICEYE")


def exception_handler(func):
    def handler(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as ex:
            logger.error(f"{func.__name__}: {repr(ex)}")
            return None

    return handler


def _request_post(url: str, content=None):
    try:
        resp = _http_session.post(url, data=content, timeout=_request_timeout())
        if resp.status_code != 200:
            response_preview = resp.text[:500].replace('\n', ' ').replace('\r', ' ')
            logger.warning(
                f'request from {url} get status={resp.status_code}, '
                f'body={response_preview!r}'
            )
            return None
        return resp
    except Exception as ex:
        logger.error(f'request from {url} error={repr(ex)}')
        return None


def _request_put(url: str, content=None):
    try:
        resp = _http_session.put(url, json=content, headers={
            'accept': '*/*', "Content-Type": "application/json"}, timeout=_request_timeout())
        if resp.status_code != 200:
            logger.warning(f'request from {url} get status={resp.status_code}')
            return None
        return resp
    except Exception as ex:
        logger.error(f'request from {url} error={repr(ex)}')
        return None


def _request_get(url: str):
    try:
        resp = _http_session.get(
            url,
            headers={'accept': '*/*'},
            timeout=_request_timeout(),
        )
        if resp.status_code != 200:
            logger.warning(f'request from {url} get status={resp.status_code}')
            return None
        return resp
    except Exception as ex:
        logger.error(f'request from {url} error={repr(ex)}')
        return None


@exception_handler
def get_info(ip: str):
    """ 获取相机信息, 获取失败会返回None

    parameters:
        ip: -- 相机的ip地址
    returns: 
        info: 类型为dict，其key-value为:
            sn: 字符串，相机的序列号
            ip: 字符串，相机的ip地址
            model: 字符串，相机的型号
            alias: 字符串，相机的名字，可在Web UI中自定义
            width: int类型，图像的width
            height: int类型，图像的height
    """
    response = _request_get(API_URLS['get_info'] % _api_address(ip))
    if response is not None:
        info = json.loads(response.content)
        return info
    return None


@exception_handler
def get_config(ip: str, frame_id: str = ""):
    """ 根据frameID获取相机参数配置，失败会返回None

    parameters:
        ip: -- 相机的ip地址
        frame_id: -- 如果frameID为空字符串""，则返回当前最新的相机配置参数
    returns:
        config: 类型为dict
    """
    response = _request_get(
        API_URLS['get_config'] % (_api_address(ip), frame_id)
    )
    if response is not None:
        return json.loads(response.content)
    return None


@exception_handler
def set_config(ip: str, config: dict):
    """更新相机参数配置，成功返回当前配置，失败返回None

    parameters:
        ip: 相机ip
        config: 类型为dict，定义同get_config，请参考get_config处文档
    returns:
        latest_config: 相机的当前配置，定义同config
    """
    response = _request_put(
        API_URLS['set_config'] % _api_address(ip),
        content=config,
    )
    if response is not None:
        return json.loads(response.content)
    return None


@exception_handler
def trigger_frame(ip: str, pointcloud: bool = True):
    """触发拍摄一个Frame，一个Frame可能同时包含
    2D图像和点云数据，通过frameID进行索引，调用此方法后，会返回frameID，
    随后可以通过getImage和getPointCloud方法将数据取回，失败则返回None

    parameters:
        ip: 相机ip
        pointcloud: bool类型，表示是否请求点云数据，如果设为False，则此次触发的Frame仅包含2D图像数据
    returns:
        frame_id: 字符串，此次触发拍照返回的frame_id
    """
    # Epic Eye firmware expects JSON-style lowercase boolean query values.
    # Formatting a Python bool directly produces "True"/"False", which some
    # firmware versions reject with HTTP 404 instead of a validation error.
    pointcloud_query = "true" if pointcloud else "false"
    response = _request_post(
        API_URLS['trigger_frame'] % (_api_address(ip), pointcloud_query)
    )
    if response is not None:
        frame_id = response.content.decode('utf-8')
        return frame_id
    return None


@exception_handler
def get_image(ip: str, frame_id: str):
    """根据frameID获取2D图像，失败则返回None

    parameters:
        ip: 相机ip
        frame_id: 待获取数据的frameID，可由triggerFrame获得
    returns:
        image: numpy array，返回的图像数据
    """
    response = _request_get(
        API_URLS['get_frame'] % (_api_address(ip), frame_id)
    )
    if response is not None:
        image = get_image_from_epicraw_bytes(response.content)
        return image
    return None


@exception_handler
def get_point_cloud(ip: str, frame_id: str):
    """根据frameID获取点云，失败则返回None

    parameters:
        ip: 相机ip
        frame_id: 待获取数据的frameID，可由triggerFrame获得
    returns:
        point_map: numpy array，点云数据，和2D图像逐像素对齐
    """
    api_address = _api_address(ip)
    response = _request_get(API_URLS['get_frame'] % (api_address, frame_id))
    if response is not None:
        width, height = get_width_height_from_bytes(response.content)
        if ip not in undistort_lut_dict:
            undistort_lut = get_undistort_lut(api_address, width, height)
            if undistort_lut is None:
                undistort_lut = np.stack(np.meshgrid(range(0, width), range(0, height)), axis=2)
            undistort_lut_dict[ip] = undistort_lut
        else:
            undistort_lut = undistort_lut_dict[ip]

        return decode_point_cloud_from_bytes(response.content, undistort_lut)
    return None


@exception_handler
def get_image_and_point_cloud(ip: str, frame_id: str):
    """根据frameID获取2D图像和点云，失败则返回None

    parameters:
        ip: 相机ip
        frame_id: 待获取数据的frameID，可由triggerFrame获得
    returns:
        image: numpy array，返回的图像数据
        point_map: numpy array，点云数据，和2D图像逐像素对齐
    """
    api_address = _api_address(ip)
    response = _request_get(API_URLS['get_frame'] % (api_address, frame_id))
    if response is not None:
        image = get_image_from_epicraw_bytes(response.content)
        width, height = get_width_height_from_bytes(response.content)
        if ip not in undistort_lut_dict:
            undistort_lut = get_undistort_lut(api_address, width, height)
            if undistort_lut is None:
                undistort_lut = np.stack(np.meshgrid(range(0, width), range(0, height)), axis=2)
            undistort_lut_dict[ip] = undistort_lut
        else:
            undistort_lut = undistort_lut_dict[ip]
        point_cloud = decode_point_cloud_from_bytes(response.content, undistort_lut)
        return image, point_cloud
    return None, None


@exception_handler
def get_depth(ip: str, frame_id: str):
    """根据frameID获取点云，失败则返回None

    parameters:
        ip: 相机ip
        frame_id: 待获取数据的frameID，可由triggerFrame获得
    returns:
        depth: numpy array，深度数据，和2D图像逐像素对齐
    """
    response = _request_get(
        API_URLS['get_depth'] % (_api_address(ip), frame_id)
    )
    if response is not None:
        return get_depth_from_bytes(response.content)
    return None


@exception_handler
def get_camera_matrix(ip: str):
    """获取2D图像对应相机的相机矩阵，失败则返回None

    parameters:
        ip: 相机ip
    returns:
        camera_matrix: numpy array，按行存储的相机矩阵，可恢复成3x3的camera matrix，与OpenCV兼容
    """
    response = _request_get(
        API_URLS['get_camera_matrix'] % _api_address(ip)
    )
    if response is not None:
        camera_matrix = json.loads(response.content)
        return camera_matrix
    return None


@exception_handler
def get_distortion(ip: str):
    """获取2D图像对应相机的畸变参数，失败则返回None

    parameters:
        ip: 相机ip
    returns:
        distortion: numpy array，相机的畸变参数，与OpenCV兼容
    """
    response = _request_get(
        API_URLS['get_distortion'] % _api_address(ip)
    )
    if response is not None:
        distortion = json.loads(response.content)
        return distortion
    return None


@exception_handler
def get_undistort_lut(ip: str, width: int, height: int):
    response = _request_get(
        API_URLS['get_undistort_lut'] % _api_address(ip)
    )
    if response is not None:
        result = np.frombuffer(response.content, dtype=np.float32)
        return result.reshape(height, width, 2)
    return None


@exception_handler
def search_camera():
    """自动搜索相机，失败或者没有搜索到则返回None

    returns:
        found_camera: list类型，以info形式返回的搜索到的相机列表
    """
    port_list = [5665, 5666, 5667, 5668]
    found_cameras = []
    camera_hash = []
    
    sock_v4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_v6 = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    
    bound_v4 = bound_v6 = False
    for port in port_list:
        if not bound_v4:
            try:
                sock_v4.bind(("0.0.0.0", port))
                bound_v4 = True
            except Exception as ex:
                logger.error(f"IPv4 port {port}: {ex}")
                
        if not bound_v6:
            try:
                sock_v6.bind(("::", port))
                bound_v6 = True
            except Exception as ex:
                logger.error(f"IPv6 port {port}: {ex}")
                
        if bound_v4 and bound_v6:
            break
            
    if not (bound_v4 and bound_v6):
        return None
        
    sock_v4.settimeout(4)
    sock_v6.settimeout(4)
    
    try:
        mreq_v4 = struct.pack("4s4s", socket.inet_aton("224.0.0.251"), 
                             socket.inet_aton("0.0.0.0"))
        sock_v4.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq_v4)
        
        group_v6 = socket.inet_pton(socket.AF_INET6, "ff02::fb")
        mreq_v6 = group_v6 + struct.pack("@I", 0)
        sock_v6.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq_v6)
    except Exception as ex:
        logger.error(f"Failed to join multicast groups: {ex}")
        return None
    
    for _ in range(6):
        try:
            data_v4, addr_v4 = sock_v4.recvfrom(1024)
            if data_v4:
                try:
                    data = json.loads(data_v4.decode("utf-8"))
                    if '127.0.0.1' in data['ip']:
                        data['ip'] = data['ip'].replace('127.0.0.1', addr_v4[0])
                    if ':' not in data['ip']:
                        data['ip'] += ':5000'
                        
                    data_hash = hashlib.sha256(data["ip"].encode('ascii')).hexdigest()
                    if data_hash not in camera_hash:
                        camera_hash.append(data_hash)
                        found_cameras.append(data)
                        logger.info(f"Found IPv4 camera: {data}")
                except Exception as ex:
                    logger.error(f"Error processing IPv4 data: {ex}")
                    
        except socket.timeout:
            pass
        except Exception as ex:
            logger.error(f"IPv4 receive error: {ex}")
            
        try:
            data_v6, addr_v6 = sock_v6.recvfrom(1024)
            if data_v6:
                try:
                    data = json.loads(data_v6.decode("utf-8"))
                    if '127.0.0.1' in data['ip']:
                        data['ip'] = data['ip'].replace('127.0.0.1', addr_v6[0])
                    if ':' not in data['ip']:
                        data['ip'] += ':5000'
                        
                    data_hash = hashlib.sha256(data["ip"].encode('ascii')).hexdigest()
                    if data_hash not in camera_hash:
                        camera_hash.append(data_hash)
                        found_cameras.append(data)
                        logger.info(f"Found IPv6 camera: {data}")
                except Exception as ex:
                    logger.error(f"Error processing IPv6 data: {ex}")
                    
        except socket.timeout:
            pass
        except Exception as ex:
            logger.error(f"IPv6 receive error: {ex}")
    
    sock_v4.close()
    sock_v6.close()
    
    if not found_cameras:
        return None
        
    return found_cameras
