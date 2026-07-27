# PROJECT FILE HEADER
# 文件：apps/camera/ring.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import os
import time
import random
import json
import sys
import cv2
import numpy as np
import open3d as o3d
from contextlib import contextmanager
from pathlib import Path
from typing import Tuple, List, Optional, Dict, Any
from ultralytics import YOLO

# 导入 epiceye SDK（假设已安装）
import epiceye

# ============================================================
#  配置管理（同原有）
# ============================================================
class OCircleFitConfig:
    def __init__(self):
        repo_root = Path(__file__).resolve().parents[2]
        self.yolo_model_path = str(repo_root / "artifacts" / "models" / "camera_weights" / "best.onnx")
        self.image_path_template = str(repo_root / "artifacts" / "camera_test_data" / "camera_{id}.jpg")
        self.pointcloud_path_template = str(repo_root / "artifacts" / "camera_test_data" / "{id}_outer_ring.ply")
        self.output_dir = str(repo_root / "artifacts" / "camera_test_data")
        # -------------------- 检测与裁剪参数 --------------------
        self.expand_pixels = 5      # 检测框扩展像素数，使裁剪区域包含更多边缘点云，避免边界点丢失
        self.voxel_size = 0.28           # 点云体素降采样尺寸（物理单位，与点云坐标系一致），越大点云越稀疏，处理越快

        # -------------------- 圆环半径范围（物理单位） --------------------
        self.outer_radius_min = 10.0     # 外圈半径最小值（用于RANSAC过滤，剔除不合理圆）
        self.outer_radius_max = 15.0     # 外圈半径最大值
        self.inner_radius_min = 4.0      # 内圈半径最小值
        self.inner_radius_max = 6.5      # 内圈半径最大值

        # -------------------- 拟合与统计滤波参数 --------------------
        self.epoch = 5                   # 对每个圆进行多次RANSAC拟合的次数，取中位数作为最终结果（抗离群值）
        self.stat_nb_neighbors = 20      # 统计滤波的邻域点数（每个点附近采样点数）
        self.stat_std_ratio = 2.0        # 统计滤波的标准差倍数（超出该倍数的点视为离群点）
        self.plane_iterations = 200      # 平面拟合（RANSAC）的最大迭代次数，越大找到正确平面的概率越高
        self.ransac_iters = 800          # 2D圆拟合（RANSAC）的最大迭代次数，影响拟合稳定性和耗时

        # -------------------- 点云有效性阈值 --------------------
        self.point_valid_threshold = 0.001  # 判断点是否有效的距离阈值（物理单位），小于该值的点被视为无效点（如原点或噪声）

import yaml
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
REPO_ROOT = APP_DIR.parents[1]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


class CircleFitConfig:
    """
    新版配置类：从 YAML 文件加载所有配置，支持跨环境迁移。
    """
    def __init__(self, config_path: str = "config.yaml"):
        self.app_dir = APP_DIR
        self.project_root = REPO_ROOT
        
        # 加载 YAML 配置文件
        config_file = Path(config_path)
        if not config_file.is_absolute():
            config_file = self.app_dir / config_file
        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)
        
        # ---- 路径处理：所有路径基于项目根目录拼接 ----
        paths = cfg['paths']
        self.yolo_model_path = str(resolve_repo_path(paths['yolo_model']))
        self.image_path_template = str(resolve_repo_path(paths['image_template']))
        self.pointcloud_path_template = str(resolve_repo_path(paths['pointcloud_template']))
        self.output_dir = str(resolve_repo_path(paths['output_dir']))
        # ---- 检测与裁剪参数 ----
        det = cfg['detection']
        self.expand_pixels = det['expand_pixels']   # 框扩展像素
        self.voxel_size = det['voxel_size']         # 体素下采样尺寸
        
        # ---- 半径范围 ----
        rad = cfg['radius']
        self.outer_radius_min = rad['outer_min']    # 外圈最小半径
        self.outer_radius_max = rad['outer_max']    # 外圈最大半径
        self.inner_radius_min = rad['inner_min']    # 内圈最小半径
        self.inner_radius_max = rad['inner_max']    # 内圈最大半径
        
        # ---- 拟合与滤波参数 ----
        fit = cfg['fitting']
        self.epoch = fit['epoch']                           # RANSAC 重复次数
        self.stat_nb_neighbors = fit['stat_nb_neighbors']   # 统计滤波邻域点数
        self.stat_std_ratio = fit['stat_std_ratio']         # 统计滤波标准差倍数
        self.plane_iterations = fit['plane_iterations']     # 平面拟合迭代次数
        self.ransac_iters = fit['ransac_iters']             # 圆拟合迭代次数
        self.point_valid_threshold = fit['point_valid_threshold']  # 有效点阈值

        # ---- 拟合与滤波参数 ----
        camera = cfg['camera']
        self.ExpTime2D = camera['ExpTime2D']                           # RANSAC 重复次数
        self.ProjectorBrightness = camera['ProjectorBrightness']   # 统计滤波邻域点数
        self.Gain2D = camera['Gain2D']         # 统计滤波标准差倍数
        self.FlashLightOn = camera['FlashLightOn']     # 平面拟合迭代次数
        self.ParamsBatch3D = camera['ParamsBatch3D']             # 圆拟合迭代次数

        
        # 自动创建输出目录
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)


# ============================================================
#  工具函数
# ============================================================
@contextmanager
def timed_step(label: str, timings: Optional[List[Tuple[str, float]]] = None):
    start = time.perf_counter()
    try:
        yield
    finally:
        elapsed = time.perf_counter() - start
        print(f"[耗时] {label}: {elapsed:.3f}s")
        if timings is not None:
            timings.append((label, elapsed))


def expand_bbox(x_center, y_center, width, height, img_w, img_h, expand_pixels=10):
    """
    将 YOLO 输出的归一化框 (cx, cy, w, h) 转换为扩展后的像素坐标框。
    扩展可避免裁剪时刚好切掉边界上的关键点。
    """
    x1 = (x_center - width / 2) * img_w - expand_pixels
    y1 = (y_center - height / 2) * img_h - expand_pixels
    x2 = (x_center + width / 2) * img_w + expand_pixels
    y2 = (y_center + height / 2) * img_h + expand_pixels
    # 限制在图像范围内
    x1 = max(0, int(x1)); y1 = max(0, int(y1))
    x2 = min(img_w, int(x2)); y2 = min(img_h, int(y2))
    return x1, y1, x2, y2


def filter_pointcloud(pcd: o3d.geometry.PointCloud,
                      nb_neighbors: int = 20,
                      std_ratio: float = 2.0) -> Tuple[o3d.geometry.PointCloud, float]:
    """
    统计滤波：剔除离群点，返回滤波后的点云和平均点间距。
    原理：计算每个点到其最近 nb_neighbors 个点的平均距离，超出均值+std_ratio*标准差则剔除。
    """
    # 执行统计滤波，返回离群点布尔掩码和内点索引
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pcd_filtered = pcd.select_by_index(ind)
    points = np.asarray(pcd_filtered.points)
    if len(points) < 50:
        raise ValueError("有效点云太少，无法拟合圆")
    # 计算平均点间距，用于自适应阈值
    distances = pcd_filtered.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    return pcd_filtered, avg_dist


def fit_plane_model(pcd_filtered: o3d.geometry.PointCloud,
                    avg_dist: float,
                    plane_threshold: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
    """
    使用 RANSAC 拟合平面，返回平面模型参数 [a,b,c,d] 和平面内点。
    ax + by + cz + d = 0
    """
    if plane_threshold is None:
        plane_threshold = avg_dist * 3   # 自适应阈值：3 倍平均点间距
    plane_model, inliers = pcd_filtered.segment_plane(
        distance_threshold=plane_threshold,
        ransac_n=3,
        num_iterations=200
    )
    pcd_plane = pcd_filtered.select_by_index(inliers)
    plane_points = np.asarray(pcd_plane.points)
    print(f"拟合平面方程: {plane_model[0]:.3f}x + {plane_model[1]:.3f}y + {plane_model[2]:.3f}z + {plane_model[3]:.3f} = 0")
    print(f"提取到的台阶面点云数量: {len(plane_points)}")
    return plane_model, plane_points


def fit_circle_ransac_2d(points_2d: np.ndarray,
                         max_iters: int = 800,
                         threshold: float = 0.1,
                         r_min: Optional[float] = None,
                         r_max: Optional[float] = None) -> Tuple[Optional[np.ndarray], float, List[int]]:
    """
    2D RANSAC 圆拟合，返回 (圆心2D, 半径, 内点索引列表)。
    核心思想：随机取 3 点确定一个圆，统计落在圆环（阈值内）的点数，迭代取最优。
    最后用最小二乘对最优内点进行精修。
    """
    best_inliers = []
    best_center = None
    best_radius = 0.0
    n_points = len(points_2d)
    if n_points < 5:
        return None, 0.0, []

    for _ in range(max_iters):
        try:
            idx = random.sample(range(n_points), 3)
        except ValueError:
            continue
        p1, p2, p3 = points_2d[idx[0]], points_2d[idx[1]], points_2d[idx[2]]
        # 三点共线则跳过（无法确定圆）
        if abs(np.cross(p2 - p1, p3 - p1)) < 1e-6:
            continue
        d1, d2 = p2 - p1, p3 - p1
        # 解线性方程组：M * center = N
        # 由 (p_i - center)^2 = (p_j - center)^2 推导
        M = np.array([d1, d2])
        N = np.array([np.dot(p2, p2) - np.dot(p1, p1),
                      np.dot(p3, p3) - np.dot(p1, p1)]) / 2.0
        try:
            center = np.linalg.solve(M, N)
        except np.linalg.LinAlgError:
            continue
        radius = np.linalg.norm(center - p1)
        # 半径过滤（剔除明显不合理的圆）
        if r_min is not None and radius < r_min: continue
        if r_max is not None and radius > r_max: continue
        # 统计内点
        distances = np.linalg.norm(points_2d - center, axis=1)
        inlier_indices = np.where(np.abs(distances - radius) < threshold)[0]
        if len(inlier_indices) > len(best_inliers):
            best_inliers = inlier_indices
            best_center = center
            best_radius = radius

    # 最小二乘精修：基于内点重新拟合，提高精度
    if best_center is not None and len(best_inliers) > 5:
        inlier_pts = points_2d[best_inliers]
        x, y = inlier_pts[:, 0], inlier_pts[:, 1]
        A_ls = np.c_[x, y, np.ones_like(x)]
        B_ls = x**2 + y**2
        try:
            cx_ls, cy_ls, C_ls = np.linalg.lstsq(A_ls, B_ls, rcond=None)[0]
            center_ls = np.array([cx_ls, cy_ls]) / 2.0
            radius_sq = (cx_ls**2 + cy_ls**2) / 4.0 + C_ls
            if radius_sq < 0 and radius_sq > -1e-8:
                radius_sq = 0.0
            radius_ls = np.sqrt(radius_sq)
            if (r_min is None or radius_ls >= r_min) and (r_max is None or radius_ls <= r_max):
                return center_ls, radius_ls, best_inliers
        except np.linalg.LinAlgError:
            pass
    return best_center, best_radius, best_inliers


def fit_circle_3d(plane_points: np.ndarray,
                  plane_model: np.ndarray,
                  avg_dist: float,
                  radius_min: Optional[float] = None,
                  radius_max: Optional[float] = None,
                  circle_threshold: Optional[float] = None) -> Tuple[Optional[np.ndarray], float]:
    """
    将平面上的 3D 点投影到 2D 坐标系，进行圆拟合，再映射回 3D 空间。
    返回 (圆心3D, 半径)。
    """
    if circle_threshold is None:
        circle_threshold = avg_dist * 3

    [a, b, c, d] = plane_model
    normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
    p0 = -d * normal  # 平面上一点
    # 构造局部 2D 坐标系 (u, v) 位于平面内
    ref_vector = np.array([1, 0, 0]) if abs(normal[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(normal, ref_vector)
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)  # u 与 v 正交

    # 将点投影到平面上的 2D 坐标
    delta = plane_points - p0
    coords_2d = np.array([np.dot(delta, u), np.dot(delta, v)]).T

    # 2D RANSAC 圆拟合
    center_2d, radius, _ = fit_circle_ransac_2d(
        coords_2d,
        max_iters=800,
        threshold=circle_threshold,
        r_min=radius_min,
        r_max=radius_max
    )
    if center_2d is None:
        return None, 0.0
    # 反投影回 3D
    center_3d = p0 + center_2d[0] * u + center_2d[1] * v
    return center_3d, radius


# ============================================================
#  基础处理类（支持离线文件模式）
# ============================================================
class RingFitProcessor:
    """
    离线模式处理器：读取本地图像和点云文件，进行 YOLO 检测、点云裁剪和圆拟合。
    """
    def __init__(self, config: CircleFitConfig):
        self.config = config
        self.timings = []
        # When PLY output is disabled, fitting uses these in-memory clouds.
        self._cropped_pointclouds: Dict[str, o3d.geometry.PointCloud] = {}
        s = time.time()
        self.model = YOLO(config.yolo_model_path)
        print(f"模型加载耗时: {time.time()-s:.3f}s")

    def detect_and_crop(self, image_id: int) -> Dict[str, Any]:
        """
        执行 YOLO 检测，根据检测框裁剪点云，保存为 PLY 文件。
        """
        img_path = self.config.image_path_template.format(id=image_id)
        results = self.model(img_path)
        img_h, img_w = results[0].orig_shape
        img = cv2.imread(img_path)

        pc_path = self.config.pointcloud_path_template.format(id=image_id)
        pcd = o3d.io.read_point_cloud(pc_path)
        points = np.asarray(pcd.points)

        bboxes_pixel = []
        result = results[0]
        boxes = result.boxes.xywhn.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)

        for box, cls_id in zip(boxes, classes):
            x_c, y_c, w, h = box
            x1, y1, x2, y2 = expand_bbox(x_c, y_c, w, h, img_w, img_h,
                                         expand_pixels=self.config.expand_pixels)
            bboxes_pixel.append((x1, y1, x2, y2))
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        out_img_path = Path(self.config.output_dir) / f"{image_id}.jpg"
        out_img_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_img_path), img)

        output_files = {}
        self._cropped_pointclouds = {}
        for i, (x1, y1, x2, y2) in enumerate(bboxes_pixel):
            label = "inner" if classes[i] == 1 else "outer"
            if self.config.only_outer and label != "outer":
                continue
            try:
                # 尝试用有序点云方式裁剪（假设点云可 reshape 成 H×W×3）
                cropped_pcd = crop_pointcloud_by_bbox(
                    points, img_h, img_w, (x1, y1, x2, y2),
                    voxel_size=self.config.voxel_size,
                    valid_threshold=self.config.point_valid_threshold
                )
            except Exception as e:
                print(f"裁剪失败: {e}")
                continue
            self._cropped_pointclouds[label] = cropped_pcd
            out_file = Path(self.config.output_dir) / f"{image_id}_{label}_ring.ply"
            if self.config.save_ply:
                out_file.parent.mkdir(parents=True, exist_ok=True)
                ok = o3d.io.write_point_cloud(str(out_file), cropped_pcd)
                if not ok:
                    raise RuntimeError(f"保存 {label} 点云失败: {out_file}")
                print(f"已保存 {label} 点云到 {out_file}")
            else:
                print(f"已跳过 {label} PLY 保存，后续直接使用内存点云")
            output_files[label] = str(out_file)

        return {
            "image_id": image_id,
            "bboxes": bboxes_pixel,
            "classes": classes.tolist(),
            "output_files": output_files
        }

    def fit_ring_centers(
        self, image_id: int
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        从已裁剪的点云文件中拟合外圈和内圈的圆心。
        对每个圆进行多次 RANSAC（epoch 次），取中位数作为最终结果。
        """
        config = self.config
        outer_file = Path(config.output_dir) / f"{image_id}_outer_ring.ply"
        raw_outer = self._cropped_pointclouds.get("outer")
        if raw_outer is None and not outer_file.exists():
            print(f"外圈点云文件 {outer_file} 不存在")
            return None, None, None

        # ---- 外圈拟合 ----
        if raw_outer is None:
            with timed_step("外圈读取点云", self.timings):
                raw_outer = o3d.io.read_point_cloud(str(outer_file))
        else:
            print("外圈使用内存点云，跳过 PLY 读取")
        with timed_step("外圈统计滤波", self.timings):
            pcd_outer, avg_dist_outer = filter_pointcloud(
                raw_outer,
                nb_neighbors=config.stat_nb_neighbors,
                std_ratio=config.stat_std_ratio
            )
        with timed_step("外圈平面拟合", self.timings):
            plane_model_outer, plane_points_outer = fit_plane_model(
                pcd_outer, avg_dist_outer, plane_threshold=avg_dist_outer * 3
            )
        outer_normal = plane_model_outer[:3]
        outer_centers = []
        for i in range(config.epoch):
            with timed_step(f"外圈圆拟合 RANSAC {i + 1}/{config.epoch}", self.timings):
                center, radius = fit_circle_3d(
                    plane_points_outer,
                    plane_model_outer,
                    avg_dist_outer,
                    radius_min=config.outer_radius_min,
                    radius_max=config.outer_radius_max,
                    circle_threshold=avg_dist_outer * 3
                )
            if center is not None:
                outer_centers.append(center)
            print(f"外圈拟合半径: {radius}")
        outer_center = np.median(outer_centers, axis=0) if outer_centers else None
        print(f"外圈圆心 (中位数): {outer_center[0]:.8f} {outer_center[1]:.8f} {outer_center[2]:.8f}")

        if config.only_outer:
            print("仅拟合外圈，跳过内圈处理")
            return outer_center, None, outer_normal

        # ---- 内圈拟合 ----
        inner_file = Path(config.output_dir) / f"{image_id}_inner_ring.ply"
        raw_inner = self._cropped_pointclouds.get("inner")
        if raw_inner is None and not inner_file.exists():
            print(f"内圈点云文件 {inner_file} 不存在")
            return outer_center, None, outer_normal

        if raw_inner is None:
            with timed_step("内圈读取点云", self.timings):
                raw_inner = o3d.io.read_point_cloud(str(inner_file))
        else:
            print("内圈使用内存点云，跳过 PLY 读取")
        with timed_step("内圈统计滤波", self.timings):
            pcd_inner, avg_dist_inner = filter_pointcloud(
                raw_inner,
                nb_neighbors=config.stat_nb_neighbors,
                std_ratio=config.stat_std_ratio
            )
        with timed_step("内圈平面拟合", self.timings):
            plane_model_inner, plane_points_inner = fit_plane_model(
                pcd_inner, avg_dist_inner, plane_threshold=avg_dist_inner * 3
            )
        inner_centers = []
        for i in range(config.epoch):
            with timed_step(f"内圈圆拟合 RANSAC {i + 1}/{config.epoch}", self.timings):
                center, radius = fit_circle_3d(
                    plane_points_inner,
                    plane_model_inner,
                    avg_dist_inner,
                    radius_min=config.inner_radius_min,
                    radius_max=config.inner_radius_max,
                    circle_threshold=avg_dist_inner * 3
                )
            if center is not None:
                inner_centers.append(center)
            print(f"内圈拟合半径: {radius}")
        inner_center = np.median(inner_centers, axis=0) if inner_centers else None

        # ---- 保存结果 ----
        if outer_center is not None and inner_center is not None:
            out_poly = Path(config.output_dir) / f"{image_id}_ring.poly"
            with timed_step("保存 poly 结果", self.timings):
                out_poly.parent.mkdir(parents=True, exist_ok=True)
                with open(out_poly, 'w') as f:
                    f.write(f"{outer_center[0]:.8f} {outer_center[1]:.8f} {outer_center[2]:.8f}\n")
                    f.write(f"{inner_center[0]:.8f} {inner_center[1]:.8f} {inner_center[2]:.8f}\n")
            print(f"结果已保存至 {out_poly}")
        return outer_center, inner_center, outer_normal

    def run(self, image_id: int) -> Dict[str, Any]:
        """完整流程：检测裁剪 + 圆拟合"""
        start = time.time()
        print(f"开始处理 ID = {image_id}")
        crop_info = self.detect_and_crop(image_id)
        outer_c, inner_c, outer_normal = self.fit_ring_centers(image_id)
        elapsed = time.time() - start
        print(f"总耗时: {elapsed:.2f} 秒")
        return {
            "image_id": image_id,
            "outer_center": outer_c.tolist() if outer_c is not None else None,
            "inner_center": inner_c.tolist() if inner_c is not None else None,
            "outer_normal": outer_normal.tolist() if outer_normal is not None else None,
            "crop_info": crop_info,
            "elapsed": elapsed
        }


# ============================================================
#  辅助函数：点云裁剪
# ============================================================
def crop_organized_pointcloud(point_map: np.ndarray,
                              bbox: Tuple[int, int, int, int],
                              voxel_size: float = 0.28,
                              valid_threshold: float = 0.001) -> o3d.geometry.PointCloud:
    """
    从有序点云 (H, W, 3) 中根据像素框直接切片裁剪。
    适用于相机实时采集的有序点云（与图像像素一一对应）。
    """
    x1, y1, x2, y2 = bbox
    cropped = point_map[y1:y2, x1:x2]
    cropped_flat = cropped.reshape(-1, 3)
    mask = np.all(np.isfinite(cropped_flat), axis=1) & (np.linalg.norm(cropped_flat, axis=1) > valid_threshold)
    valid = cropped_flat[mask]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(valid)
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return pcd


def crop_pointcloud_by_bbox(points: np.ndarray,
                            img_h: int,
                            img_w: int,
                            bbox: Tuple[int, int, int, int],
                            voxel_size: float = 0.28,
                            valid_threshold: float = 0.001) -> o3d.geometry.PointCloud:
    """
    根据像素框裁剪点云（假设点云可重排为有序矩阵）。
    如果点数不等于 img_h*img_w，会抛出异常。
    """
    x1, y1, x2, y2 = bbox
    try:
        points_organized = points.reshape((img_h, img_w, 3))
    except ValueError as e:
        raise ValueError("点云点数与图像尺寸不匹配，无法重排为有序矩阵") from e

    cropped_points = points_organized[y1:y2, x1:x2]
    cropped_flat = cropped_points.reshape(-1, 3)
    mask = np.all(np.isfinite(cropped_flat), axis=1) & (np.linalg.norm(cropped_flat, axis=1) > valid_threshold)
    valid_cropped = cropped_flat[mask]

    cropped_pcd = o3d.geometry.PointCloud()
    cropped_pcd.points = o3d.utility.Vector3dVector(valid_cropped)
    if voxel_size > 0:
        cropped_pcd = cropped_pcd.voxel_down_sample(voxel_size=voxel_size)
    return cropped_pcd


# ============================================================
#  相机处理类（继承自 RingFitProcessor）
# ============================================================
class CameraRingFitProcessor(RingFitProcessor):
    """
    相机模式处理器：从 epiceye 3D 相机实时采集图像和有序点云，
    进行 YOLO 检测、点云裁剪和圆拟合。
    """
    def __init__(self, config: CircleFitConfig):
        super().__init__(config)
        self.camera_matrix = None
        self.distortion = None

    def capture_from_camera(self, ip: str) -> Tuple[np.ndarray, np.ndarray, str]:
        """
        触发相机采集，返回 (8-bit 图像, 有序点云 (H,W,3), frame_id)
        """
        # 自动补全端口。已经传入 IP 时直接连接，避免每次 search_camera() 等待广播发现。
        if ip and ':' not in ip:
            ip = ip + ':5000'

        if ip:
            print("using ip: ", ip)
        else:
            with timed_step("搜索相机", self.timings):
                found_camera = epiceye.search_camera()
            print(found_camera)
            if found_camera is not None:
                ip = found_camera[0]["ip"]
                print("using ip: ", ip)
            else:
                raise RuntimeError("No camera found!")

        skip_optional_camera_requests = os.environ.get("RING_SKIP_OPTIONAL_CAMERA_REQUESTS") == "1"

        # 获取相机内参（只做一次）
        if skip_optional_camera_requests:
            print("[跳过] 获取相机内参和畸变")
        elif self.camera_matrix is None:
            with timed_step("获取相机内参", self.timings):
                self.camera_matrix = epiceye.get_camera_matrix(ip)
            with timed_step("获取相机畸变", self.timings):
                self.distortion = epiceye.get_distortion(ip)
            if self.camera_matrix is None:
                print("警告: 无法获取相机内参，当前流程未使用内参，继续执行")

        # ---- 设置相机参数 ----
        if skip_optional_camera_requests:
            print("[跳过] 读取并设置相机配置")
        else:
            with timed_step("读取并设置相机配置", self.timings):
                epiceye_config = epiceye.get_config(ip)
                if epiceye_config:
                    # 2D 曝光时间（控制彩色图亮度，单位 ms）
                    epiceye_config["ExpTime2D"] = self.config.ExpTime2D
                    # 投影仪亮度（影响点云质量，非彩色图亮度）
                    epiceye_config["ProjectorBrightness"] = self.config.ProjectorBrightness
                    # 2D 增益（增加噪点风险，尽量保持 0）
                    epiceye_config["Gain2D"] = self.config.Gain2D
                    # 开启白光补光灯（适合暗环境）
                    epiceye_config["FlashLightOn"] = self.config.FlashLightOn
                    # 3D 曝光时间（影响深度图/点云质量）
                    if epiceye_config["ParamsBatch3D"]:
                        epiceye_config["ParamsBatch3D"][0]["ExpTime3D"] = self.config.ParamsBatch3D
                epiceye.set_config(ip=ip, config=epiceye_config)
                current_config = epiceye.get_config(ip)
            print(f"相机配置: {current_config}")

        # ---- 触发采集 ----
        with timed_step("触发相机采集", self.timings):
            frame_id = epiceye.trigger_frame(ip=ip, pointcloud=True)
        if frame_id is None:
            raise RuntimeError("触发帧失败")

        # ---- 获取图像（10-bit 转 8-bit） 获取点云（有序 H×W×3） ----
        with timed_step("获取图像并转 8bit", self.timings):
            image_raw, point_map = epiceye.get_image_and_point_cloud(
                                                                ip=ip,
                                                                frame_id=frame_id,
                                                            )
            if image_raw is None:
                raise RuntimeError("获取图像失败")
            
            if point_map is None:
                raise RuntimeError("获取点云失败")
            image_8bit = cv2.convertScaleAbs(image_raw, alpha=(255.0 / 1024.0))
        return image_8bit, point_map, frame_id

    def detect_and_crop_from_camera(self, ip: str, image_id: int = 0) -> Dict[str, Any]:
        """
        相机模式的核心处理：采集 → YOLO 检测 → 按类别筛选最佳框 → 有序裁剪 → 保存点云
        """
        # 1. 采集
        with timed_step("相机采集总计", self.timings):
            image_8bit, point_map, frame_id = self.capture_from_camera(ip)
        img_h, img_w = image_8bit.shape[:2]

        # 2. YOLO 推理
        with timed_step("YOLO 推理", self.timings):
            results = self.model(image_8bit)
            result = results[0]
            boxes = result.boxes.xywhn.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confs = result.boxes.conf.cpu().numpy()
        print(f"检测到的类别: {classes}")

        # ---- 每个类别只保留置信度最高的框 ----
        with timed_step("筛选检测框", self.timings):
            unique_classes = np.unique(classes)
            filtered_boxes = []
            filtered_classes = []
            for cls in unique_classes:
                mask = (classes == cls)
                cls_confs = confs[mask]
                if len(cls_confs) == 0:
                    continue
                best_idx = np.argmax(cls_confs)
                cls_indices = np.where(mask)[0]
                best_original_idx = cls_indices[best_idx]
                filtered_boxes.append(boxes[best_original_idx])
                filtered_classes.append(cls)
            boxes = np.array(filtered_boxes)
            classes = np.array(filtered_classes)

        # 3. 绘制检测框并保存图像
        with timed_step("绘制并保存检测图", self.timings):
            img_draw = image_8bit.copy()
            bboxes_pixel = []
            for box, cls_id in zip(boxes, classes):
                x_c, y_c, w, h = box
                x1, y1, x2, y2 = expand_bbox(x_c, y_c, w, h, img_w, img_h,
                                             expand_pixels=self.config.expand_pixels)
                bboxes_pixel.append((x1, y1, x2, y2))
                cv2.rectangle(img_draw, (x1, y1), (x2, y2), (0, 0, 255), 2)

            out_img_path = Path(self.config.output_dir) / f"camera_{image_id}.jpg"
            out_img_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_img_path), img_draw)

        # 4. 裁剪点云（有序切片，直接索引像素坐标）
        output_files = {}
        self._cropped_pointclouds = {}
        for i, (x1, y1, x2, y2) in enumerate(bboxes_pixel):
            label = "inner" if classes[i] == 1 else "outer"
            if self.config.only_outer and label != "outer":
                continue
            with timed_step(f"{label} 点云裁剪", self.timings):
                cropped_pcd = crop_organized_pointcloud(
                    point_map,
                    (x1, y1, x2, y2),
                    voxel_size=self.config.voxel_size,
                    valid_threshold=self.config.point_valid_threshold
                )
            self._cropped_pointclouds[label] = cropped_pcd
            out_file = Path(self.config.output_dir) / f"{image_id}_{label}_ring.ply"
            if self.config.save_ply:
                with timed_step(f"{label} 点云保存", self.timings):
                    out_file.parent.mkdir(parents=True, exist_ok=True)
                    ok = o3d.io.write_point_cloud(str(out_file), cropped_pcd)
                    if not ok:
                        raise RuntimeError(f"保存 {label} 点云失败: {out_file}")
            else:
                print(f"已跳过 {label} PLY 保存，后续直接使用内存点云")
            output_files[label] = str(out_file)
            if self.config.save_ply:
                print(f"已保存 {label} 点云到 {out_file}")

        return {
            "image_id": image_id,
            "bboxes": bboxes_pixel,
            "classes": classes.tolist(),
            "output_files": output_files,
            "frame_id": frame_id
        }

    def run_camera_pipeline(self, ip: str, image_id: int = 0) -> Dict[str, Any]:
        """完整相机流程：采集 → 检测裁剪 → 圆拟合"""
        start = time.time()
        self.timings = []
        print(f"开始处理相机 {ip}，ID={image_id}")

        with timed_step("采集检测裁剪总计", self.timings):
            crop_info = self.detect_and_crop_from_camera(ip, image_id)
        if not crop_info["output_files"]:
            print("未检测到任何目标，退出")
            return {"error": "no target"}

        with timed_step("圆心拟合总计", self.timings):
            outer_c, inner_c, outer_normal = self.fit_ring_centers(image_id)
        elapsed = time.time() - start
        print(f"总耗时: {elapsed:.2f} 秒")
        print("耗时汇总:")
        for label, seconds in self.timings:
            print(f"  {label}: {seconds:.3f}s")
        return {
            "image_id": image_id,
            "outer_center": outer_c.tolist() if outer_c is not None else None,
            "inner_center": inner_c.tolist() if inner_c is not None else None,
            "outer_normal": outer_normal.tolist() if outer_normal is not None else None,
            "crop_info": crop_info,
            "elapsed": elapsed,
            "timings": self.timings
        }


# ============================================================
#  主程序入口
# ============================================================
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="相机采集、点云拟合和圆心计算")
    parser.add_argument("camera_ip", nargs="?", help="相机 IP，例如 192.168.1.66")
    parser.add_argument(
        "--no-save-ply",
        dest="save_ply",
        action="store_false",
        help="不保存和重新读取 PLY，直接使用内存点云拟合",
    )
    parser.add_argument(
        "--only-outer",
        action="store_true",
        help="只拟合外圈，跳过内圈点云和内圈圆拟合",
    )
    cli_args = parser.parse_args()

    config = CircleFitConfig()          # 从 config.yaml 加载配置
    config.epoch = 3                    # 拟合次数，可临时覆盖
    config.save_ply = cli_args.save_ply
    config.only_outer = cli_args.only_outer

    if cli_args.camera_ip is not None:
        # ---- 相机模式 ----
        camera_ip = cli_args.camera_ip
        print(f"相机模式，IP={camera_ip}")
        processor = CameraRingFitProcessor(config)
        import datetime
        image_id = int(datetime.datetime.now().timestamp()) % 10000
        result = processor.run_camera_pipeline(camera_ip, image_id)
        print("最终结果:", result)
    else:
        # ---- 离线模式 ----
        print("离线模式，批量处理文件")
        processor = RingFitProcessor(config)
        ids = [203, 108, 14, 4]
        for img_id in ids:
            processor.run(img_id)
