import os
import glob
import cv2
import numpy as np
import open3d as o3d
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor


# ============================================================
# User parameters
# ============================================================

cx, cy = 0.0, 0.0
filter_radius = 2.0

pcd_voxel_size = 0.1
map_voxel_size = 0.1

pose_npy_path = "/workspace/slam_output/2026-04-28_06-23-46/velodyne_poses.npy"

pcd_folder = "/data/kitti/data_odometry_velodyne/sequences_pcd/00/velodyne"

# image_2를 쓰면 P2 사용
image_folder = "/data/kitti/data_odometry_color/sequences/00/image_2"

# KITTI odometry calib.txt
calib_path = "/data/kitti/data_odometry_calib/sequences/00/calib.txt"

output_path = "/workspace/slam_output/2026-04-28_06-23-46/fused_colored.pcd"

# True면 같은 픽셀에 여러 LiDAR 점이 투영될 때 가장 가까운 점만 색칠
use_z_buffer = True

# 이미지에 투영되지 않는 점 처리 방식
# "skip"      : 색을 못 얻은 점은 맵에 넣지 않음
# "gray"      : 색을 못 얻은 점은 회색으로 저장
uncolored_policy = "skip"

# pose가 어떤 좌표계인지 설정
# 기존 코드처럼 pose @ lidar_point를 사용하고 있었다면 보통 True
pose_is_T_world_lidar = True

# 만약 pose가 T_world_camera라면 False로 바꾸세요.
# 이 경우 world point = T_world_camera @ T_camera_lidar @ lidar_point
pose_is_T_world_camera = not pose_is_T_world_lidar


# ============================================================
# Utility functions
# ============================================================

def load_kitti_calib(calib_file):
    """
    KITTI odometry calib.txt 파싱.
    반환:
        P2: 3x4 projection matrix
        T_cam_lidar: 4x4 LiDAR -> rectified camera coordinate transform
    """
    data = {}

    with open(calib_file, "r") as f:
        for line in f:
            if ":" not in line:
                continue
            key, value = line.strip().split(":", 1)
            values = np.array([float(x) for x in value.split()], dtype=np.float64)
            data[key] = values

    if "P2" not in data:
        raise ValueError("calib.txt에서 P2를 찾지 못했습니다. image_2를 쓰려면 P2가 필요합니다.")

    P2 = data["P2"].reshape(3, 4)

    # KITTI odometry는 보통 'Tr' 사용
    if "Tr" in data:
        Tr = data["Tr"].reshape(3, 4)
    elif "Tr_velo_to_cam" in data:
        Tr = data["Tr_velo_to_cam"].reshape(3, 4)
    else:
        raise ValueError("calib.txt에서 Tr 또는 Tr_velo_to_cam을 찾지 못했습니다.")

    T_velo_to_cam = np.eye(4, dtype=np.float64)
    T_velo_to_cam[:3, :4] = Tr

    # KITTI object format에는 R0_rect가 있고 odometry format에는 없는 경우가 많음
    if "R0_rect" in data:
        R0 = np.eye(4, dtype=np.float64)
        R0[:3, :3] = data["R0_rect"].reshape(3, 3)
    elif "R_rect_00" in data:
        R0 = np.eye(4, dtype=np.float64)
        R0[:3, :3] = data["R_rect_00"].reshape(3, 3)
    else:
        R0 = np.eye(4, dtype=np.float64)

    T_cam_lidar = R0 @ T_velo_to_cam

    return P2, T_cam_lidar


def normalize_poses(poses):
    """
    poses shape:
        N x 4 x 4
        N x 3 x 4
    둘 다 지원.
    """
    poses = np.asarray(poses)

    if poses.ndim != 3:
        raise ValueError(f"pose 배열 차원이 이상합니다. 현재 shape: {poses.shape}")

    if poses.shape[1:] == (4, 4):
        return poses.astype(np.float64)

    if poses.shape[1:] == (3, 4):
        out = np.tile(np.eye(4, dtype=np.float64), (poses.shape[0], 1, 1))
        out[:, :3, :4] = poses
        return out

    raise ValueError(f"지원하지 않는 pose shape입니다: {poses.shape}")


def project_lidar_to_image(points_lidar, image, P2, T_cam_lidar):
    """
    LiDAR 좌표계의 점들을 image_2 평면에 투영하고 RGB 색을 추출.

    입력:
        points_lidar: N x 3
        image: H x W x 3, BGR
        P2: 3 x 4
        T_cam_lidar: 4 x 4

    반환:
        color_mask: N bool, 색을 얻은 점 여부
        colors_rgb: N x 3, 0~1 RGB
    """
    h, w = image.shape[:2]
    n = points_lidar.shape[0]

    color_mask = np.zeros(n, dtype=bool)
    colors_rgb = np.zeros((n, 3), dtype=np.float64)

    if n == 0:
        return color_mask, colors_rgb

    points_h = np.hstack([points_lidar, np.ones((n, 1), dtype=np.float64)])

    points_cam_h = (T_cam_lidar @ points_h.T).T
    points_cam = points_cam_h[:, :3]

    z = points_cam[:, 2]
    in_front = z > 1e-6

    if not np.any(in_front):
        return color_mask, colors_rgb

    idx_front = np.where(in_front)[0]
    points_cam_h_front = points_cam_h[idx_front]

    proj = (P2 @ points_cam_h_front.T).T

    proj_z = proj[:, 2]
    valid_z = proj_z > 1e-6

    if not np.any(valid_z):
        return color_mask, colors_rgb

    idx_valid = idx_front[valid_z]
    proj = proj[valid_z]

    u = proj[:, 0] / proj[:, 2]
    v = proj[:, 1] / proj[:, 2]

    u_int = np.round(u).astype(np.int32)
    v_int = np.round(v).astype(np.int32)

    inside = (
        (u_int >= 0) & (u_int < w) &
        (v_int >= 0) & (v_int < h)
    )

    if not np.any(inside):
        return color_mask, colors_rgb

    idx_img = idx_valid[inside]
    u_img = u_int[inside]
    v_img = v_int[inside]
    depth_img = z[idx_img]

    if use_z_buffer:
        linear_pixel = v_img * w + u_img

        # pixel 번호 기준 정렬 후, 같은 pixel 안에서는 depth가 작은 점 우선
        order = np.lexsort((depth_img, linear_pixel))
        linear_sorted = linear_pixel[order]

        first = np.ones(len(order), dtype=bool)
        first[1:] = linear_sorted[1:] != linear_sorted[:-1]

        selected = order[first]

        idx_img = idx_img[selected]
        u_img = u_img[selected]
        v_img = v_img[selected]

    bgr = image[v_img, u_img, :].astype(np.float64)
    rgb = bgr[:, ::-1] / 255.0

    color_mask[idx_img] = True
    colors_rgb[idx_img] = rgb

    return color_mask, colors_rgb


def transform_points(T, points):
    """
    4x4 transform으로 N x 3 point 변환.
    """
    if points.shape[0] == 0:
        return points

    points_h = np.hstack([points, np.ones((points.shape[0], 1), dtype=np.float64)])
    points_tf = (T @ points_h.T).T[:, :3]
    return points_tf


# ============================================================
# Load data
# ============================================================

print("------------------")
print(f"Pose path   : {pose_npy_path}")
print(f"PCD path    : {pcd_folder}")
print(f"Image path  : {image_folder}")
print(f"Calib path  : {calib_path}")
print(f"Output path : {output_path}")
print("------------------")

poses = normalize_poses(np.load(pose_npy_path))
pcd_files = sorted(glob.glob(os.path.join(pcd_folder, "*.pcd")))
image_files = sorted(
    glob.glob(os.path.join(image_folder, "*.png")) +
    glob.glob(os.path.join(image_folder, "*.jpg")) +
    glob.glob(os.path.join(image_folder, "*.jpeg"))
)

P2, T_cam_lidar = load_kitti_calib(calib_path)

print(f"Num Poses : {len(poses)}")
print(f"Num PCDs  : {len(pcd_files)}")
print(f"Num Images: {len(image_files)}")

num_frames = min(len(poses), len(pcd_files), len(image_files))

if len(poses) != len(pcd_files) or len(poses) != len(image_files):
    print("경고: pose / pcd / image 개수가 다릅니다.")
    print(f"처리 프레임 수는 min 값인 {num_frames}개로 제한합니다.")

if num_frames == 0:
    raise RuntimeError("처리할 데이터가 없습니다.")


# ============================================================
# Frame processing
# ============================================================

def process_one(args):
    idx, pcd_file, image_file, pose = args

    pcd = o3d.io.read_point_cloud(pcd_file)

    if len(pcd.points) == 0:
        return None, None

    pcd = pcd.voxel_down_sample(voxel_size=pcd_voxel_size)

    points_lidar = np.asarray(pcd.points, dtype=np.float64)

    if points_lidar.shape[0] == 0:
        return None, None

    # 차량 주변부 제거
    d2 = (points_lidar[:, 0] - cx) ** 2 + (points_lidar[:, 1] - cy) ** 2
    keep_mask = d2 > (filter_radius ** 2)
    points_lidar = points_lidar[keep_mask]

    if points_lidar.shape[0] == 0:
        return None, None

    image = cv2.imread(image_file, cv2.IMREAD_COLOR)

    if image is None:
        print(f"이미지 로드 실패: {image_file}")
        return None, None

    color_mask, colors_rgb = project_lidar_to_image(
        points_lidar=points_lidar,
        image=image,
        P2=P2,
        T_cam_lidar=T_cam_lidar,
    )

    if uncolored_policy == "skip":
        points_lidar = points_lidar[color_mask]
        colors_rgb = colors_rgb[color_mask]

        if points_lidar.shape[0] == 0:
            return None, None

    elif uncolored_policy == "gray":
        gray = np.array([0.5, 0.5, 0.5], dtype=np.float64)
        colors_rgb[~color_mask] = gray

    else:
        raise ValueError(f"지원하지 않는 uncolored_policy입니다: {uncolored_policy}")

    # 좌표계 변환
    if pose_is_T_world_lidar:
        T_world_lidar = pose
    else:
        # pose가 T_world_camera인 경우
        T_lidar_cam = np.linalg.inv(T_cam_lidar)
        T_world_lidar = pose @ T_cam_lidar

        # 주의:
        # 위 식은 pose가 rectified camera 좌표계 기준인지에 따라 달라질 수 있습니다.
        # 일반적으로 T_world_cam @ T_cam_lidar @ p_lidar 형태가 맞습니다.

    points_world = transform_points(T_world_lidar, points_lidar)

    return points_world, colors_rgb


# ============================================================
# Run
# ============================================================

all_points = []
all_colors = []

pairs = [
    (i, pcd_files[i], image_files[i], poses[i])
    for i in range(num_frames)
]

max_workers = os.cpu_count() or 4

with ThreadPoolExecutor(max_workers=max_workers) as executor:
    for points, colors in tqdm(
        executor.map(process_one, pairs),
        total=len(pairs),
        desc="Colorizing frames"
    ):
        if points is not None and colors is not None:
            all_points.append(points)
            all_colors.append(colors)


# ============================================================
# Merge and save
# ============================================================

if not all_points:
    print("점군 데이터가 없습니다.")
else:
    print("Merging pointclouds")

    merged_points = np.vstack(all_points)
    merged_colors = np.vstack(all_colors)

    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
    merged_pcd.colors = o3d.utility.Vector3dVector(merged_colors)

    print(f"색칠된 원본 점 개수: {len(merged_pcd.points)}")

    down_pcd = merged_pcd.voxel_down_sample(voxel_size=map_voxel_size)

    print(f"Voxel 적용 후 점 개수: {len(down_pcd.points)}")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    o3d.io.write_point_cloud(output_path, down_pcd)

    print(f"컬러 맵 저장 완료: {output_path}")
