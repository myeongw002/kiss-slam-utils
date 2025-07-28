import numpy as np
import open3d as o3d
import os
import glob

# --- 원 범위 파라미터 ---
cx, cy = 0.0, 0.0   # 원의 중심 좌표
filter_radius = 2.0       # 반지름

pose_npy_path = '/home/myungw00/kiss_slam/slam_output/2024-06-15/2025-07-28_13-35-57/2024-06-15-11-49-21_1_poses.npy'
pcd_folder = '/home/myungw00/kiss_slam/slam_output/2024-06-15/2025-07-28_13-35-57/pcd'
output_path = '/home/myungw00/kiss_slam/slam_output/2024-06-15/2025-07-28_13-35-57/merged.pcd'

poses = np.load(pose_npy_path)  # shape: (n, 4, 4)
pcd_files = sorted(glob.glob(os.path.join(pcd_folder, '*.pcd')))

print(f"Num Poses: {len(poses)}, Num PCDs: {len(pcd_files)}")
if len(poses) != len(pcd_files):
    print(f"경고: pose 수({len(poses)})와 pcd 파일 수({len(pcd_files)})가 다릅니다. 인덱스 매칭 확인 필요.")

all_points = []
all_colors = []

for idx, pcd_file in enumerate(pcd_files):
    if idx >= len(poses):
        print(f"pcd 파일이 pose 개수보다 많음, 이후 파일은 무시됩니다.")
        break
    pose = poses[idx]  # shape (4, 4)
    pcd = o3d.io.read_point_cloud(pcd_file)
    print(f"Pose: {pose} \nPCD: {pcd_file}")
    voxel_size = 0.1  # voxel 크기
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # --- 원 범위 내 점 제거 ---
    points = np.asarray(pcd.points)
    d2 = (points[:,0] - cx)**2 + (points[:,1] - cy)**2
    keep_mask = d2 > (filter_radius ** 2)
    points = points[keep_mask]
    
    # 색상 정보도 같이 필터링
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)[keep_mask]
    else:
        colors = None

    if points.shape[0] == 0:
        continue

    # homogeneous 좌표로 확장 및 pose 적용
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack([points, ones])
    points_world = (pose @ points_h.T).T[:, :3]
    all_points.append(points_world)
    if colors is not None:
        all_colors.append(colors)

if not all_points:
    print("점군 데이터가 없습니다.")
else:
    print("Merging pointclouds")
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(np.vstack(all_points))
    if all_colors:
        merged_pcd.colors = o3d.utility.Vector3dVector(np.vstack(all_colors))
    print(f"원본 점 개수: {len(merged_pcd.points)}")

    # ---- 여기서 Voxel Downsampling 적용 ----
    voxel_size = 0.1  # voxel 크기
    down_pcd = merged_pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"Voxel 적용 후 점 개수: {len(down_pcd.points)}")

    o3d.io.write_point_cloud(output_path, down_pcd)
    print(f"다운샘플링 맵 저장 완료: {output_path}")

