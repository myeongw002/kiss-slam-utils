import numpy as np
import open3d as o3d
import os
import glob

# 입력 경로 설정
pose_npy_path = '/home/myungw00/kiss_slam/slam_output/2025-07-22_16-43-16/2024-06-15-11-44-28_0_poses.npy'
ply_folder = '/home/myungw00/kiss_slam/slam_output/2025-07-22_16-43-16/pcd'
output_path = '/home/myungw00/kiss_slam/slam_output/2025-07-22_16-43-16/merged_map.ply'

# 1. pose 불러오기
poses = np.load(pose_npy_path)  # shape: (n, 4, 4)

# 2. ply 파일 정렬
ply_files = sorted(glob.glob(os.path.join(ply_folder, '*.ply')))

print(f"Num Poses: {len(poses)}, Num PCDs: {len(ply_files)}")

if len(poses) != len(ply_files):
    print(f"경고: pose 수({len(poses)})와 ply 파일 수({len(ply_files)})가 다릅니다. 인덱스 매칭 확인 필요.")

all_points = []
all_colors = []

print()
for idx, ply_file in enumerate(ply_files):
    if idx >= len(poses):
        print(f"ply 파일이 pose 개수보다 많음, 이후 파일은 무시됩니다.")
        break
    pose = poses[idx]  # shape (4, 4)
    pcd = o3d.io.read_point_cloud(ply_file)
    print(f"Pose: {pose} \n PCD: {ply_file}")
    voxel_size = 0.1  # voxel 크기(단위=ply와 동일, 예: 5cm)로 조절
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    # 3xN 점 행렬로 변환
    points = np.asarray(pcd.points)
    # homogeneous 좌표로 확장
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack([points, ones])  # (N, 4)
    # 변환 적용
    points_world = (pose @ points_h.T).T[:, :3]  # (N, 3)
    all_points.append(points_world)
    if pcd.has_colors():
        all_colors.append(np.asarray(pcd.colors))

if not all_points:
    print("점군 데이터가 없습니다.")
else:
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(np.vstack(all_points))
    if all_colors:
        merged_pcd.colors = o3d.utility.Vector3dVector(np.vstack(all_colors))
    print(f"원본 점 개수: {len(merged_pcd.points)}")

    # ---- 여기서 Voxel Downsampling 적용 ----
    voxel_size = 0.1  # voxel 크기(단위=ply와 동일, 예: 5cm)로 조절
    down_pcd = merged_pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"Voxel 적용 후 점 개수: {len(down_pcd.points)}")

    o3d.io.write_point_cloud(output_path, down_pcd)
    print(f"다운샘플링 맵 저장 완료: {output_path}")


