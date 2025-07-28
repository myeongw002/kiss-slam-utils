import numpy as np
import open3d as o3d
import os
import glob
from concurrent.futures import ThreadPoolExecutor

cx, cy = 0.0, 0.0
filter_radius = 2.0
pcd_voxel_size = 0.1
map_voxel_size = 0.1

pose_npy_path = '/home/myungw00/kiss_slam/slam_output/2025-02-20/2025-07-28_13-53-32/2025-02-20-14-09-46_1_poses.npy'
pcd_folder = '/home/myungw00/kiss_slam/slam_output/2025-02-20/2025-07-28_13-53-32/pcd'
output_path = '/home/myungw00/kiss_slam/slam_output/2025-02-20/2025-07-28_13-53-32/merged.pcd'

poses = np.load(pose_npy_path)
pcd_files = sorted(glob.glob(os.path.join(pcd_folder, '*.pcd')))

print(f"Num Poses: {len(poses)}, Num PCDs: {len(pcd_files)}")
if len(poses) != len(pcd_files):
    print(f"경고: pose 수({len(poses)})와 pcd 파일 수({len(pcd_files)})가 다릅니다. 인덱스 매칭 확인 필요.")

def process_one(idx_pcd_pose):
    idx, (pcd_file, pose) = idx_pcd_pose
    pcd = o3d.io.read_point_cloud(pcd_file)
    pcd = pcd.voxel_down_sample(voxel_size=pcd_voxel_size)
    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        return None, None
    d2 = (points[:,0] - cx)**2 + (points[:,1] - cy)**2
    keep_mask = d2 > (filter_radius ** 2)
    points = points[keep_mask]
    
    if points.shape[0] == 0:
        return None, None
        
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)[keep_mask]
    else:
        colors = None
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack([points, ones])
    points_world = (pose @ points_h.T).T[:, :3]
    return points_world, colors

from tqdm import tqdm
all_points = []
all_colors = []
pairs = list(enumerate(zip(pcd_files, poses)))

with ThreadPoolExecutor(max_workers=os.cpu_count()) as executor:
    for points, colors in tqdm(executor.map(process_one, pairs), total=len(pairs)):
        if points is not None:
            all_points.append(points)
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

    down_pcd = merged_pcd.voxel_down_sample(voxel_size=map_voxel_size)
    print(f"Voxel 적용 후 점 개수: {len(down_pcd.points)}")

    o3d.io.write_point_cloud(output_path, down_pcd)
    print(f"다운샘플링 맵 저장 완료: {output_path}")

