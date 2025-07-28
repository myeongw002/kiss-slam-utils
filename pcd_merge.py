#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import open3d as o3d
import os
import glob

def merge_pcds(pcd_paths, output_path, voxel_downsample=None):
    """
    여러 PCD 파일을 읽어 하나로 합친 뒤 저장합니다.

    :param pcd_paths: 합칠 PCD 파일 경로 리스트
    :param output_path: 합쳐진 PCD를 저장할 파일 경로
    :param voxel_downsample: (선택) 합친 뒤 voxel downsampling 할 voxel size (float)
    """
    merged = o3d.geometry.PointCloud()
    for idx, path in enumerate(pcd_paths):
        if not os.path.exists(path):
            print(f"경고: 파일이 없습니다: {path}")
            continue
        print(f"Found pcd : {path}")    
        pcd = o3d.io.read_point_cloud(path)
        merged += pcd
        

    # VOXEL 다운샘플링
    if voxel_downsample is not None and voxel_downsample > 0:
        merged = merged.voxel_down_sample(voxel_downsample)

    # 시각화 및 저장
    o3d.visualization.draw_geometries([merged])
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    o3d.io.write_point_cloud(output_path, merged)
    print(f"병합된 PCD 저장 완료: {output_path}")


if __name__ == "__main__":
    # 경로 설정 (코드 내에서 직접 수정)
    INPUT_DIR = "/home/myungw00/kiss_slam/slam_output/2024-06-15/2025-07-23_15-35-25/local_maps/plys"
    OUTPUT_PCD = "/home/myungw00/kiss_slam/slam_output/2024-06-15/2025-07-23_15-35-25/local_maps/merged.pcd"
    VOXEL_SIZE = None  # 예: 0.02

    # 디렉터리 내 모든 PCD 파일 검색
    if os.path.isdir(INPUT_DIR):
        pcd_files = glob.glob(os.path.join(INPUT_DIR, "*.ply"))
        pcd_files.sort()
    else:
        pcd_files = [INPUT_DIR]  # 단일 파일도 처리

    if not pcd_files:
        print(f"오류: 입력 경로에서 PCD 파일을 찾을 수 없습니다: {INPUT_DIR}")
        exit(1)

    merge_pcds(pcd_files, OUTPUT_PCD, voxel_downsample=VOXEL_SIZE)

