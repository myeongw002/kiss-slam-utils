# kiss-slam-utils

Python utilities for post-processing KISS-SLAM outputs. This repository includes scripts for extracting LiDAR point clouds from ROS bags, merging per-frame point clouds into a global map using KISS-SLAM poses, and generating colorized maps by projecting LiDAR points onto KITTI-format camera images.

## Files

| File | Description |
| --- | --- |
| `pcd_extractor.py` | Extracts `sensor_msgs/PointCloud2` messages from a ROS bag and saves each frame as a `.pcd` file |
| `map_fusion.py` | Builds a geometry-only global point cloud map from per-frame `.pcd` files and pose `.npy` data |
| `map_fusion_color.py` | Builds a colorized global point cloud map using KITTI images and calibration data |
| `pcd_merge.py` | Merges multiple `.ply` or `.pcd` files into a single point cloud |
| `Dockerfile` | Ubuntu-based Docker environment for KISS-SLAM and post-processing |
| `docker_command` | Example Docker run command |

## Features

- Extract PCD files from ROS bag point cloud topics such as `/velodyne_points`
- Transform local per-frame point clouds into the world frame using pose matrices
- Remove points close to the vehicle or sensor origin
- Apply voxel downsampling per frame and on the final merged map
- Project LiDAR points onto camera images using KITTI `calib.txt` entries such as `P2`, `Tr`, or `Tr_velo_to_cam`
- Load, merge, save, and visualize point clouds with Open3D

## Requirements

For the basic post-processing scripts:

```bash
pip install numpy open3d tqdm opencv-python
```

`pcd_extractor.py` additionally requires a ROS 1 environment with `rosbag` and `sensor_msgs`.

```bash
sudo apt install ros-<distro>-rosbag ros-<distro>-sensor-msgs
```

For example, use `noetic` as `<distro>` if you are working with ROS Noetic.

## Usage

The scripts currently use hard-coded parameters instead of command-line arguments. Before running a script, edit the paths and parameters near the top of the file or inside the `__main__` block.

### 1. Extract PCD files from a ROS bag

Edit `pcd_extractor.py`:

```python
bag_path = "/path/to/input.bag"
pointcloud_topic = "/velodyne_points"
out_dir = "/path/to/output_pcd_dir"
```

Run:

```bash
python3 pcd_extractor.py
```

Example output:

```text
cloud_0000.pcd
cloud_0001.pcd
cloud_0002.pcd
```

### 2. Merge KISS-SLAM PCD outputs into a map

Edit `map_fusion.py`:

```python
pose_npy_path = "/path/to/poses.npy"
pcd_folder = "/path/to/pcd"
output_path = "/path/to/merged.pcd"
```

Key parameters:

| Parameter | Description |
| --- | --- |
| `filter_radius` | Radius around the sensor origin to remove, in meters |
| `pcd_voxel_size` | Voxel size applied to each frame before fusion |
| `map_voxel_size` | Voxel size applied to the final merged map |

Run:

```bash
python3 map_fusion.py
```

### 3. Generate a colorized point cloud map

Edit `map_fusion_color.py`:

```python
pose_npy_path = "/path/to/velodyne_poses.npy"
pcd_folder = "/path/to/velodyne_pcd"
image_folder = "/path/to/image_2"
calib_path = "/path/to/calib.txt"
output_path = "/path/to/fused_colored.pcd"
```

This script assumes KITTI odometry-style calibration files. When using `image_2`, the calibration file must contain `P2`. The LiDAR-to-camera transform is loaded from `Tr` or `Tr_velo_to_cam`.

Key parameters:

| Parameter | Description |
| --- | --- |
| `use_z_buffer` | If `True`, only the nearest LiDAR point is colored when multiple points project to the same pixel |
| `uncolored_policy` | `"skip"` removes uncolored points, while `"gray"` keeps them with a gray color |
| `pose_is_T_world_lidar` | Set to `True` if poses are `T_world_lidar`; set to `False` for `T_world_camera` |

Run:

```bash
python3 map_fusion_color.py
```

### 4. Merge multiple PCD/PLY files

Edit the `__main__` block in `pcd_merge.py`:

```python
INPUT_DIR = "/path/to/input_dir"
OUTPUT_PCD = "/path/to/merged.pcd"
VOXEL_SIZE = None  # Example: 0.02
```

Run:

```bash
python3 pcd_merge.py
```

The current script searches for `*.ply` files in the input directory. To merge `.pcd` files instead, change the glob pattern from `*.ply` to `*.pcd`.

## Docker

Build the Docker image:

```bash
docker build \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -t kissslam .
```

The `docker_command` file contains an example command that connects GPU, X11 GUI, workspace, and dataset volumes. Update the `-v` mount paths for your machine before running it.

```bash
docker run -it --rm \
    --name kissslam_con \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-docker \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /path/to/kissslam_ws:/workspace \
    -v /path/to/dataset:/data \
    kissslam bash
```

If you want to open Open3D visualization windows from inside the container, the host may need to allow local Docker X11 access:

```bash
xhost +local:docker
```

## Example Workflow

```text
ROS bag
  -> pcd_extractor.py
  -> per-frame PCD files
  -> run KISS-SLAM and generate pose.npy
  -> map_fusion.py or map_fusion_color.py
  -> merged.pcd / fused_colored.pcd
```

## Notes

- If the number of PCD files and poses differs, check frame indexing and file ordering.
- `map_fusion_color.py` matches poses, PCD files, and images by sorted filename order.
- If projected colors are misaligned, first check the calibration file, camera index, and pose coordinate-frame setting.
- Large datasets can use a lot of memory during merging. Increase `pcd_voxel_size` or `map_voxel_size` to reduce map density.
