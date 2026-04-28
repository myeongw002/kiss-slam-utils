# kiss-slam-utils

A collection of Python utility scripts for post-processing [KISS-SLAM](https://github.com/PRBonn/kiss-slam) outputs — merging per-frame point clouds into a global map, colorizing maps with camera images, extracting point clouds from ROS bags, and merging arbitrary PCD/PLY files.

---

## Scripts

### `map_fusion.py`
Fuses all per-frame `.pcd` files produced by KISS-SLAM into a single downsampled global point cloud map.

**What it does:**
1. Loads per-frame poses (`.npy`) and the corresponding `.pcd` files from KISS-SLAM output.
2. For each frame: voxel-downsamples the scan, removes points within a configurable radius around the sensor origin, and transforms the remaining points to world coordinates using the pose matrix.
3. Merges all frames and applies a final voxel downsampling.
4. Saves the merged map as a `.pcd` file.

**Key parameters (edit at the top of the script):**

| Parameter | Description |
|---|---|
| `cx`, `cy` | Sensor origin offset for near-point filtering |
| `filter_radius` | Radius (m) around the sensor to discard points |
| `pcd_voxel_size` | Voxel size (m) applied per-frame before fusion |
| `map_voxel_size` | Voxel size (m) applied to the final merged map |
| `pose_npy_path` | Path to the `.npy` pose file from KISS-SLAM |
| `pcd_folder` | Directory containing per-frame `.pcd` files |
| `output_path` | Output path for the merged `.pcd` file |

**Dependencies:** `numpy`, `open3d`, `tqdm`

---

### `map_fusion_color.py`
Same as `map_fusion.py` but additionally colorizes each LiDAR point by projecting it onto the corresponding camera image using KITTI-format calibration data.

**What it does:**
1. Loads per-frame poses, `.pcd` files, and camera images.
2. Parses KITTI `calib.txt` to extract the LiDAR→camera projection matrix (`Tr`) and camera projection matrix (`P2`).
3. For each frame: removes near-sensor points, projects LiDAR points onto the image plane using a z-buffer for depth disambiguation, and samples the RGB color at each projected pixel.
4. Transforms colored points to world coordinates, merges all frames, voxel-downsamples, and saves.

**Key parameters (edit at the top of the script):**

| Parameter | Description |
|---|---|
| `cx`, `cy` | Sensor origin offset for near-point filtering |
| `filter_radius` | Radius (m) around the sensor to discard points |
| `pcd_voxel_size` | Voxel size (m) applied per-frame |
| `map_voxel_size` | Voxel size (m) applied to the final merged map |
| `pose_npy_path` | Path to the `.npy` pose file |
| `pcd_folder` | Directory containing per-frame `.pcd` files |
| `image_folder` | Directory containing camera images (`.png`/`.jpg`) |
| `calib_path` | Path to KITTI `calib.txt` |
| `output_path` | Output path for the colorized `.pcd` file |
| `use_z_buffer` | If `True`, only the nearest LiDAR point per pixel is colored |
| `uncolored_policy` | `"skip"` removes uncolored points; `"gray"` keeps them as gray |
| `pose_is_T_world_lidar` | Set `True` if pose represents T\_world\_lidar (default); `False` for T\_world\_camera |

**Dependencies:** `numpy`, `open3d`, `opencv-python`, `tqdm`

---

### `pcd_extractor.py`
Extracts per-frame point clouds from a ROS bag file and saves each frame as a numbered `.pcd` file.

**What it does:**
1. Opens a ROS bag and reads all messages on a configurable point cloud topic.
2. Saves each message as a zero-indexed `.pcd` file (`cloud_0000.pcd`, `cloud_0001.pcd`, …).

**Key parameters (edit at the top of the script):**

| Parameter | Description |
|---|---|
| `bag_path` | Path to the input `.bag` file |
| `pointcloud_topic` | ROS topic name (e.g. `/velodyne_points`) |
| `out_dir` | Output directory for the extracted `.pcd` files |

**Dependencies:** `rosbag` (ROS), `sensor_msgs` (ROS), `open3d`, `numpy`

> **Note:** This script requires a working ROS (1) environment with `rosbag` and `sensor_msgs` installed.

---

### `pcd_merge.py`
Simple utility that merges a directory of `.pcd` or `.ply` files into a single output file, with optional voxel downsampling and interactive visualization.

**What it does:**
1. Reads all `.ply` (or `.pcd`) files from the input directory.
2. Concatenates them into one `PointCloud` object.
3. Optionally applies voxel downsampling.
4. Visualizes the merged cloud interactively and saves it to disk.

**Key parameters (edit the constants in `__main__`):**

| Parameter | Description |
|---|---|
| `INPUT_DIR` | Directory containing `.ply` / `.pcd` files to merge |
| `OUTPUT_PCD` | Output file path |
| `VOXEL_SIZE` | Voxel size for downsampling (`None` to skip) |

**Dependencies:** `open3d`

---

## Installation

```bash
pip install numpy open3d tqdm opencv-python
```

For `pcd_extractor.py`, a ROS 1 installation is additionally required.

---

## Typical Workflow

```
ROS bag
   └─► pcd_extractor.py  →  per-frame .pcd files
                                      │
                          KISS-SLAM poses (.npy)
                                      │
                          map_fusion.py          (geometry-only map)
                          map_fusion_color.py    (colorized map, needs images + calib)
                                      │
                                 merged .pcd
                                      │
                          pcd_merge.py           (optional: merge multiple maps)
```
