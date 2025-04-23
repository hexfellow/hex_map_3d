# **hex_map_3d**

## **Overview**

This **hex_map_3d** repository provides an implementation for 3D point cloud map processing and transformation, including map filtering, transformation, and visualization.

### **License**

This project is licensed under the terms of the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

### **Maintainer**

**[Dong Zhaorui](https://github.com/IBNBlank)**
**[Zhang Jianhuai](https://github.com/aalicecc)**

### **Supported Platform**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**

### **Supported ROS Version**

- [x] **ROS Noetic**
- [ ] **ROS Humble**

---

## **Getting Started**

### **Dependencies**

- **ROS Noetic** or **ROS Humble**

### **Install**

1. Create a workspace `catkin_ws` and get into the `src`.

   ```shell
   mdkir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone this repo.

   ```shell
   git clone git@github.com:hexfellow/hex_map_3d.git
   ```

3. Go to `catkin_ws` directory and build the repo.

   ```shell
   cd ../
   catkin_make
   ```

4. Source the `setup.bash`

   ```shell
   source devel/setup.bash --extend
   ```

### **Usage**

1. Save point cloud map:

   ```shell
   rosrun hex_map_3d map_save_fast_lio_sam.sh [map_name]
   ```

2. Convert the 3D point cloud map into a grid map and record the navigation points:

   ```shell
   rosrun hex_map_3d map_preprocess.sh [map_name]
   ```

3. View the processed map:

   ```shell
   rosrun hex_map_3d map_viewer.py [map_name]
   ```
