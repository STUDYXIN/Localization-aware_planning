## 更新日志

### 8月22日更新

- **新增特征点地图**：现在支持特征点地图，数量限制在250个左右。
- **增加交互接口**：
  - 函数
  `int FeatureMap::get_NumCloud_using_PosOrient(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, vector<Eigen::Vector3d> &res)`：
    - **功能**：输入任意位姿，导出可见的特征点云及数量。
    - **用法**：可以参考 `void FeatureMap::sensorposCallback(const geometry_msgs::PoseStampedConstPtr& pose)` 函数，此函数实时发布当前里程计可见的特征点云。

- **当前缺陷**：
  - 有时会看到障碍物后面的特征点。这是读取的文件的点云是随机采样，判断是否可见考虑连线上有没有占据带你，故有时会看到障碍物后面的特征点。
  - 多远的特诊点都可以看到。这是由于可见特征点的深度设置的很大。

- **TODO**：
  - 在当前框架下生成轨迹。

### 8月25日更新

- **清理无关地图**：移除了无关地图文件，避免仓库中累积过多文件。

- **新增文件**：
  - **`right_corner_sample5.ply`**：此文件解决了看到障碍物的问题，但在某些情况下可能会导致卡顿。
  - **`right_corner_sample10.ply`**：此文件能够防止 `fronter` 生成在地图之外，但有时特征点可能会穿透障碍物。

- **规划上的改动**：
  - 尝试增加一个到目标的状态，未完成，但可以将 `algorith.xml` 文件中的配置保持为 `<param name="fsm/flight_type" value="0" type="int"/>`就是原本的fuel。

- **使用方法**：
  - 理论上，直接运行 `start.sh` 即可。

- **TODO**：
  - 尚未添加 Clang Format 配置文件。
  - `/home/star/ActiveSlam/fuel_planner/src/Localization-aware_planning/fuel_planner/utils/lkh_tsp_solver/resource` 目录下存在一些中间文件，在每次上传时可能会导致冲突，需解决此问题。

### 8月26日更新

- **修改地图文件**：调整了地图文件以防止特征点穿透障碍物的问题。

- **接收触发器**：在地图接收到触发器时，地图发布将停止，以避免系统出现卡顿现象。

- **使用方法**：
  - 理论上，直接运行 `start.sh` 即可。

### 8月29日更新

- **安装 clang-format**： `sudo apt install clang-format` 在vscode插件中搜索安装clang-format

- **手动进行clang-format** 在`ClangFormat.py`的目录打开终端，运行 `python3 ClangFormat.py`,弹出一系列被格式化的文件，则格式化成功，注：目前只格式化`.cpp` 和 `.h`文件

- **自动进行clang-format** 保存的时候自动格式化，注意可以在使用 `Ctrl+Shift+I` 确保编辑器中的代码文件，选择 "Format Document"。

### 8月30日更新

- **FeatureMap::get_NumCloud_using_CamPosOrient**

  **功能：**  
  根据`相机的位置和方向`，返回相机视野中的特征点云及其数量。

  **函数声明与重载：**

  ```cpp
  int get_NumCloud_using_CamPosOrient(
        const Eigen::Vector3d& pos, 
        const Eigen::Quaterniond& orient, 
        vector<Eigen::Vector3d>& res);
  ```

  **参数：**
  - `pos`：相机的位置（在世界坐标系中）。
  - `orient`：相机的方向（以四元数表示）。
  - `res`：输出参数，包含相机视野中的特征点云。

  **返回值：**
  - 返回在相机视野中的特征点数量。  

  ```cpp
  int get_NumCloud_using_CamPosOrient(
        const Eigen::Vector3d& pos,
        const Eigen::Quaterniond& orient);
  ```

  **参数：**
  - `pos`：相机的位置（在世界坐标系中）。
  - `orient`：相机的方向（以四元数表示）。

  **返回值：**
  - 返回在相机视野中的特征点数量。  

- **FeatureMap::get_NumCloud_using_Odom**

  **功能：**  
  根据`odom信息`，返回相机视野中的特征点云及其数量。

  **函数声明与重载：**

  ```cpp
  int get_NumCloud_using_Odom(
      const Eigen::Vector3d& pos,
      const Eigen::Quaterniond& orient,
      vector<Eigen::Vector3d>& res);
  ```

  **参数：**
  - `pos`：odom的位置（在世界坐标系中）。
  - `orient`：odom的方向（以四元数表示）。
  - `res`：输出参数，包含相机视野中的特征点云。

  **返回值：**
  - 返回在相机视野中的特征点数量。  

  ```cpp
  int get_NumCloud_using_Odom(
      const nav_msgs::OdometryConstPtr& msg,
      vector<Eigen::Vector3d>& res);
  ```

  **参数：**
  - `msg`：包含无人机位姿和方向的里程计消息。
  - `res`：输出参数，包含相机视野中的特征点云。

  **返回值：**
  - 返回在相机视野中的特征点数量。  

  ```cpp
  int get_NumCloud_using_Odom(
      const Eigen::Vector3d& pos,
      const Eigen::Quaterniond& orient);
  ```

  **参数：**
  - `pos`：odom的位置（在世界坐标系中）。
  - `orient`：odom的方向（以四元数表示）。

  **返回值：**
  - 返回在相机视野中的特征点数量。  

  ```cpp
  int get_NumCloud_using_Odom(
      const nav_msgs::OdometryConstPtr& msg);
  ```

  **参数：**
  - `msg`：包含无人机位姿和方向的里程计消息。

  **返回值：**
  - 返回在相机视野中的特征点数量。  
