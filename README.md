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

### 9月2日更新

**`path_searching`功能包：**

- 新加入了RRT*相关的代码，调用的接口如下：

  ```c++
  bool sampleBasedReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,const double start_yaw, const Eigen::Vector3d& end_pt, const double end_yaw, const double& time_lb = -1);
  ```

  然而由于没有考虑动力学，使用RRT*规划飞行的效果不是很好，并且在waypoint数量只有2个时会导致程序崩溃，懒得修，目前暂时还是采用混合Astar

- 将混合Astar的状态中加入了yaw维度，输入中加入了yaw_rate维度，目前只是简单将FOV里特征点少于阈值的节点视为Invalid，目前单次search的时间可达几百ms，后面需要优化过程减少耗时

- 后端轨迹优化暂时还没有把APACE揉进去

**`exploration_manager`功能包：**

- 加入了任务失败检测的机制，原理是开启定时器，检测当前FOV特征点是否少于阈值，如果是就触发急停，无人机停在当前位置，FSM没有后续状态跳转，具体触发部分在如下代码：

  ```C++
  void PAExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
    if (!have_odom_) {
      return;
    }
  
    // if (exec_state_ == FSM_EXEC_STATE::EMERGENCY_STOP) {
    //   return;
    // }
  
    if (!planner_manager_->checkCurrentLocalizability(odom_pos_, odom_orient_)) {
      ROS_WARN("Replan: Too few features detected==================================");
      emergency_stop_pub_.publish(std_msgs::Empty());
      transitState(EMERGENCY_STOP, "safetyCallback");
      return;
    }
  
    if (exec_state_ == FSM_EXEC_STATE::MOVE_TO_NEXT_GOAL) {
      // Check safety and trigger replan if necessary
      double dist;
      bool safe = planner_manager_->checkTrajLocalizability(dist);
      if (!safe) {
        ROS_WARN("Replan: Poor localizability detected==================================");
        transitState(PLAN_TO_NEXT_GOAL, "safetyCallback");
      }
    }
  
    if (exec_state_ == FSM_EXEC_STATE::MOVE_TO_NEXT_GOAL) {
      // Check safety and trigger replan if necessary
      double dist;
      bool safe = planner_manager_->checkTrajCollision(dist);
      if (!safe) {
        ROS_WARN("Replan: collision detected==================================");
        transitState(PLAN_TO_NEXT_GOAL, "safetyCallback");
      }
    }
  }
  ```

- 预留了重规划的状态机定义，但是还没有写跳转

### 9月3日更新

- **为了便于viewpoint的调试，本次conmit注注释掉了：**

  ```C++
  void PAExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
    ...
    if (!planner_manager_->checkCurrentLocalizability(odom_pos_, odom_orient_)) {
      ROS_WARN("Replan: Too few features detected==================================");
      emergency_stop_pub_.publish(std_msgs::Empty());
      transitState(EMERGENCY_STOP, "safetyCallback");
      return;
    }
    ...
  }
  ```

  ```C++
  int KinodynamicAstar::search(const Vector3d& start_pt, const Vector3d& start_v, const Vector3d& start_a, const double start_yaw,
      const Vector3d& end_pt, const Vector3d& end_v, const double end_yaw) {

    ...
    if (!checkLocalizability(cur_node->state)) {
      ROS_ERROR("Start point is not localizable!!!");
      return NO_PATH;
    }
    ...
  }
  ```

- **核心功能是在`perception_aware_exploration_manager`文件增加了以下部分**

  ```C++
  NEXT_GOAL_TYPE PAExplorationManager::selectNextGoal(Vector3d& next_pos, double& next_yaw) {
    ...
    // 使用A*算法搜索一个的点，这个点事这条路径上靠近终点且在free区域的最后一个点
    planner_manager_->path_finder_->reset();
    if (planner_manager_->path_finder_->search(start_pos, final_goal) == Astar::REACH_END) {
      ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
      Vector3d junction_pos;
      double junction_yaw;
      if (findJunction(ed_->path_next_goal_, junction_pos, junction_yaw)) {
        ed_->points_.push_back(junction_pos);
        ed_->yaws_.push_back(junction_yaw);
        cout << "[PAExplorationManager] ADD point: " << junction_pos << " yaw: " << junction_yaw << endl;
        ed_->visb_num_.push_back(frontier_finder_->getVisibleFrontiersNum(junction_pos, junction_yaw));
      }
    }
    ...
  ```

  - 这一部分是把A*算法搜索的点直接加入viewpoint中作为备选

  ```C++
    ...
    // Select point with highest score
    const double dg = (final_goal - start_pos).norm();
    vector<pair<size_t, double>> gains;
    for (size_t i = 0; i < ed_->points_.size(); ++i) {
      double visb_score = static_cast<double>(ed_->visb_num_[i]) / static_cast<double>(ep_->visb_max);
      double goal_score = (dg - (final_goal - ed_->points_[i]).norm()) / dg;
      double feature_score = static_cast<double>(feature_map_->get_NumCloud_using_justpos(ed_->points_[i])) /
                            static_cast<double>(ep_->feature_num_max);
      double motioncons_score =
          std::sin((ed_->points_[i] - start_pos).dot(start_vel) / ((ed_->points_[i] - start_pos).norm() * start_vel.norm())) /
          (M_PI / 2);
      double score = ep_->we * visb_score + ep_->wg * goal_score + ep_->wf * feature_score + ep_->wc * motioncons_score;
      cout << "[PAExplorationManager] SCORE DEUBUG NUM: " << i << " visb_score: " << visb_score << " goal_score: " << goal_score
          << " feature_score: " << feature_score << " score: " << score << " motioncons_score: " << motioncons_score << endl;
      gains.emplace_back(i, score);
    }
    ...
  }
  ```

  - 修改了`score`的构成，四个量都被归一化，其中`wc`的范围是(-1,1)
  - 对应的`algorithm.xml`文件增加了以下调参内容

    ```xml
    <param name="exploration/feature_num_max" value="200" type="int"/>
    <param name="exploration/visb_max" value="200" type="int"/>
    <param name="exploration/we" value="0.2" type="double"/>
    <param name="exploration/wg" value="1.0" type="double"/>
    <param name="exploration/wf" value="1.0" type="double"/>
    <param name="exploration/wc" value="0.2" type="double"/>
    ```

    - 其中 we约束这个viewpoint可以看到的frontier点数
    - 其中 wg约束到目标的距离
    - 其中 wf约束可见特征点数量（不考虑YAW）
    - 其中 we约束运动一致性（防止无人机来回移动）

  - 相应的，在`path_searching`,`active_perception`,`plan_env`都有所修改，主要目的是增加`A*`和`feature_score`的接口。  

-**TODO**
  还未找到解决报错的办法
