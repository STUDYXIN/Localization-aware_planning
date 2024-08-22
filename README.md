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
