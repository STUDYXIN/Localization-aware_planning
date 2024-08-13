#ifndef __ODOM_EKF_H
#define __ODOM_EKF_H

#include <nav_msgs/Odometry.h>
#include <pose_data.h>

class kalmanxd
{
public:
    Imu_Data_t imu_data;
    Pose_Data_t pose_data;

    ros::Publisher odom_pub;
    nav_msgs::Odometry odom_msg;

    Eigen::VectorXd X_hat_prior_; // 先验值
    bool is_initialized_;         // 初始化标志位
    kalmanxd(Eigen::VectorXd);
    // 判断是否已初始化 Q, P, H, R
    bool initialize(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd);
    bool set_Q(Eigen::MatrixXd);        // 设置过程噪声矩阵
    bool set_R(Eigen::MatrixXd);        // 设置观测噪声矩阵
    bool set_P(Eigen::MatrixXd);        // 设置协方差矩阵
    bool set_H(Eigen::MatrixXd);        // 设置观测矩阵
    bool set_G(Eigen::MatrixXd);        // 设置控制矩阵
    bool set_U(Eigen::MatrixXd);        // 设置输入矩阵
    Eigen::VectorXd kalman_measure(Eigen::VectorXd, Eigen::MatrixXd);
    ~kalmanxd();

private:
    // 滤波器相关
    Eigen::VectorXd X_hat_;   // 最优估计值
    Eigen::VectorXd B_;       // 输入项
    Eigen::VectorXd U_;       // 输入矩阵
    Eigen::MatrixXd Q_;       // 过程噪声
    Eigen::MatrixXd R_;       // 测量噪声
    Eigen::MatrixXd F_;       // 转化矩阵
    Eigen::MatrixXd H_;       // 状态观测矩阵
    Eigen::MatrixXd P_;       // 协方差
    Eigen::MatrixXd P_prior_; // 先验协方差
    Eigen::MatrixXd Kk_;      // 卡尔曼增益
    Eigen::MatrixXd Zk_;      // 当前测量量
    Eigen::MatrixXd G_;       // 控制矩阵

    // 数据更新标志位
    bool initialized_ = false;
    bool transfer_matrix_setted_ = false;
    bool Q_setted_ = false;
    bool P_setted_ = false;
    bool H_setted_ = false;
    bool R_setted_ = false;
    bool Zk_setted_ = false;
    bool G_setted_ = false;
    bool U_setted_ = false;

    void update_prior_est(Eigen::MatrixXd);                    // 先验估计
    void update_p_prior_est(void);                             // 先验协方差更新
    void update_kalman_gain(void);                             // 卡尔曼增益更新
    Eigen::VectorXd update_best_measurement(Eigen::VectorXd); // 更新最优测量/后验估计
    void update_p_postterior(void);                            // 更新状态估计协方差矩阵P
};

#endif