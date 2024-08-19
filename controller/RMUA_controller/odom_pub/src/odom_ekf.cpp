#include <odom_ekf.h>

kalmanxd::kalmanxd(Eigen::VectorXd X)
{
    X_hat_ = X;
    is_initialized_ = false;
}

bool kalmanxd::initialize(
    Eigen::MatrixXd Q,
    Eigen::MatrixXd P,
    Eigen::MatrixXd H,
    Eigen::MatrixXd R)
{
    if (
        set_Q(Q) &&
        set_P(P) &&
        set_H(H) &&
        set_R(R))
    {
        initialized_ = true;
        return true;
    }
    initialized_ = false;
    return false;
}

bool kalmanxd::set_Q(Eigen::MatrixXd Q)
{
    Q_ = Q;
    Q_setted_ = true;
    return true;
}

bool kalmanxd::set_R(Eigen::MatrixXd R)
{
    R_ = R;
    R_setted_ = true;
    return true;
}

bool kalmanxd::set_P(Eigen::MatrixXd P)
{
    P_ = P;
    P_setted_ = true;
    return true;
}

bool kalmanxd::set_H(Eigen::MatrixXd H)
{
    H_ = H;
    H_setted_ = true;
    return true;
}

bool kalmanxd::set_G(Eigen::MatrixXd G)
{
    G_ = G;
    G_setted_ = true;
    return true;
}
bool kalmanxd::set_U(Eigen::MatrixXd U)
{
    U_ = U;
    U_setted_ = true;
    return true;
}

Eigen::VectorXd kalmanxd::kalman_measure(
    Eigen::VectorXd,
    Eigen::MatrixXd)
{
    if (initialized_)
    {
        Eigen::VectorXd best_measurement;
        update_prior_est(F_);
        update_p_prior_est();
        update_kalman_gain();
        best_measurement = update_best_measurement(Zk_);
        update_p_postterior();
        return best_measurement;
    }
    throw "it didn't initialized!";
}

void kalmanxd::update_prior_est(Eigen::MatrixXd F)
{
    F_ = F;
    if (G_setted_ && U_setted_)
    {
        X_hat_prior_ = F * X_hat_ + G_ * U_;
    }
    else
    {
        X_hat_prior_ = F_ * X_hat_;
    }
}

void kalmanxd::update_p_prior_est(void)
{
    P_prior_ = F_ * P_ * F_.transpose() + Q_;
}

void kalmanxd::update_kalman_gain(void)
{
    Eigen::MatrixXd K1 = H_ * P_prior_ * H_.transpose() + R_;
    Kk_ = (P_prior_ * H_.transpose()) * K1.inverse();
}

Eigen::VectorXd kalmanxd::update_best_measurement(Eigen::VectorXd Zk)
{
    Zk_ = Zk;
    X_hat_ = X_hat_prior_ + Kk_ *(Zk_ -H_ * X_hat_prior_);
    return X_hat_;
}

void kalmanxd::update_p_postterior(void)
{
    int i = X_hat_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(i, i);
    P_ = (I - Kk_ * H_) * P_prior_;
}

kalmanxd::~kalmanxd()
{
}