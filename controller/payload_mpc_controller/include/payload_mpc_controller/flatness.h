#pragma once

#include <Eigen/Eigen>

#include <cmath>

namespace PayloadMPC
{
    class Flatness
    {
    public:
        // void reset(const double &quadrotor_mass,
        //            const double &payload_mass,
        //            const double &l_length,
        //            const double gravity = 9.81)
        // {
        //     mass_q_ = quadrotor_mass;
        //     mass_l_ = payload_mass;
        //     l_length_ = l_length;
        //     grav_ = gravity;
        // }

        void  qnorm(const Eigen::Vector3d &q, const Eigen::Vector3d &qd,
                   const Eigen::Vector3d &qdd, const Eigen::Vector3d &q3d,
                   Eigen::Vector3d &norm, Eigen::Vector3d &qnormd, Eigen::Vector3d &qnormdd, Eigen::Vector3d &qnorm3d)
        {
            double q_dot_q = q.squaredNorm();
            double q_norm = sqrt(q_dot_q);
            // If here need to check if q_norm is zero?
            // if (q_norm < 1e-8)
            // {
            //     norm=Eigen::Vector3d::Zero();
            //     norm.z() = -1.0;
            //     qnormd=Eigen::Vector3d::Zero();
            //     qnormdd = Eigen::Vector3d::Zero();
            //     qnorm3d = Eigen::Vector3d::Zero();
            // }
            assert(q_norm > 1e-8);

            double q_norm_3 = q_dot_q * q_norm;
            double q_norm_5 = q_dot_q * q_norm_3;
            double q_norm_7 = q_dot_q * q_norm_5;
            double q_dot_qd = q.dot(qd);
            double q_dot_qdd = q.dot(qdd);
            double qd_dot_qd = qd.dot(qd);
            double qd_dot_qdd = qd.dot(qdd);
            double q_dot_q3d = q.dot(q3d);

            double sqr_q_dot_qd = q_dot_qd * q_dot_qd;
            double q_dot_qd_3 = sqr_q_dot_qd * q_dot_qd;

            norm = q / q_norm;

            qnormd = (-q_dot_qd * q + q_dot_q * qd) / q_norm_3;

            qnormdd = (3 * (sqr_q_dot_qd)*q 
            - q_dot_q * ((q_dot_qdd + qd_dot_qd) * q 
            + 2 * q_dot_qd * qd)) / q_norm_5 + qdd / q_norm;

            qnorm3d = (-15 * q_dot_qd_3 * q 
            + 9 * q_dot_q * q_dot_qd * ((qd_dot_qd + q_dot_qdd) * q 
            + q_dot_qd * qd)) / q_norm_7 
            - ((q_dot_q3d + 3 * qd_dot_qdd) * q 
            + 3 * (qd_dot_qd + q_dot_qdd) * qd + 3 * q_dot_qd * qdd) / q_norm_3 
            + q3d / q_norm;
        }

        void normalizeFDF(const Eigen::Vector3d &x,
                             Eigen::Vector3d &xNor,
                             Eigen::Matrix3d &G)
        {
            const double a = x(0), b = x(1), c = x(2);
            const double aSqr = a * a, bSqr = b * b, cSqr = c * c;
            const double ab = a * b, bc = b * c, ca = c * a;
            const double xSqrNorm = aSqr + bSqr + cSqr;
            const double xNorm = sqrt(xSqrNorm);
            const double den = xSqrNorm * xNorm;
            xNor = x / xNorm;
            G(0, 0) = bSqr + cSqr;
            G(0, 1) = -ab;
            G(0, 2) = -ca;
            G(1, 0) = -ab;
            G(1, 1) = aSqr + cSqr;
            G(1, 2) = -bc;
            G(2, 0) = -ca;
            G(2, 1) = -bc;
            G(2, 2) = aSqr + bSqr;
            G /= den;
            // return;
        }

        // void qdotdot(const Eigen::Vector3d &q, const Eigen::Vector3d &jerk,
        //                                 Eigen::Matrix3d &G)
        // {
        //     double t0 = q.norm();
        //     double t1 = 1.0 / pow(t0, 3);
        //     double t2 = q.dot(jerk);

        //     G = -(t1 * jerk * q.transpose() + t1 * t2 * Eigen::Matrix3d::Identity() + t1 * q * jerk.transpose() - (3 * t2) * q * q.transpose() / pow(t0, 5));

        //     return;
        // }
        // inline void forward_p(const Eigen::Vector3d &pos,
        //                     const Eigen::Vector3d &vel,
        //                     const Eigen::Vector3d &acc,
        //                     const Eigen::Vector3d &jerk,
        //                     const Eigen::Vector3d &snap,
        //                     const Eigen::Vector3d &crackle,

        //                     Eigen::Vector3d &pos_quad,
        //                     Eigen::Vector3d &vel_quad,
        //                     Eigen::Vector3d &acc_quad,
        //                     Eigen::Vector3d &jerk_quad,
        //                     Eigen::Vector3d &p,
        //                     Eigen::Vector3d &pd)
        // {
        //     Eigen::Vector3d Tp = -acc; ////here is -Tp
        //     Tp(2) -= grav_;
        //     Eigen::Vector3d pdd, p3d;

        //     qnorm(Tp, -jerk, -snap, -crackle, p, pd, pdd, p3d);
        //     pos_quad = pos - l_length_ * p;
        //     vel_quad = vel - l_length_ * pd;
        //     acc_quad = acc - l_length_ * pdd;
        //     jerk_quad = jerk - l_length_ * p3d;

        // }

        /*inline void forward_rot(const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jerk,
                            const Eigen::Vector3d &acc_quad,
                            const Eigen::Vector3d &jerk_quad,
                            const Eigen::Vector3d &fq,
                            const Eigen::Vector3d &fl,
                            const double &heading,
                            const double &heading_rate,

                            Eigen::Quaterniond &quat,
                            double &thr,
                            Eigen::Vector3d &omg)
        {

            Eigen::Vector3d des_az_in_world = mass_q_ * acc_quad + mass_l_ * acc + Eigen::Vector3d(0, 0, (mass_q_ + mass_l_) * grav_) - fq - fl;

            thr = des_az_in_world.norm();

            Eigen::Vector3d z_B;
            Eigen::Matrix3d G;
            normalizeFDF(des_az_in_world, z_B, G);
            double scal = sqrt(2 * (1 + z_B(2)));
            // 1 + z_B(2) shouldn't be zero

            const double c_half_psi = cos(heading * 0.5);
            const double s_half_psi = sin(heading * 0.5);
            const double s_psi = sin(heading);
            const double c_psi = cos(heading);
            Eigen::Quaterniond qz(scal * 0.5 * c_half_psi,          //w
             (-z_B[1] * c_half_psi + z_B[0] * s_half_psi) / scal,   //x 
             (z_B[0] * c_half_psi + z_B[1] * s_half_psi) / scal,    //y
             scal * 0.5 * s_half_psi);                              //z
            quat = qz.normalized();
            Eigen::Vector3d dz = G * (mass_q_ * jerk_quad + mass_l_ * jerk);

            double omg_den = 1.0 + z_B(2);
            double omg_term = dz(2) / omg_den;

            omg(0) = dz(0) * s_psi - dz(1) * c_psi -
                    (z_B(0) * s_psi - z_B(1) * c_psi) * omg_term;
            omg(1) = dz(0) * c_psi + dz(1) * s_psi -
                    (z_B(0) * c_psi + z_B(1) * s_psi) * omg_term;
            omg(2) = (z_B(1) * dz(0) - z_B(0) * dz(1)) / omg_den + heading_rate;

            return;
        }*/
        
    // 微分平坦：计算期望的姿态、角速度
    void forward_rot(const Eigen::Vector3d &acc, // 无人机在世界系下的加速度
                     const Eigen::Vector3d &jerk,
                     double yaw,
                     const double &yaw_rate,
                     Eigen::Quaterniond &quat,
                     Eigen::Vector3d &omg,
                     double &acc_z)
    {
        Eigen::Vector3d acc_thrust = acc + Eigen::Vector3d(0, 0, 9.81); // 桨产生的加速度
        Eigen::Vector3d acc_normlized = acc_thrust.normalized(); // 无人机z轴在世界系的方向
        double a_x = acc_normlized(0), a_y = acc_normlized(1), a_z = acc_normlized(2);
        Eigen::Vector3d acc_dot;
        acc_dot = (acc_thrust.dot(acc_thrust) * Eigen::MatrixXd::Identity(3, 3) 
                     - acc_thrust * acc_thrust.transpose()) / acc_thrust.norm() / acc_thrust.squaredNorm() * jerk;
        double a_x_dot = acc_dot(0), a_y_dot = acc_dot(1), a_z_dot = acc_dot(2);
        double syaw = sin(yaw), cyaw = cos(yaw);
        if(a_z > 0)
        {
            double norm = sqrt(2 * (1 + a_z));
            Eigen::Quaterniond q((1 + a_z) / norm, -a_y / norm, a_x / norm, 0);
            Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2)); // 绕z轴旋转 yaw 的旋转
            quat = q * q_yaw;
            // NOTE: 求角速度，但是为什么这么算捏？
            omg(0) = syaw * a_x_dot - cyaw * a_y_dot - (a_x*syaw - a_y*cyaw) * a_z_dot / (a_z + 1);
            omg(1) = cyaw * a_x_dot + syaw * a_y_dot - (a_x*cyaw + a_y*syaw) * a_z_dot / (a_z + 1);
            omg(2) = (a_y * a_x_dot - a_x * a_y_dot) / (a_z + 1) + yaw_rate;
        }
        else
        {
            double norm = sqrt(2 * (1 - a_z));
            Eigen::Quaterniond q(-a_y / norm, (1 - a_z) / norm, 0, a_x / norm);
            yaw += 2 * atan2(a_x, a_y);
            Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
            quat = q * q_yaw;
            syaw = sin(yaw);
            cyaw = cos(yaw);
            // yaw_rate = yaw_rate;  // + atan2_dot(a, b, a_dot, b_dot);
            omg(0) = syaw * a_x_dot + cyaw * a_y_dot - (a_x * syaw + a_y * cyaw) * a_z_dot / (a_z - 1);
            omg(1) = cyaw * a_x_dot - syaw * a_y_dot - (a_x * cyaw - a_y * syaw) * a_z_dot / (a_z - 1);
            omg(2) = (a_y * a_x_dot - a_x * a_y_dot) / (a_z - 1) + yaw_rate;
        }
        acc_z = acc_thrust.norm();
    }  

    // private:
    //     const double kAlmostZeroAccThreshold_{0.001};
    //     const double kAlmostZeroValueThreshold_{0.001};
    };
}
