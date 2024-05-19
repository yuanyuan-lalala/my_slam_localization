#include "Eigen/Core"
#include "Eigen/Dense"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
// #include "src/common."

// template<typename S = double>
// class ESKF{

//     using SO3 = Sophus::SO3<S>;
//     using VecT = Eigen::Matrix<S, 3, 1>;            // 向量类型
//     using Vec18T = Eigen::Matrix<S, 18, 1>;         // 18维向量类型
//     using Mat3T = Eigen::Matrix<S, 3, 3>;
//     using MotionNoiseT = Eigen::Matrix<S, 18, 18>;  // 运动噪声类型
//     using OdomNoiseT = Eigen::Matrix<S, 3, 3>;      // 里程计噪声类型
//     using GnssNoiseT = Eigen::Matrix<S, 6, 6>;      // GNSS噪声类型
//     using Mat18T = Eigen::Matrix<S, 18, 18>;        // 18维方差类型
//     using NavStateT = NavState<S>;                  // 整体名义状态变量类型
//     //可以直接把参数放在类的结构体中
//     struct Options {
//         Options() = default;

//         /// IMU 测量与零偏参数
//         double imu_dt_ = 0.01;  // IMU测量间隔
//         // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
//         double gyro_var_ = 1e-5;       // 陀螺测量标准差
//         double acce_var_ = 1e-2;       // 加计测量标准差
//         double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
//         double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

//         /// 里程计参数
//         double odom_var_ = 0.5;
//         double odom_span_ = 0.1;        // 里程计测量间隔
//         double wheel_radius_ = 0.155;   // 轮子半径
//         double circle_pulse_ = 1024.0;  // 编码器每圈脉冲数

//         /// RTK 观测参数
//         double gnss_pos_noise_ = 0.1;                   // GNSS位置噪声
//         double gnss_height_noise_ = 0.1;                // GNSS高度噪声
//         double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS旋转噪声

//         /// 其他配置
//         bool update_bias_gyro_ = true;  // 是否更新陀螺bias
//         bool update_bias_acce_ = true;  // 是否更新加计bias
//     };
//      bool Predict(const IMU& imu);

// private:


//     /// 成员变量
//     double current_time_ = 0.0;  // 当前时间

//     /// 名义状态
//     VecT p_ = VecT::Zero();
//     VecT v_ = VecT::Zero();
//     SO3 R_;
//     VecT bg_ = VecT::Zero();
//     VecT ba_ = VecT::Zero();
//     VecT g_{0, 0, -9.8};

//     /// 误差状态
//     Vec18T dx_ = Vec18T::Zero();

//     /// 协方差阵
//     Mat18T cov_ = Mat18T::Identity();

//     /// 噪声阵
//     MotionNoiseT Q_ = MotionNoiseT::Zero();
//     OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
//     GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

//     /// 标志位
//     bool first_gnss_ = true;  // 是否为第一个gnss数据

//     /// 配置项
//     Options options_;

// };
// // template<typename S>
// // bool ESKF<S>::Predict(const IMU& imu){

    


// // }