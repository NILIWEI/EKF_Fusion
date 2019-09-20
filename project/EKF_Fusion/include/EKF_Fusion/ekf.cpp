//
// Created by nlw on 9/18/19.
//

#include <iostream>
#include <queue>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>    //  IMU的信息格式
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>  //  输出的Odometry格式

#include <Eigen/Eigen>

static ros::Publisher odom_pub; //  用于发布的里程计信息

bool flag_g_init = false;       //  重力向量是否初始化
bool flag_odom_init = false;

Eigen::VectorXd x(16);          //  四元数(4)，位置(3)，线速度(3)，线加速度(3)，角速度(3)
Eigen::MatrixXd P = 100 * Eigen::MatrixXd::Identity(15 , 15);   //协方差矩阵
Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12 , 12);
Eigen::MatrixXd Rt = Eigen::MatrixXd::Identity(6 , 6);          //旋转平移，一般不都是4*4吗？
Eigen::Vector3d g_init = Eigen::Vector3d::Zero();               //陀螺仪的重力初始化向量
Eigen::Vector3d G;                                              //陀螺仪的重力向量

typedef Eigen::Matrix<double  , 16 , 1> Vector16d;
typedef Eigen::Matrix<double  , 15 , 15> Matrix15d;
std::queue<Vector16d> x_history;        //这里可能由于编译器的问题  会将 >> 识别为移位操作符
std::queue<Matrix15d> P_history;

int cnt_g_int = 0;          //用于保存g初始化的次数
double time_stamp;          //时间戳

/// 用于缓存    IMU 以及 Odom 的队列
std::queue<sensor_msgs::Imu::ConstPtr> imu_buffer;
std::queue<sensor_msgs::Imu::ConstPtr> odom_buffer;

/// 相机相对于IMU的外参
Eigen::Matrix3d imu_R_cam = Eigen::Quaterniond(0 , 0 , 1 , 0).toRotationMatrix();
Eigen::Vector3d imu_t_cam = Eigen::Vector3d(0 , -0.04 , -0.02);
Eigen::Matrix3d Rcam;

//! @brief  发布里程计数据
//! \param header
void pub_odom(std_msgs::Header header)
{
    Eigen::Matrix3d vicon_R_tag;
    vicon_R_tag << 0 , 1 , 0 , 1 , 0 , 0 , 0 , 0 , -1;
    Eigen::Quaterniond Quat(x(0) , x(1) , x(2) , x(3));
    Quat = vicon_R_tag * Quat;      //四元数和矩阵的乘法 @TODO四元数和矩阵谁在前后的问题

    Eigen::Vector3d vicon_p  ,vicon_v;  // position 以及 线速度
    vicon_p = vicon_R_tag * Eigen::Vector3d(x(4) , x(5) , x(6));
    vicon_v = vicon_R_tag * Eigen::Vector3d(x(7) , x(8) , x(9));

    nav_msgs::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = "world";
    odom.pose.pose.position.x = vicon_p(0);
    odom.pose.pose.position.y = vicon_p(1);
    odom.pose.pose.position.z = vicon_p(2);
    odom.pose.pose.orientation.w = Quat.w();
    odom.pose.pose.orientation.x = Quat.x();
    odom.pose.pose.orientation.y = Quat.y();
    odom.pose.pose.orientation.z = Quat.z();
    odom.twist.twist.linear.x = vicon_v(0);
    odom.twist.twist.linear.y = vicon_v(1);
    odom.twist.twist.linear.z = vicon_v(2);

    odom_pub.publish(odom);                 //  通过发布器将里程计数据发布
}

//! @brief 预测和更新circle中的预测
//! //! \param imu_msg
void propagate(const sensor_msgs::ImuConstPtr &imu_msg)
{
    ROS_INFO("propagetion");
    double cur_time = imu_msg->header.stamp.toSec();    //IMU信息的时间戳  秒

    /// 线加速度和角速度
    Eigen::VectorXd w(3) , a(3);
    a(0) = imu_msg->linear_acceleration.x;
    a(1) = imu_msg->linear_acceleration.y;
    a(2) = imu_msg->linear_acceleration.z;

    w(0) = imu_msg->angular_velocity.x;
    w(1) = imu_msg->angular_velocity.y;
    w(2) = imu_msg->angular_velocity.z;

    double delta_t = cur_time - time_stamp;
    ROS_INFO("last  time is %f" , time_stamp);
    ROS_INFO("current  time is %f" , cur_time);
    ROS_INFO("delta  time is %f" , delta_t);

    if (delta_t <= 0 )  //  主要是处理特殊情况，时间戳静止不动或者倒流的
    {
        ROS_BREAK();
    }
    Eigen::Quaterniond quat(x(0) , x(1) , x(2) , x(3));
    // vector.segment<n>(i) :   向量从下标i之后的连续n个数的向量 ，
    // x.segment<3>(4)表示    [ x(4) , x(5) , x(6) ]

    ///  计算加速度大小
    Eigen::Vector3d delta_linear_acc = quat * (a - x.segment<3>(10)) - G;

    /// 更新 位置   利用的就是速度位移公式  s1 = s0 + v0*t + 0.5 * a * t * t
    x.segment<3>(4) += x.segment<3>(7) * delta_t + 0.5 * delta_linear_acc * delta_t * delta_t;

    /// 更新线速度  利用速度加速度公式    v1 = v0 + a*t
    x.segment<3>(7) += delta_linear_acc * delta_t;

    //@TODO 这里的计算方法是什么
    Eigen::Vector3d delta_angular_v = w - x.segment<3>(13);
    delta_angular_v *= 0.5 * delta_t;   //  旋转的角度
    //@TODO 这里由3维的delta_av向量如何到delta_q
    Eigen::Quaterniond delta_q(std::sqrt(1 - delta_angular_v.squaredNorm()) , delta_angular_v(0) , delta_angular_v(1) , delta_angular_v(2));

    /// 更新四元数
    Eigen::Quaterniond cur_q;
    cur_q = (quat * delta_q).normalized();
    x.segment<4>(0) << cur_q.w() , cur_q.x() , cur_q.y() , cur_q.z();

    Eigen::Vector3d w_x = w - x.segment<3>(13);
    Eigen::Vector3d a_x = a - x.segment<3>(10);
    Eigen::Matrix3d R_w_x , R_a_x;

    //  向量对应的矩阵，但是要保证矩阵是反对称的
    R_w_x << 0      , -w_x(2) , w_x(1),
            w_x(2)  , 0       , w_x(0),
            -w_x(1) , -w_x(0) , 0;

    R_a_x<< 0       , -a_x(2) , a_x(1),
            a_x(2)  , 0       , -a_x(0),
            -a_x(1) , a_x(0)  , 0;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(15 , 15);
    //  matrix.block<m , n>(i , j) i行j列开始
    A.block<3 , 3>(0 , 0) = -R_w_x;
    A.block<3 , 3>(0 , 12) = -1 * Eigen::MatrixXd::Identity(3 , 3);
    A.block<3,3>(3,6) = Eigen::MatrixXd::Identity(3,3);
    A.block<3,3>(6,0) = (-1 * quat.toRotationMatrix()) * R_a_x;
    A.block<3,3>(6,9) = (-1 * quat.toRotationMatrix());
    std::cout << "A:" << A << std::endl;

    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(15 , 12);
    U.block<3,3>(0,0) = -1 * Eigen::MatrixXd::Identity(3,3);
    U.block<3,3>(6,3) = -1 * quat.toRotationMatrix();
    U.block<3,3>(9,6) = Eigen::MatrixXd::Identity(3,3);
    U.block<3,3>(12,9) = Eigen::MatrixXd::Identity(3,3);

    Eigen::MatrixXd F , V;
    F = Eigen::MatrixXd::Identity(15 , 15) + A * delta_t;   //  F为状态方程函数的一阶偏导数的雅克比矩阵
    V = U * delta_t;
    ///  Pk(先) = F * Pk-1(后) * F.transpose + Rk       Pk为先验 ， Pk-1为后验
    P = F * P * F.transpose() + V * Q * V.transpose();
    time_stamp = cur_time;
}

//! 预测和更新circle中的更新阶段
//! \param msg
void update(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("update");
    Eigen::Quaterniond Quat_r;
    Quat_r.w() = msg->pose.pose.orientation.w;
    Quat_r.x() = msg->pose.pose.orientation.x;
    Quat_r.y() = msg->pose.pose.orientation.y;
    Quat_r.z() = msg->pose.pose.orientation.z;
    Eigen::Matrix3d cam_R_w = Quat_r.toRotationMatrix();

    Eigen::Vector3d cam_t_w;
    cam_t_w(0) = msg->pose.pose.position.x;
    cam_t_w(1) = msg->pose.pose.position.y;
    cam_t_w(2) = msg->pose.pose.position.z;

    //@TODO 这里的意思待定  IMU在world坐标系下的 R t
    Eigen::Matrix3d R = cam_R_w.transpose() * imu_R_cam.transpose();
    Eigen::Vector3d t = -cam_R_w.transpose() * (imu_R_cam.transpose() * imu_t_cam + cam_t_w);

    ///  H为观测方程的函数的一阶偏导数
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6 , 15);
    H.block<3 , 3>(0 , 0) = Eigen::Matrix3d::Identity();
    H.block<3 , 3>(3 , 3) = Eigen::Matrix3d::Identity();

    ///  卡尔曼增益  K = P(先)* H.transpose()*(H*P(先)*H.transpose() + Q).inverse()  ,Q为观测方程噪声的高斯分布的方差
    Eigen::MatrixXd K(15 , 6);
    K = P * H.transpose() * (H * P * H.transpose() + Rt).inverse();


    /*****          更新方程            ****/
    ///  1  更新整个状态向量x(16)  公式：  x(后) = x(先) + K * (z - h(x(先)))
    Eigen::VectorXd r(6);
    Eigen::Quaterniond qm(R);
    Eigen::Quaterniond q(x(0) , x(1) , x(2) , x(3));
    Eigen::Quaterniond delta_q = q.conjugate() * qm;    // q.conjugate()    为q的共轭四元数
    r.head<3>() = 2 * delta_q.vec();
    r.tail<3>() = t - x.segment<3>(4);              //@TODO 这里的r是怎么算的
    //     _r向量为15维 ， 为  K * (z - h(x(先))
    Eigen::VectorXd _r  = K * r;

    ///  计算四元数
    Eigen::Vector3d delta_w(_r(0) / 2 , _r(1) / 2 , _r(2) / 2);
    delta_q = Eigen::Quaterniond(1 , delta_w(0) ,delta_w(1) , delta_w(2)).normalized();
    q = q * delta_q;
    ///  更新四元数
    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3)  =q.z();
    ///  更新其他数
    x.segment<12>(4) += _r.tail(12);

    /// 2   更新后验概率 P(后) = ( I - K*H)*P(先)
    P = P - K * H * P;
}


void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_INFO("imu callback  ,TIME : %f"  ,msg->header.stamp.toSec());

    //  这里可以理解为30帧进行重新初始化 g_init
    if(!flag_g_init && cnt_g_int < 30)
    {
        Eigen::Vector3d a;
        a(0) = msg->linear_acceleration.x;
        a(1) = msg->linear_acceleration.y;
        a(2) = msg->linear_acceleration.z;
        cnt_g_int++;
        g_init += a;
    }
    if (!flag_g_init && cnt_g_int == 30)
    {
        g_init /= cnt_g_int;        //相当于30帧时，求平均的 g 是多少
        flag_g_init = true;
    }
    if (flag_g_init && flag_odom_init)
    {
        imu_buffer.push(msg);
        propagate(msg);
        x_history.push(x);
        P_history.push(P);
        pub_odom(msg->header);
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("odom callback , TIME : %f" , msg->header.stamp.toSec());
    if(!flag_odom_init)
    {
        double cur_time = msg->header.stamp.toSec();
        Eigen::Quaterniond Quat_r;
        Quat_r.w() = msg->pose.pose.orientation.w;
        Quat_r.x() = msg->pose.pose.orientation.x;
        Quat_r.y() = msg->pose.pose.orientation.y;
        Quat_r.z() = msg->pose.pose.orientation.z;
        Eigen::Matrix3d cam_R_w = Quat_r.toRotationMatrix();

        Eigen::Vector3d cam_t_w;
        cam_t_w(0) = msg->pose.pose.position.x;
        cam_t_w(1) = msg->pose.pose.position.y;
        cam_t_w(2) = msg->pose.pose.position.z;
        Eigen::Matrix3d w_R_imu = cam_R_w.transpose() * imu_R_cam.transpose();
        Eigen::Vector3d w_t_imu = -cam_R_w.transpose() * (imu_R_cam.transpose() * imu_t_cam + cam_t_w);

        x.setZero();
        Eigen::Quaterniond w_Q_imu(w_R_imu);
        x.head<4>() << w_Q_imu.w() , w_Q_imu.vec();
        x.segment<3>(4) = w_t_imu;

        time_stamp = cur_time;
        if (flag_g_init)
        {
            ROS_WARN_STREAM("average g vector : " << g_init.transpose());
            G = w_R_imu * g_init;
            ROS_WARN_STREAM("gravity vector in world frame:" << G.transpose());
            //  G = G / G.norm() * 9.81;
            flag_odom_init = true;
        }
    }
    else
    {
        if(flag_g_init && flag_odom_init)
        {
            ///  新来的odom_msg的时间戳大于imu_msg的时间戳，需要扔掉之前的inu_msgs，进行简单的时间戳同步
            while(!imu_buffer.empty() && imu_buffer.front()->header.stamp < msg->header.stamp)
            {
                ROS_INFO("throw state with TIME : %f" , imu_buffer.front()->header.stamp.toSec());
                time_stamp  = imu_buffer.front()->header.stamp.toSec();

                //@TODO 这里是不是不应该扔IMU数据，应该通过IMU预积分？？？
                imu_buffer.pop();
                x_history.pop();
                P_history.pop();
            }
            //  时间戳同步之后，就该处理 x_history 中的数据了
            if(!x_history.empty())
            {
                x = x_history.front();
                P = P_history.front();
                time_stamp = imu_buffer.front()->header.stamp.toSec();
                imu_buffer.pop();
                x_history.pop();
                P_history.pop();
            }
            ROS_INFO("update state with TIME : %f" , msg->header.stamp.toSec());
            update(msg);
            while(!x_history.empty())
            {
                x_history.pop();
            }
            while(!P_history.empty())
            {
                P_history.pop();
            }
            std::queue< sensor_msgs::Imu::ConstPtr > new_imu_buffer;
            while (!imu_buffer.empty())
            {
                ROS_INFO("propagate state with Time : %f" , imu_buffer.front()->header.stamp.toSec());
                propagate(imu_buffer.front());
                new_imu_buffer.push(imu_buffer.front());
                imu_buffer.pop();
                x_history.pop();
                P_history.pop();
            }
            std::swap(imu_buffer , new_imu_buffer);

        }

    }
}

int main(int argc , char** argv)
{
    ros::init(argc , argv , "ekf_node");
    ros::NodeHandle n("~");
    ros::Subscriber sub_imu = n.subscribe("imu" , 100 , imu_callback);
    ros::Subscriber sub_odom = n.subscribe("tag_odom" , 100 , odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom" , 100);

    Rcam = Eigen::Quaterniond(0,0,-1,0).toRotationMatrix();

    ros::Rate r(100);   //  设置更新的频率
    Q.topLeftCorner(6,6)  = 0.01 * Q.topLeftCorner(6,6);                // Odom pose covariance
    Q.bottomRightCorner(6 , 6) =  0.0001 * Q.bottomRightCorner(6 , 6);  // Odom twist covariance

    Rt.topLeftCorner(3 ,3) *= 0.5;      //  Measurement orientation
    Rt.bottomRightCorner(3 , 3) *= 0.5; //  Measurement position
    Rt.bottomRightCorner(1 , 1) *= 0.5; //  Measurement position

    Eigen::Matrix3d R_tmp;
    R_tmp << -1, 0, 0,
              0, 1, 0,
              0, 0, -1;
    Eigen::Quaterniond Q_tmp(R_tmp);
    std::cout << "Q_tmp" << Q_tmp.w() << Q_tmp.vec().transpose() << std::endl;
    ROS_WARN("EKF time sychro version!");

    ros::spin();



    /*******    测试四元数和矩阵想乘   **********/
//    Eigen::Matrix3d vicon_R_tag;
//    vicon_R_tag << 0 , 1 , 0 , 1 , 0 , 0 , 0 , 0 , -1;
//    Eigen::Quaterniond q(1 , 0.9 , 0.4 , 0.8);
//    std::cout << q.coeffs()<< std::endl;
//    std::cout << vicon_R_tag * q << std::endl << std::endl;
//    std::cout << q * vicon_R_tag << std::endl;
///     test vector.segment<>();
//    Eigen::VectorXd v(16);
//    v << 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15;
//    std::cout << "v.segment<3>(4)" << std::endl;
//    std::cout << v.segment<3>(4) << std::endl;
//    std::cout << "v.segment<3>(7)" << std::endl;
//    std::cout << v.segment<3>(7) << std::endl;
//    std::cout << "v.segment<3>(10)" << std::endl;
//    std::cout << v.segment<3>(10) << std::endl;

///     test matrix.block<>();
//    Eigen::MatrixXd m(10 , 10);
//    m << 0,1,2,3,4,5,6,7,8,9,
//            0,1,2,3,4,5,6,7,8,9,
//            0,1,2,3,4,5,6,7,8,9,
//            0,1,2,3,4,5,6,7,8,9,
//            0,1,2,3,4,1,6,7,8,9,
//            0,1,2,3,4,5,6,7,8,9,
//            0,1,2,3,4,5,6,1,8,9,
//            0,1,2,3,4,5,6,7,8,9,
//            0,1,2,3,4,5,6,7,8,9,
//            0,1,2,3,4,5,6,7,8,9;
//    std::cout << m.block<3,3>(4,5) << std::endl;

}

//<remap from="~tag_odom" to="/nlw/odom_ref"/>
//<node pkg="tag_detector" type="tag_detector" name="tag_detector" output="log">
//<remap from="~iamge_raw" to="/nlw/image_raw">
//<param name="cam_cal_file" type="string" value="$(find ekf)/config/calibration.yaml" />
//<param name="board_config_file" type="string" value="$(find tag_detector)/config/tag_board.yaml"/>
//</node>
