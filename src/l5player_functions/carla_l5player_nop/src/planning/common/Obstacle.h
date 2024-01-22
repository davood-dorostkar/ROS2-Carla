#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "l5player_nop_msgs/msg/obstacle.hpp"
#include "math_utils.h"
#include "boundarys.h"

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <cmath>
// #include <tf/tf.h>
#include <tf2/utils.h>
#include <boost/thread.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <eigen3/Eigen/Dense>

// #include "CubicSpline2D.h"
#include "trajectoryPoint.h"
#include "Ob_prediction_trajectory.h"

using namespace Eigen;

// 坐标点
struct PPoint {
    double x;    // 坐标点x坐标
    double y;    // 坐标点y坐标
    PPoint(double _x, double _y) : x(_x), y(_y){};
    PPoint() : x(0), y(0){};
    PPoint(const PPoint &other) : x(other.x), y(other.y){};
    const PPoint operator=(const PPoint &p) {
        x = p.x;
        y = p.y;
        return *this;
    }
    bool operator==(const PPoint &p) { return (abs(x - p.x) < 0.0001 && abs(y - p.y) < 0.0001); }
};


// 障碍物的类，无非就是id、长宽高(可不用，换成4个顶点位置)、中心位置以及朝向
class Obstacle {
    struct Point3d_s {
        double x;
        double y;
        double z;
    };

   public:
    Obstacle() = default;
    Obstacle(bool sub_obstacle);
    ~Obstacle() = default;

    bool IsStatic() const;     // use
    bool IsVirtual() const;    // use

    bool HasTrajectory() const;                                  // use
    Box2d GetBoundingBox(const TrajectoryPoint &point) const;    // use

    TrajectoryPoint GetPointAtTime(const double relative_time) const;    // use

    const Ob_Trajectory &Trajectory() const { return trajectory_; }    // use

    void SetTrajectory(const Ob_Trajectory trajectory) { trajectory_ = trajectory; }    // use

    void CalculateCarBoundaryPoint(double car_length, double car_width, PPoint &center, double angle,
                                   PPoint &front_left, PPoint &back_left, PPoint &back_right, PPoint &front_right);

    double getCross(PPoint p1, PPoint p2, PPoint p);
    bool Inside_rectangle(PPoint p1, PPoint p2, PPoint p3, PPoint p4, PPoint p);

    // 用此函数模拟动态障碍物的预测轨迹，即：前探距离=当前位置*预测距离，然后线性插值，来得到预测轨迹点
    Ob_Trajectory Generater_Trajectory(geometry_msgs::msg::Pose ob_pos, double pre_time,
                                                   double obstacle_theta, double obstacle_velocity);
    void average_interpolation(Eigen::MatrixXd &input, Eigen::MatrixXd &output, double pre_time);

   private:
    // SL_Boundary sl_boundary_;
    Ob_Trajectory trajectory_;    // 动态障碍物的预测轨迹
    common::math::Polygon2d perception_polygon_;
    Box2d perception_bounding_box_;

    double eff_length;
    double eff_width;
    double eff_dis;
    std::vector<std::pair<double, double>> pinnacles;

   public:
    // 障碍物的id
    std::string obstacle_id;
    // 障碍物的类型
    /*
      uint8 UNKNOWN=0
      uint8 CAR=1
      uint8 TRUCK=2
      uint8 BUS=3
      uint8 BICYCLE=4
      uint8 MOTORBIKE=5
      uint8 PEDESTRIAN=6
      uint8 ANIMAL=7
    */
    int obstacle_type;
    // 障碍物的形状
    /*
      uint8 BOUNDING_BOX=0
      uint8 CYLINDER=1
      uint8 POLYGON=2
    */
    int obstacle_shape;
    // 障碍物4个顶点位置, 两种表达方式
    geometry_msgs::msg::PoseArray pinnacle;
    std::vector<Vec2d> polygon_points;

    // 中心点位置
    geometry_msgs::msg::Pose centerpoint;
    // 障碍物的速度
    double obstacle_velocity;
    // 障碍物朝向
    double obstacle_theta;

    // 扣一圈的半径大小，适用于frenet下的圆形和方形
    double obstacle_radius;

    // 障碍物的长宽高
    double obstacle_length;
    double obstacle_width;
    double obstacle_height;

    // 时间戳
    rclcpp::Time timestamp_;
};

extern std::vector<Obstacle> AllObstacle;
extern std::mutex obstacle_mutex_;

#endif    // OBSTACLE_H