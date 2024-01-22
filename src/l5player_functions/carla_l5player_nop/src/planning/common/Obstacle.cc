#include "Obstacle.h"
#include "path_matcher.h"

std::vector<Obstacle> AllObstacle;
std::mutex obstacle_mutex_;

// 在有效范围内的所有障碍物
PPoint front_left;
PPoint back_left;
PPoint back_right;
PPoint front_right;
// 求障碍物顶点
PPoint ob_left_front;
PPoint ob_left_buttom;
PPoint ob_right_front;
PPoint ob_right_buttom;

std::pair<double, double> ego_pos;    // 自主车的实时位置
double ego_head;                      // 自主车的朝向

Obstacle::Obstacle(bool sub_obstacle) {
}



bool Obstacle::IsStatic() const { return obstacle_velocity < 0.2; }
bool Obstacle::IsVirtual() const {
    return false;    // 假设都不是虚拟的
}

bool Obstacle::HasTrajectory() const {
    return !(trajectory_.trajectory_point_size() == 0);    // 没有预测轨迹就是静态障碍物
}


Box2d Obstacle::GetBoundingBox(const TrajectoryPoint &point) const {
    return Box2d({point.x, point.y}, point.theta, obstacle_length, obstacle_width);
}

// 获得障碍物在当前时刻的TrajectoryPoint,为了求GetBoundingBox
TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const {
    const auto &points = trajectory_.trajectory_point();
    // std::cout<<"points.size():"<<points.size()<<"\n";
    if (points.size() < 2)    // 认为是静态障碍物
    {
        TrajectoryPoint point;
        point.set_x(centerpoint.position.x);
        point.set_y(centerpoint.position.y);
        point.set_z(0);
        point.set_theta(obstacle_theta);
        point.set_s(0.0);
        point.set_kappa(0.0);
        point.set_dkappa(0.0);
        point.set_v(0.0);
        point.set_a(0.0);
        point.set_relative_time(0.0);
        return point;
    } else    // 认为是一个运动的障碍物
    {
        // for (size_t i = 0; i < points.size(); i++)
        // {
        // std::cout << "*relative_time2:" << points[i].relative_time << "\n";
        // }
        // std::cout << "--------------------------"
        //           << "\n";
        if (relative_time >= points.back().relative_time) {
            return points.back();
        }

        auto comp = [](const TrajectoryPoint p, const double time) { return p.relative_time < time; };

        auto it_lower = std::lower_bound(points.begin(), points.end(), relative_time, comp);
        if (it_lower == points.begin()) {
            return points.front();
        } else if (it_lower == points.end()) {
            return points.back();
        }

        return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, relative_time);
    }
}

// 已知载具（矩形）的中心点坐标、长、宽和倾斜角度，求载具（矩形）四个边界点
//************************************
//  Method:    GenerateCarBoundaryPoint
//  FullName:  GenerateCarBoundaryPoint
//  Access:    public
//  Returns:   void
//  Qualifier:根据载具中心坐标生成载具四个边界点的坐标
//  Parameter: double car_length		载具长
//  Parameter: double car_width		载具宽
//  Parameter: PPoint & center 		中心点
//  Parameter: double angle			倾斜角度 非弧度角 [0,360) x轴正方向为0度
//  Parameter: PPoint & front_left	左前点
//  Parameter: PPoint & back_left	左后点
//  Parameter: PPoint & back_right	右后点
//  Parameter: PPoint & front_right	右前点
//************************************
void Obstacle::CalculateCarBoundaryPoint(double car_length, double car_width, PPoint &center, double angle,
                                               PPoint &front_left, PPoint &back_left, PPoint &back_right,
                                               PPoint &front_right) {
    // 角度为负值，错误输入，返回
    if (angle < 0) return;
    double X1, Y1, X2, Y2, X3, Y3, X4, Y4;
    // 以(x,y)为中心点，不旋转的情况下四个顶点的坐标
    back_right.x = (center.x - car_length / 2);
    back_right.y = (center.y - car_width / 2);
    back_left.x = (center.x - car_length / 2);
    back_left.y = (center.y + car_width / 2);
    front_left.x = (center.x + car_length / 2);
    front_left.y = (center.y + car_width / 2);
    front_right.x = (center.x + car_length / 2);
    front_right.y = (center.y - car_width / 2);
    if (angle <= 0.00001)
        return;
    else {
        // 按逆时针旋转角度center.x后的四个点坐标
        X1 = (back_right.x - center.x) * cos(angle) - (back_right.y - center.y) * sin(angle) + center.x;
        Y1 = (back_right.y - center.y) * cos(angle) + (back_right.x - center.x) * sin(angle) + center.y;
        X2 = (back_left.x - center.x) * cos(angle) - (back_left.y - center.y) * sin(angle) + center.x;
        Y2 = (back_left.y - center.y) * cos(angle) + (back_left.x - center.x) * sin(angle) + center.y;
        X3 = (front_left.x - center.x) * cos(angle) - (front_left.y - center.y) * sin(angle) + center.x;
        Y3 = (front_left.y - center.y) * cos(angle) + (front_left.x - center.x) * sin(angle) + center.y;
        X4 = (front_right.x - center.x) * cos(angle) - (front_right.y - center.y) * sin(angle) + center.x;
        Y4 = (front_right.y - center.y) * cos(angle) + (front_right.x - center.x) * sin(angle) + center.y;
        back_right.x = X1;
        back_right.y = Y1;
        back_left.x = X2;
        back_left.y = Y2;
        front_left.x = X3;
        front_left.y = Y3;
        front_right.x = X4;
        front_right.y = Y4;
    }
}

double Obstacle::getCross(PPoint p1, PPoint p2, PPoint p) {
    return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

bool Obstacle::Inside_rectangle(PPoint p1, PPoint p2, PPoint p3, PPoint p4, PPoint p) {
    if (getCross(p1, p2, p) * getCross(p3, p4, p) >= 0 && getCross(p2, p3, p) * getCross(p4, p1, p) >= 0) {
        return true;
    }
    return false;
}

// 产生pre_timepre_time秒的预测
Ob_Trajectory Obstacle::Generater_Trajectory(geometry_msgs::msg::Pose ob_pos, double pre_time,
                                                               double obstacle_theta, double obstacle_velocity) {
    Ob_Trajectory result;
    std::vector<TrajectoryPoint> Trajectories;
    Eigen::MatrixXd ob_points;
    ob_points = Eigen::MatrixXd::Zero(2, 3);    // 初始化零矩阵
    ob_points(0, 0) = ob_pos.position.x;
    ob_points(0, 1) = ob_pos.position.y;
    ob_points(0, 2) = 0;

    ob_points(1, 0) = ob_pos.position.x + pre_time * cos(obstacle_theta);
    ob_points(1, 1) = ob_pos.position.y + pre_time * sin(obstacle_theta);
    ob_points(1, 2) = 0;

    Eigen::MatrixXd path_point_after_interpolation;
    average_interpolation(ob_points, path_point_after_interpolation, pre_time);    // 线性插值

    double s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    double relative_time = 0.0;
    bool empty_path = true;
    std::vector<double> headings;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::vector<double> accumulated_s;
    std::vector<std::pair<double, double>> xy_points;

    for (size_t i = 0; i < path_point_after_interpolation.rows(); i++) {
        xy_points.emplace_back(path_point_after_interpolation(i, 0), path_point_after_interpolation(i, 1));
    }

    if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
        RCLCPP_WARN(rclcpp::get_logger("Obstacle_avoid"), "obstacle prediction trajectory generate failed!");
    }
    // std::cout << "path_point_after_interpolation.rows():" << path_point_after_interpolation.rows() << "\n";

    for (int i = 0; i < path_point_after_interpolation.rows(); i++)    // 遍历每个预测轨迹点
    {
        TrajectoryPoint tra_;
        tra_.set_x(path_point_after_interpolation(i, 0));    // x
        tra_.set_y(path_point_after_interpolation(i, 1));    // y
        tra_.set_theta(headings[i]);
        tra_.set_v(obstacle_velocity);    // 假设未来匀速
        tra_.set_a(0);                    // 匀速，那么a=0
        tra_.set_kappa(kappas[i]);
        tra_.set_dkappa(dkappas[i]);
        tra_.set_s(accumulated_s[i]);

        tra_.set_relative_time(relative_time);

        Trajectories.push_back(tra_);

        relative_time += Config_.FLAGS_trajectory_time_resolution;
    }

    result.Set_Trajectory(Trajectories);
    return result;
}

void Obstacle::average_interpolation(Eigen::MatrixXd &input, Eigen::MatrixXd &output, double pre_time) {
    // 1.定义
    std::vector<Point3d_s> vec_3d;
    std::vector<Point3d_s> n_vec;
    Point3d_s p;
    const int INSERTPOINTNUMBER = pre_time / Config_.FLAGS_trajectory_time_resolution;

    // 第一个点
    double start_x = input(0, 0);
    double start_y = input(0, 1);
    // 最后一个点
    double end_x = input(1, 0);
    double end_y = input(1, 1);

    // 在两点间插入INSERTPOINTNUMBER个点
    for (int i = 0; i < INSERTPOINTNUMBER; ++i) {
        // 计算(x,y)两点的距离
        double sqrt_val = sqrt((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
        // 计算角度
        double sin_a = (end_y - start_y) / sqrt_val;
        double cos_a = (end_x - start_x) / sqrt_val;

        p.x = (start_x * (INSERTPOINTNUMBER - i) + end_x * i) / float(INSERTPOINTNUMBER);
        p.y = (start_y * (INSERTPOINTNUMBER - i) + end_y * i) / float(INSERTPOINTNUMBER);
        p.z = 0;
        vec_3d.push_back(p);
    }

    // 4.漏了终点，需要加上
    p.x = input(input.rows() - 1, 0);
    p.y = input(input.rows() - 1, 1);
    p.z = input(input.rows() - 1, 2);
    vec_3d.push_back(p);

    // 传给输出矩阵output
    output = Eigen::MatrixXd::Zero(vec_3d.size(), 3);
    int j = 0;
    for (std::vector<Point3d_s>::iterator it = vec_3d.begin(); it != vec_3d.end(); it++) {
        output(j, 0) = (*it).x;
        output(j, 1) = (*it).y;
        output(j, 2) = (*it).z;
        j++;
    }
}