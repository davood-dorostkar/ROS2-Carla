#include "reference_line.h"

#include <iostream>
#include <vector>


namespace l5player {
namespace reference_line {

referenceLine::referenceLine() { smoother_ = std::make_unique<DiscretePointsReferenceLineSmoother>(); }

void referenceLine::process(Eigen::MatrixXd& hdmap_way_points) {
    std::cout << "referenceLine_runonce process!!!" << std::endl;
    std::cout << "guard process in!!!" << std::endl;
    std::lock_guard<std::mutex> guard(mutex_);
    routing_waypoints_ = Eigen::MatrixXd::Zero(hdmap_way_points.rows(), 3);
    for (Eigen::Index i = 0; i < hdmap_way_points.rows(); ++i) {
        routing_waypoints_(i, 0) = hdmap_way_points(i, 0);
        routing_waypoints_(i, 1) = hdmap_way_points(i, 1);
        routing_waypoints_(i, 2) = (double)0.0;
    }

    average_interpolation(routing_waypoints_, path_point_after_interpolation_, 0.2, 0.6);
    // Smooth(path_point_after_interpolation_);
    // Smooth(routing_waypoints_);
    Smooth();

    referencePointsCalc(referenceline_);    // 计算参考点的kappa、theta，确保只计算一次，减少重复计算

    std::cout << "reference_points.size: " << reference_points.size();
    std::cout << "guard process out!!!" << std::endl;

}

void referenceLine::referenceLine_split(Eigen::MatrixXd& hdmap_way_points) {
    average_interpolation(hdmap_way_points, path_point_after_interpolation_, 0.2, 0.6);
    // Smooth(path_point_after_interpolation_);
    Smooth();
}

void referenceLine::average_interpolation(Eigen::MatrixXd& input, Eigen::MatrixXd& output, double interval_dis,
                                          double distance)    // 0.2 0.6
{
    // 1.定义一个容器，类型为Point3d_s,即（x,y,z）
    std::vector<Point3d_s> vec_3d;
    std::vector<Point3d_s> n_vec;
    Point3d_s p;
    // 2.遍历
    // std::cout << " input.rows()" << input.rows() << std::endl;
    for (int i = 0; i < input.rows() - 1; i++) {
        double dis = (input.row(i + 1) - input.row(i)).norm();    // 求两点的距离，前一行和这一行坐标的距离
        // std::cout << "dis " << dis << std::endl;
        // 两点距离太长的话就进行插点
        if (dis >= distance) {
            // 计算(x,y)两点的距离
            double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
                                   (input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
            // 计算角度
            double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
            double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
            // 两点之间要插值的插值点的数量
            int num = dis / interval_dis;    // 分割了一下
            // std::cout << "num " << num << std::endl;
            // 插入点
            for (int j = 0; j < num; j++) {
                // i=0,j=0的时候其实是插入起点
                p.x = input(i, 0) + j * interval_dis * cos_a;
                p.y = input(i, 1) + j * interval_dis * sin_a;
                p.z = input(i, 2);
                vec_3d.push_back(p);
            }
        }
        // 3.有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
        else if (dis < distance) {
            p.x = input(i, 0);
            p.y = input(i, 1);
            p.z = input(i, 2);
            vec_3d.push_back(p);
        }
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

void referenceLine::Smooth() {
    std::vector<AnchorPoint> anchor_points;

    for (Eigen::Index i = 0; i < path_point_after_interpolation_.rows(); ++i) {
        AnchorPoint anchor_point;
        anchor_point.path_point.x_ = path_point_after_interpolation_(i, 0);
        anchor_point.path_point.y_ = path_point_after_interpolation_(i, 1);
        anchor_point.lateral_bound = 1.0;
        anchor_points.emplace_back(anchor_point);
    }

    smoother_->SetAnchorPoints(anchor_points);
    smoother_->Smooth(referenceline_);
}

void referenceLine::referencePointsCalc(const nav_msgs::msg::Path& path_point) {
    // 计算参考点的kappa、theta，只需要计算一次
    if (path_point.poses.size() > 0 && path_point_flag_.size() < 1) {
        reference_path.first.clear();
        reference_path.second.clear();
        accumulated_s.clear();
        reference_points.clear();

        path_point_flag_.push_back(path_point.poses[0].pose.position.x);
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<std::pair<double, double>> xy_points;

        // auto beforeTime = std::chrono::steady_clock::now(); //计时开始
        for (size_t i = 0; i < path_point.poses.size(); ++i) {
            reference_path.first.push_back(path_point.poses[i].pose.position.x);
            reference_path.second.push_back(path_point.poses[i].pose.position.y);
            xy_points.emplace_back(path_point.poses[i].pose.position.x, path_point.poses[i].pose.position.y);
        }

        if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
            // ROS_WARN("rerferenceline generate failed!");
            std::cout << "rerferenceline generate failed!" << std::endl;
        } else {
            for (size_t i = 0; i < xy_points.size(); ++i) {
                // 创建ReferencePoint类
                ReferencePoint reference_point(kappas[i], dkappas[i], xy_points[i].first, xy_points[i].second,
                                               headings[i], accumulated_s[i]);
                reference_points.emplace_back(reference_point);
            }
        }
    }
}

std::vector<TrajectoryPoint> referenceLine::plan_start_point(double& current_time) {
    if (this->is_first_run) {
        // 第一次运行
        std::cout << "第1次运行" << std::endl;
        this->is_first_run = false;
        double x_init = gps_.posx;    // 定位
        double y_init = gps_.posy;
        double z_init = 0;
        double v_init = 0;
        double a_init = 0;
        double theta_init = gps_.oriz;
        double kappa_init = 0;
        double dkappa_init = 0;
        double s0 = 0;
        this->init_relative_time = 0.0;
        plan_start_time = current_time + 0.1;    // start point absolute time

        TrajectoryPoint init_point;
        init_point.set_s(0.0);
        init_point.set_x(x_init);
        init_point.set_y(y_init);
        init_point.set_z(z_init);
        init_point.set_theta(theta_init);
        init_point.set_kappa(kappa_init);
        init_point.set_v(v_init);
        init_point.set_a(a_init);
        init_point.set_relative_time(0.0);
        return std::vector<TrajectoryPoint>(1, init_point);
    } else {
        // 非第一次运行
        std::cout << "非第一次运行且有历史轨迹" << std::endl;
        double x_cur = gps_.posx;    // 定位
        double y_cur = gps_.posy;
        double theta_cur = gps_.oriz;
        double kappa_cur = 0;
        double vx_cur = gps_.velx;
        double vy_cur = gps_.vely;
        double ax_cur = gps_.accelx;
        double ay_cur = gps_.accely;
        double dt = 0.1;
        int index = 0;

        for (int i = 0; i < pre_trajectory_.size() - 1; ++i) {
            if (pre_trajectory_[i].absolute_time <= current_time &&
                pre_trajectory_[i + 1].absolute_time > current_time) {
                index = i;
                break;
            }
        }
        // 上一周期规划的本周期车辆应该在的位置
        double pre_x_desire = pre_trajectory_[index].x;
        double pre_y_desire = pre_trajectory_[index].y;
        double pre_theta_desire = pre_trajectory_[index].theta;
        // 计算横纵向误差
        Eigen::Matrix<double, 2, 1> tor;
        tor << cos(pre_theta_desire), sin(pre_theta_desire);
        Eigen::Matrix<double, 2, 1> nor;
        nor << -sin(pre_theta_desire), cos(pre_theta_desire);
        // 误差向量
        Eigen::Matrix<double, 2, 1> d_err;
        d_err << x_cur - pre_x_desire, y_cur - pre_y_desire;
        double lon_err = abs(d_err.transpose() * tor);
        double lat_err = abs(d_err.transpose() * nor);

        std::size_t position_matched_index = pre_trajectory_.QueryNearestPoint({x_cur, y_cur});
        std::cout << "pre_trajectory_ size : " << pre_trajectory_.size() << std::endl;
        std::cout << "poisition match point index: " << position_matched_index << std::endl;
        std::cout << "time match point index: " << index << std::endl;
        printf("pre_x_desire = %f m\n", pre_x_desire);    // 局部路径
        printf("pre_y_desire = %f m\n", pre_y_desire);
        printf("lon_err = %f m\n", lon_err);
        printf("lat_err = %f m\n", lat_err);
        // 如果纵向误差大于2.5或者横向误差大于0.5认为控制没跟上
        if (lon_err > 2.5 || lat_err > 0.5) {
            // ROS_INFO("out of control, use dynamic to deduce");
            std::cout << "ax_cur = " << ax_cur << " ay_cur = " << ay_cur << std::endl;
            double x_init = x_cur + vx_cur * dt + 0.5 * ax_cur * dt * dt;
            double y_init = y_cur + vy_cur * dt + 0.5 * ay_cur * dt * dt;
            double z_init = 0;
            double v_init = std::sqrt(std::pow(vy_cur + ay_cur * dt, 2) + std::pow(vx_cur + ax_cur * dt, 2));
            double a_init = std::sqrt(ax_cur * ax_cur + ay_cur * ay_cur);
            double theta_init = atan2(vy_cur + ay_cur * dt, vx_cur + ax_cur * dt);
            double kappa_init = kappa_cur;
            double dkappa_init = 0;
            double s0 = 0;
            this->init_relative_time = 0.0;

            plan_start_time = current_time + 0.1;

            TrajectoryPoint init_point;
            init_point.set_s(0.0);
            init_point.set_x(x_init);
            init_point.set_y(y_init);
            init_point.set_z(z_init);
            init_point.set_theta(theta_init);
            init_point.set_kappa(kappa_init);
            init_point.set_v(v_init);
            init_point.set_a(a_init);
            init_point.set_relative_time(0.0);
            return std::vector<TrajectoryPoint>(1, init_point);
        } else {
            // ROS_INFO("good control");
            int index_good = 0;
            for (int i = index; i < pre_trajectory_.size() - 1; ++i) {
                if (pre_trajectory_[i].absolute_time <= current_time + 0.1 &&
                    pre_trajectory_[i + 1].absolute_time > current_time + 0.1) {
                    index_good = i;
                    std::cout << "forward time match point index:: " << i << std::endl;
                    break;
                }
            }
            double x_init = pre_trajectory_[index_good].x;
            double y_init = pre_trajectory_[index_good].y;
            double z_init = 0;
            double v_init = pre_trajectory_[index_good].v;
            double a_init = pre_trajectory_[index_good].a;
            double theta_init = pre_trajectory_[index_good].theta;
            double kappa_init = pre_trajectory_[index_good].kappa;
            double dkappa_init = pre_trajectory_[index_good].dkappa;
            // double s0 = pre_trajectory_[index_good].s;
            // double ds0 = pre_trajectory_[index_good].s_d;
            // double dd0 = pre_trajectory_[index_good].d_d;
            // double d0 = pre_trajectory_[index_good].d;
            // double dds0 = pre_trajectory_[index_good].s_dd;
            // double ddd0 = pre_trajectory_[index_good].d_dd;
            plan_start_time = pre_trajectory_[index_good].absolute_time;

            TrajectoryPoint init_point;
            init_point.set_s(0.0);
            init_point.set_x(x_init);
            init_point.set_y(y_init);
            init_point.set_z(z_init);
            init_point.set_theta(theta_init);
            init_point.set_kappa(kappa_init);
            init_point.set_v(v_init);
            init_point.set_a(a_init);
            init_point.set_relative_time(0.0);
            return std::vector<TrajectoryPoint>(1, init_point);
        }
    }
}

bool referenceLine::is_update_dynamic(nav_msgs::msg::Path& trj_point_array, int size) {
    bool is_update;
    is_update = false;
    // 车的当前位置
    double pp_x = gps_.posx;
    double pp_y = gps_.posy;
    double xx = trj_point_array.poses[size].pose.position.x;
    double yy = trj_point_array.poses[size].pose.position.y;
    double distance = sqrt(pow(pp_x - xx, 2) + pow(pp_y - yy, 2));
    if (distance < 2)    // 接近了
    {
        is_update = true;
    }
    return is_update;
}

}    // namespace reference_line
}    // namespace l5player
