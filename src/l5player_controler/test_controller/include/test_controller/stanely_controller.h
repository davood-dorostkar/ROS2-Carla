#pragma once
#include <math.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "common.h"
#include "test_controller/rateLimiter.h"

namespace l5player
{
    namespace control
    {
        using Matrix = Eigen::MatrixXd;

        class StanleyController
        {
        private:
            double angleCommandMaxChangeRate_ = 0.1;
            RateLimiter limiter_;

        public:
            StanleyController() : limiter_(0.1){};
            ~StanleyController(){};

            void LoadControlConf();
            void ComputeControlCmd(const VehicleState &vehicle_state, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd);
            void ComputeLateralErrors(const double x, const double y, const double theta, double &e_y, double &e_theta);
            TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);

        protected:
            std::vector<TrajectoryPoint> trajectory_points_;
            double k_y_ = 0.0;
            double u_min_ = 0.0;
            double u_max_ = 100.0;

            double theta_ref_;
            double theta_0_;
        };
    } // namespace control
} // namespace l5player
