#ifndef SR_UKF_
#define SR_UKF_

#define EIGEN_NO_DEBUG

#define N 3
#define INPUTS 2
#define ROWS 2

#include <Eigen/Dense>
#include "merwe_scaled_sigma_generator.h"
#include "spherical_conversions.h"



class sr_ukf {
    public:
    sr_ukf(const Eigen::Vector<double, N>& initial_state, const Eigen::Vector<double, N>& initial_stddevs, const Eigen::Vector<double, N>& process_stddevs, const Eigen::Vector<double, ROWS>& measurement_stddevs);

    void predict(const Eigen::Vector<double, INPUTS>& u, const double& dt);
    void update(const Eigen::Vector<double, ROWS>& u, const double& dt);

    void set_contR(const Eigen::Vector<double, ROWS>& new_R);
    void set_xhat(const Eigen::Vector<double, N>& new_xhat);

    Eigen::Vector<double, N>& get_xhat();
    Eigen::Matrix<double, N, N>& get_s();

    Eigen::Vector<double, ROWS> h_lidar(Eigen::Vector<double, N> x, double beam_angle);

    private:
    Eigen::Vector<double, N> m_xhat;
    Eigen::Matrix<double, N, N> m_S;
    Eigen::Matrix<double, N, N> m_contQ;
    Eigen::Matrix<double, ROWS, ROWS> m_contR;
    Eigen::Matrix<double, N, 2 * N + 1> m_sigmasf;
    merwe_scaled_sigma_generator m_pts;
    c_point m_reference_pose;
    
    Eigen::Vector<double, N> process_model(const Eigen::Vector<double, N>& x, const Eigen::Vector<double, INPUTS>& u, const double& dt);
    
    double normalize_angle180(double x);
    double normalize_angle360(double x);

    Eigen::Vector<double, ROWS> measurement_mean(const Eigen::Matrix<double, ROWS, 2 * N + 1>& sigmas, const Eigen::Vector<double, 2 * N + 1>& wm);
    Eigen::Vector<double, ROWS> measurement_residual(const Eigen::Vector<double, ROWS>& a, const Eigen::Vector<double, ROWS>& b);
    Eigen::Vector<double, N> state_mean(const Eigen::Matrix<double, N, 2 * N + 1>& sigmas, const Eigen::Vector<double, 2 * N + 1>& wm);
    Eigen::Vector<double, N> state_residual(const Eigen::Vector<double, N>& a, const Eigen::Vector<double, N>& b);
    Eigen::Vector<double, N> state_add(const Eigen::Vector<double, N>& a, const Eigen::Vector<double, N>& b);
};

#endif
