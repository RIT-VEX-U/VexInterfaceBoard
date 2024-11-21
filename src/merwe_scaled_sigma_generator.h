#ifndef MERWE_SCALED_SIGMA_GENERATOR_
#define MERWE_SCALED_SIGMA_GENERATOR_

#define EIGEN_NO_DEBUG

#define N 3

#include <Eigen/Dense>

class merwe_scaled_sigma_generator
{
public:
    merwe_scaled_sigma_generator();
    Eigen::Vector<double, 2 * N + 1>& wm();
    Eigen::Vector<double, 2 * N + 1>& wc();
    Eigen::Matrix<double, N, 2 * N + 1> square_root_sigma_points(const Eigen::Vector<double, N> &x, const Eigen::Matrix<double, N, N> &S);

private:
    Eigen::Vector<double, 2 * N + 1> m_wm;
    Eigen::Vector<double, 2 * N + 1> m_wc;
    double m_alpha;
    double m_kappa;

    void compute_weights(double beta);
};

#endif
    