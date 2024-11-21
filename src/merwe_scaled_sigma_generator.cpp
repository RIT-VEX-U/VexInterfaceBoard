#include <Eigen/Dense>
#include "merwe_scaled_sigma_generator.h"

Eigen::Vector<double, 2 * N + 1>& merwe_scaled_sigma_generator::wm() {return m_wm;}
Eigen::Vector<double, 2 * N + 1>& merwe_scaled_sigma_generator::wc() {return m_wc;}

merwe_scaled_sigma_generator::merwe_scaled_sigma_generator()
{
    m_alpha = 1e-2;
    m_kappa = 0;
    double beta = 2;

    m_wm.resize(2 * N + 1);
    m_wc.resize(2 * N + 1);

    compute_weights(beta);
}

Eigen::Matrix<double, N, 2 * N + 1> merwe_scaled_sigma_generator::square_root_sigma_points(
    const Eigen::Vector<double, N>& x,
    const Eigen::Matrix<double, N, N>& S)
{
    const double lambda = (m_alpha * m_alpha) * (N + m_kappa) - N;
    double eta = std::sqrt(lambda + N);
    Eigen::Matrix<double, N, N> U;
    U = eta * S;

    Eigen::Matrix<double, N, 2 * N + 1> sigmas;
    sigmas.col(0) = x;
    for (int k = 0; k < N; k++)
    {
        sigmas.col(k + 1) = x + U.col(k);
        sigmas.col(N + k + 1) = x - U.col(k);
    }

    return sigmas;
}

void merwe_scaled_sigma_generator::compute_weights(double beta)
{
    const double lambda = (m_alpha * m_alpha) * (N + m_kappa) - N;
    const double c = 0.5 / (N + lambda);

    m_wm.fill(c);
    m_wc.fill(c);
    m_wc[0] = lambda / (N + lambda) + (1 - (m_alpha * m_alpha) + beta);
    m_wm[0] = lambda / (N + lambda);
}
