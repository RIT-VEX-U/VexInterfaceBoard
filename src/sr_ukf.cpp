#include <Eigen/Dense>
#include "sr_ukf.h"
#include "spherical_conversions.h"

sr_ukf::sr_ukf(const Eigen::Vector<double, N>& initial_state, const Eigen::Vector<double, N>& initial_stddevs, const Eigen::Vector<double, N>& process_stddevs, const Eigen::Vector<double, ROWS>& measurement_stddevs) {
    m_xhat = initial_state;

    // these are intended to be the square root of what they would usually be
    m_contQ = process_stddevs.asDiagonal();
    m_contR = measurement_stddevs.asDiagonal();
    m_S = initial_stddevs.asDiagonal();    

}

void sr_ukf::predict(const Eigen::Vector<double, INPUTS>& u, const double& dt) {
    // discretize the noise model
    Eigen::Matrix<double, N, N> disc_q = m_contQ * dt;
    // for (int i = 0; i < N; i++) {
    //     disc_q(i, i) = sqrt(disc_q(i, i));
    // }

    // generate sigma points
    Eigen::Matrix<double, N, 2 * N + 1> sigmas = m_pts.square_root_sigma_points(m_xhat, m_S);

    // project sigmas through process model (merwe 18)
    for (int i = 0; i < (2 * N + 1); i++) {
        m_sigmasf.col(i) = (process_model(sigmas.col(i), u, dt));
    }

    // xhat is mean of sigmas (merwe 19)
    m_xhat = state_mean(m_sigmasf, m_pts.wm());

    // qr of matrix of weighted sigmas, and sqrt(noise cov) (merwe 20)
    Eigen::Vector<double, 2 * N + 1> wc = m_pts.wc();
    Eigen::Matrix<double, N, N * 2 + N> sbar;
    for (int i = 0; i < N * 2; i++) {
        sbar.col(i) = std::sqrt(wc(1)) * state_residual(m_sigmasf.col(1 + i), m_xhat);
    }
    sbar.block<N, N>(0, N * 2) = disc_q.triangularView<Eigen::Lower>();
    Eigen::Matrix<double, N, N> S = sbar.transpose().householderQr().matrixQR().block<N, N>(0, 0).triangularView<Eigen::Upper>();

    // cholupdate (merwe 21)
    Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(S, state_residual(m_sigmasf.col(0), m_xhat), wc(0));
    m_S = S;
}

void sr_ukf::update(const Eigen::Vector<double, ROWS>& y, const double& dt) {
    // discretize the noise model
    Eigen::Matrix<double, ROWS, ROWS> disc_r = m_contR / dt;
    Eigen::internal::llt_inplace<double, Eigen::Lower>::blocked(disc_r);

    // project sigmas through measurement function, they become predicted measurements at their state (merwe 22)
    Eigen::Matrix<double, ROWS, 2 * N + 1> h_sigmas;
    Eigen::Matrix<double, N, 2 * N + 1> sigmas;
    sigmas = m_pts.square_root_sigma_points(m_xhat, m_S);

    for (int i = 0; i < (2 * N + 1); i++) {
        h_sigmas.col(i) = h_lidar(sigmas.col(i), y(1));
    }

    // yhat is mean of measurement sigmas (merwe 23)
    Eigen::Vector<double, ROWS> yhat = measurement_mean(h_sigmas, m_pts.wm());

    // qr of matrix of weighted measurement sigmas, and sqrt(noise cov) (merwe 24)
    Eigen::Vector<double, 2 * N + 1> wc = m_pts.wc();
    Eigen::Matrix<double, ROWS, N * 2 + ROWS> sbar;
    for (int i = 0; i < N * 2; i++) {
        sbar.col(i) = std::sqrt(wc(1)) * measurement_residual(h_sigmas.col(1 + i), yhat);
    }
    sbar.block<ROWS, ROWS>(0, N * 2) = disc_r.triangularView<Eigen::Lower>();
    Eigen::Matrix<double, ROWS, ROWS> sy = sbar.transpose().householderQr().matrixQR().block<ROWS, ROWS>(0, 0).triangularView<Eigen::Upper>();

    // cholupdate (merwe 25)
    Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(sy, measurement_residual(h_sigmas.col(0), yhat), wc(0));

    // pxy (merwe 26)
    Eigen::Matrix<double, N, ROWS> pxy;
    pxy.Zero();
    for (int i = 0; i < (2 * N + 1); i++) {
        pxy.noalias() += wc(i) * (state_residual(sigmas.col(i), m_xhat) * measurement_residual(h_sigmas.col(i), yhat).transpose());
    }

    // calculate kalman gain with two least squares solutions (merwe 26)
    Eigen::Matrix<double, N, ROWS> K = sy.transpose().fullPivHouseholderQr().solve(sy.fullPivHouseholderQr().solve(pxy.transpose())).transpose();
    
    // update mean using kalman gain and measurement residual (merwe 27)
    m_xhat = state_add(m_xhat, K * measurement_residual(y, yhat));

    // intermediate matrix to find new covariance (merwe 28)
    Eigen::Matrix<double, N, ROWS> U = K * sy;

    // cholupdate covariance (merwe 29)
    for (int i = 0; i < ROWS; i++) {
        Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(m_S, U.col(i), -1);
    }

    // std::cout << std::endl << std::endl << sigmas << std::endl << std::endl << h_sigmas << std::endl << std::endl << yhat << std::endl << std::endl;
}

void sr_ukf::set_contR(const Eigen::Vector<double, ROWS>& new_R) {
    m_contR = new_R.asDiagonal();
}

void sr_ukf::set_xhat(const Eigen::Vector<double, N>& new_xhat) {
    m_xhat = new_xhat;
}

Eigen::Vector<double, N>& sr_ukf::get_xhat() {
    return m_xhat;
}

Eigen::Matrix<double, N, N>& sr_ukf::get_s() {
    return m_S;
}

Eigen::Vector<double, 2> sr_ukf::h_lidar(Eigen::Vector<double, N> x, double beam_angle) {
    double theta_rad = beam_angle + x(2);

    double cos_theta = std::cos(theta_rad);
    double sin_theta = std::sin(theta_rad);

    double d_left = (cos_theta < 0) ? (x(0) / -cos_theta) : INFINITY;
    double d_right = (cos_theta > 0) ? ((3.56616 - x(0)) / cos_theta) : INFINITY;
    double d_bottom = (sin_theta < 0) ? (x(1) / -sin_theta) : INFINITY;
    double d_top = (sin_theta > 0) ? ((3.56616 - x(1)) / sin_theta) : INFINITY;

    return Eigen::Vector<double, 2> {std::min({d_left, d_right, d_bottom, d_top}), beam_angle};
}

// private methods

Eigen::Vector<double, N> sr_ukf::process_model(const Eigen::Vector<double, N>& x, const Eigen::Vector<double, INPUTS>& u, const double& dt) {
    double track_width = 0.12;
    double v = (u(0) + u(1)) / 2;
    double dxdt = v * std::cos(x(2));
    double dydt = v * std::sin(x(2));
    double dthetadt = (u(1) - u(0)) / track_width;
    // double dxdt = x(3);
    // double dydt = x(4);
    // double dthetadt = x(5);
    return Eigen::Vector<double, N> {x(0) + (dxdt * dt), x(1) + (dydt * dt), x(2) + (dthetadt * dt)};
}

double sr_ukf::normalize_angle180(double x) {
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0) {
        x += (2 * M_PI);
    }
    return x - M_PI;
}

double sr_ukf::normalize_angle360(double x) {
    x = fmod(x, 2 * M_PI);
    if (x < 0) {
        x += (2 * M_PI);
    }
    return x;
}

Eigen::Vector<double, N> sr_ukf::state_mean(const Eigen::Matrix<double, N, 2 * N + 1>& sigmas, const Eigen::Vector<double, 2 * N + 1>& wm) {
    Eigen::Vector<double, N> x;
    x.fill(0);

    double sum_sin2 = 0;
    double sum_cos2 = 0;



    for (int i = 0; i < (2 * N + 1); i++) {
        // assume x,y,theta...... rewrite for glider :sob:
        x(0) += sigmas(0, i) * wm(i);
        x(1) += sigmas(1, i) * wm(i);

        sum_sin2 += std::sin(sigmas(2, i)) * wm(i);
        sum_cos2 += std::cos(sigmas(2, i)) * wm(i);

    }

    x(2) = std::atan2(sum_sin2, sum_cos2);


    return x;
}

Eigen::Vector<double, ROWS> sr_ukf::measurement_mean(const Eigen::Matrix<double, ROWS, 2 * N + 1>& sigmas, const Eigen::Vector<double, 2 * N + 1>& wm) {
    Eigen::Vector<double, ROWS> yhat;
    yhat.fill(0);

    for (int i = 0; i < (2 * N + 1); i++) {
        // assume r,theta..... rewrite for glider (will be easier)

        yhat(0) += sigmas(0, i) * wm(i);
    }

    yhat(1) = sigmas(1, 0);
    return yhat;
}

Eigen::Vector<double, N> sr_ukf::state_residual(const Eigen::Vector<double, N>& a, const Eigen::Vector<double, N>& b) {
    return Eigen::Vector<double, N> {a(0) - b(0), a(1) - b(1), normalize_angle180(a(2) - b(2))};
}

Eigen::Vector<double, N> sr_ukf::state_add(const Eigen::Vector<double, N>& a, const Eigen::Vector<double, N>& b) {
    return Eigen::Vector<double, N> {a(0) + b(0), a(1) + b(1), normalize_angle180(a(2) + b(2))};
}

Eigen::Vector<double, ROWS> sr_ukf::measurement_residual(const Eigen::Vector<double, ROWS>& a, const Eigen::Vector<double, ROWS>& b) {
    return Eigen::Vector<double, ROWS> {a(0) - b(0), normalize_angle180(a(1) - b(1))};
}
