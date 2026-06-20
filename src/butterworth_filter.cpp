#include "butterworth_filter.hpp"

#include <cmath>
#include <complex>
#include <stdexcept>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace signal_processing {

namespace {

/// Compute binomial coefficient C(n, k).
constexpr double binomial(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1;
    // Use multiplicative formula for numerical stability with small n.
    double result = 1.0;
    for (int i = 1; i <= k; ++i) {
        result = result * (n - k + i) / i;
    }
    return result;
}

/// Compute polynomial coefficients from complex roots.
///
/// Given roots r_0..r_{m-1}, computes real coefficients p_0..p_m such that
///   p_0 + p_1*z + ... + p_m*z^m = prod(z - r_i)
///
/// Uses complex arithmetic internally and returns the real part. Since
/// Butterworth poles come in conjugate pairs, the imaginary residues cancel
/// to machine precision.
Eigen::VectorXd polyFromRoots(const std::vector<std::complex<double>>& roots) {
    int m = static_cast<int>(roots.size());

    // Start with polynomial "1" (coefficient of z^0 = 1)
    std::vector<std::complex<double>> coeffs(m + 1, std::complex<double>(0, 0));
    coeffs[0] = std::complex<double>(1.0, 0.0);

    // For each root r, multiply current polynomial by (z - r)
    for (int k = 0; k < m; ++k) {
        std::complex<double> r = roots[k];
        // new_coeffs[i] = coeffs[i-1] - r * coeffs[i]
        // (with coeffs[-1] = 0, coeffs[m] treated as 0)
        std::vector<std::complex<double>> new_coeffs(m + 1, std::complex<double>(0, 0));
        for (int i = 0; i <= k + 1; ++i) {
            std::complex<double> prev = (i >= 1) ? coeffs[i - 1] : std::complex<double>(0, 0);
            std::complex<double> cur = (i <= k) ? coeffs[i] : std::complex<double>(0, 0);
            new_coeffs[i] = prev - r * cur;
        }
        coeffs = new_coeffs;
    }

    // Extract real parts (imaginary parts should be ~0)
    Eigen::VectorXd result(m + 1);
    for (int i = 0; i <= m; ++i) {
        result(i) = coeffs[i].real();
    }
    return result;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

ButterworthFilterDesign designButterworthLowpass(double Wp, double Ws,
                                                  double Rp, double Rs,
                                                  double fs) {
    if (Wp <= 0 || Wp >= 1 || Ws <= Wp || Ws >= 1) {
        throw std::invalid_argument(
            "designButterworthLowpass: require 0 < Wp < Ws < 1");
    }
    if (fs <= 0) {
        throw std::invalid_argument(
            "designButterworthLowpass: fs must be positive");
    }

    // 1. Pre-warp analog frequencies
    //    Wp, Ws are normalised [0,1] where 1 = Nyquist (fs/2).
    //    The bilinear prewarp is: Omega = 2*fs * tan(pi * W / 2)
    double omega_p = 2.0 * fs * std::tan(M_PI * Wp / 2.0);
    double omega_s = 2.0 * fs * std::tan(M_PI * Ws / 2.0);

    // 2. Minimum order
    double ep = std::sqrt(std::pow(10.0, 0.1 * Rp) - 1.0);
    double es = std::sqrt(std::pow(10.0, 0.1 * Rs) - 1.0);
    double n_float = std::log10(es / ep) / std::log10(omega_s / omega_p);
    int n = static_cast<int>(std::ceil(n_float));
    if (n < 1) n = 1;

    // 3. 3-dB cutoff (analog rad/s)
    double omega_c = omega_p / std::pow(ep, 1.0 / n);

    // 4. Normalized 3-dB cutoff (digital, for reporting)
    double wn = 2.0 / M_PI * std::atan(omega_c / (2.0 * fs));

    // 5. Compute n LHP Butterworth poles in the s-plane
    std::vector<std::complex<double>> poles;
    poles.reserve(n);
    for (int k = 0; k < n; ++k) {
        double angle = M_PI * (2.0 * k + n + 1.0) / (2.0 * n);
        double real = omega_c * std::cos(angle);
        double imag = omega_c * std::sin(angle);
        // real is always negative for Butterworth LHP poles
        poles.emplace_back(real, imag);
    }

    // 6. Bilinear transform: map each s-plane pole to a z-plane pole
    double two_fs = 2.0 * fs;
    std::vector<std::complex<double>> digital_poles;
    digital_poles.reserve(n);
    for (const auto& p : poles) {
        // z = (2*fs + p) / (2*fs - p)
        auto z = (two_fs + p) / (two_fs - p);
        digital_poles.push_back(z);
    }

    // 7. Denominator polynomial from digital poles.
    //    polyFromRoots returns coefficients in ascending powers of z:
    //      c[0] + c[1]*z + ... + c[n]*z^n = prod(z - z_k)
    //    We need MATLAB convention: a[k] is coefficient of z^{-k}, a[0] = 1.
    Eigen::VectorXd c = polyFromRoots(digital_poles);
    double cn = c(n);
    Eigen::VectorXd a(n + 1);
    for (int k = 0; k <= n; ++k) {
        // a[k] = c[n-k] / c[n]   (k=0..n, a[0] = c[n]/c[n] = 1)
        a(k) = c(n - k) / cn;
    }

    // 8. Numerator: n zeros at z = -1  →  H(z) ∝ (1 + z^{-1})^n
    //    b_raw[k] = C(n, k)  are the coefficients of z^{-k} in (1+z^{-1})^n
    //    (binomial coefficients are symmetric: C(n,k) = C(n,n-k))
    Eigen::VectorXd b_raw(n + 1);
    for (int k = 0; k <= n; ++k) {
        b_raw(k) = binomial(n, k);
    }

    // 9. DC gain normalisation: H(z=1) = sum(b)/sum(a) = 1
    double sum_a = a.sum();
    double sum_b_raw = b_raw.sum();   // = 2^n
    double gain = sum_a / sum_b_raw;
    Eigen::VectorXd b = gain * b_raw;

    ButterworthFilterDesign design;
    design.order = n;
    design.cutoff_normalized = wn;
    design.b = b;
    design.a = a;
    return design;
}

Eigen::VectorXd filter(const Eigen::VectorXd& b, const Eigen::VectorXd& a,
                       const Eigen::VectorXd& x) {
    int na = static_cast<int>(a.size());
    int nstates = (na > 0) ? na - 1 : 0;
    Eigen::VectorXd zi = Eigen::VectorXd::Zero(nstates);
    return filterWithZi(b, a, x, zi);
}

Eigen::VectorXd filterWithZi(const Eigen::VectorXd& b, const Eigen::VectorXd& a,
                             const Eigen::VectorXd& x,
                             const Eigen::VectorXd& zi) {
    int N = static_cast<int>(x.size());
    int na = static_cast<int>(a.size());
    int nb = static_cast<int>(b.size());

    Eigen::VectorXd y = Eigen::VectorXd::Zero(N);
    int nstates = na - 1;
    if (nstates < 0) nstates = 0;

    // Copy initial state (scaled by x[0] as in scipy convention)
    Eigen::VectorXd w = zi * x(0);

    for (int i = 0; i < N; ++i) {
        double xi = x(i);
        // Output
        double yi = b(0) * xi + (nstates > 0 ? w(0) : 0.0);
        y(i) = yi;

        // Update states
        for (int j = 0; j < nstates - 1; ++j) {
            double bj1 = (j + 1 < nb) ? b(j + 1) : 0.0;
            double aj1 = (j + 1 < na) ? a(j + 1) : 0.0;
            w(j) = bj1 * xi + w(j + 1) - aj1 * yi;
        }
        if (nstates > 0) {
            int last = nstates - 1;
            double blast = (last + 1 < nb) ? b(last + 1) : 0.0;
            double alast = (last + 1 < na) ? a(last + 1) : 0.0;
            w(last) = blast * xi - alast * yi;
        }
    }
    return y;
}

Eigen::VectorXd computeFilterZi(const Eigen::VectorXd& b,
                                const Eigen::VectorXd& a) {
    int n = std::max(static_cast<int>(a.size()), static_cast<int>(b.size())) - 1;
    if (n <= 0) return Eigen::VectorXd();

    // DC gain K = sum(b) / sum(a)
    double sum_b = b.sum();
    double sum_a = a.sum();
    double K = sum_b / sum_a;

    // Compute suffix sums of b and a
    //   sum_b_tail[i] = sum_{j=i+1}^{n} b[j]
    //   sum_a_tail[i] = sum_{j=i+1}^{n} a[j]
    Eigen::VectorXd zi(n);

    double b_tail = 0.0;
    double a_tail = 0.0;
    // Work backwards from the last state to the first.
    // For each state index i, the tail sum includes coefficients b[i+1..n] and a[i+1..n].
    for (int i = n - 1; i >= 0; --i) {
        int j = i + 1;  // coefficient index corresponding to state w[i]
        double bj = (j < b.size()) ? b(j) : 0.0;
        double aj = (j < a.size()) ? a(j) : 0.0;

        // Accumulate this coefficient into the tail sum FIRST,
        // so that zi(i) includes b[j] and a[j].
        b_tail += bj;
        a_tail += aj;

        // Steady-state: w[i] = (sum_{k=j}^{n} b[k] - K * sum_{k=j}^{n} a[k]) * x
        zi(i) = b_tail - K * a_tail;
    }

    return zi;
}

Eigen::MatrixXd filtfilt(const Eigen::VectorXd& b, const Eigen::VectorXd& a,
                         const Eigen::MatrixXd& X) {
    int K = static_cast<int>(X.rows());   // number of time samples
    int C = static_cast<int>(X.cols());   // number of channels
    int nfilt = static_cast<int>(std::max(b.size(), a.size()));
    int nedge = 3 * (nfilt - 1);           // MATLAB-style edge extension length

    // Clamp nedge if signal is too short
    if (nedge > K) {
        nedge = K / 2;
        if (nedge < 0) nedge = 0;
    }

    // Compute steady-state initial conditions (MATLAB lfilter_zi equivalent)
    Eigen::VectorXd zi = computeFilterZi(b, a);

    Eigen::MatrixXd Y(K, C);

    for (int c = 0; c < C; ++c) {
        Eigen::VectorXd x = X.col(c);

        // --- Odd-reflection extension at both ends ---
        int ext_len = K + 2 * nedge;
        Eigen::VectorXd x_ext(ext_len);

        // Copy original signal into centre
        x_ext.segment(nedge, K) = x;

        // Left extension: x_ext[i] = 2*x[0] - x[nedge - i]
        for (int i = 0; i < nedge; ++i) {
            x_ext(i) = 2.0 * x(0) - x(nedge - i);
        }

        // Right extension: x_ext[K+nedge+i] = 2*x[K-1] - x[K-2-i]
        for (int i = 0; i < nedge; ++i) {
            x_ext(K + nedge + i) = 2.0 * x(K - 1) - x(K - 2 - i);
        }

        // --- Forward filter with steady-state initial conditions ---
        Eigen::VectorXd y_fwd = filterWithZi(b, a, x_ext, zi);

        // --- Reverse ---
        Eigen::VectorXd y_rev = y_fwd.reverse();

        // --- Filter reversed (zi scaled by first value of reversed signal) ---
        Eigen::VectorXd y_rev_filt = filterWithZi(b, a, y_rev, zi);

        // --- Reverse back ---
        Eigen::VectorXd y_final = y_rev_filt.reverse();

        // --- Strip extensions ---
        Y.col(c) = y_final.segment(nedge, K);
    }

    return Y;
}

Eigen::MatrixXd centralDifference(const Eigen::MatrixXd& X, double Ts) {
    int K = static_cast<int>(X.rows());
    int C = static_cast<int>(X.cols());
    Eigen::MatrixXd Xdot = Eigen::MatrixXd::Zero(K, C);

    if (K < 3) {
        // Not enough points for central difference; return zeros
        return Xdot;
    }

    double denom = 2.0 * Ts;

    // Interior points (indices 1 .. K-2)
    for (int i = 1; i < K - 1; ++i) {
        Xdot.row(i) = (X.row(i + 1) - X.row(i - 1)) / denom;
    }

    // Boundary handling: match MATLAB revoarm_filter.m lines 42-43
    // 1. First row stays as 0 (MATLAB zeros() initialisation)
    Xdot.row(0) = Eigen::VectorXd::Zero(C);
    // 2. Last row stays as 0 (never computed by the for loop)
    //    (already zero-initialised)
    // 3. The last interior row (i = K-2 in 0-indexed) has its first and last
    //    columns overwritten with adjacent columns (MATLAB lines 42-43):
    //      q_ddot_filtered(i,1)   = q_ddot_filtered(i,2);
    //      q_ddot_filtered(i,end) = q_ddot_filtered(i,end-1);
    if (K >= 3 && C >= 2) {
        Xdot(K - 2, 0) = Xdot(K - 2, 1);
        Xdot(K - 2, C - 1) = Xdot(K - 2, C - 2);
    }

    return Xdot;
}

}  // namespace signal_processing
