#pragma once

#include <Eigen/Dense>
#include <complex>
#include <vector>

namespace signal_processing {

/// Holds the design result of a digital Butterworth lowpass filter.
struct ButterworthFilterDesign {
    int order = 0;                 ///< Filter order n
    double cutoff_normalized = 0;  ///< Normalized 3-dB cutoff frequency Wn
    Eigen::VectorXd b;             ///< Numerator coefficients, length n+1
    Eigen::VectorXd a;             ///< Denominator coefficients, length n+1 (a[0] == 1)
};

/// Design a digital Butterworth lowpass filter using bilinear transform.
///
/// Implements the equivalent of MATLAB's buttord + butter.
///
/// @param Wp  Normalized passband edge (0 < Wp < 1)
/// @param Ws  Normalized stopband edge (Wp < Ws < 1)
/// @param Rp  Passband ripple in dB
/// @param Rs  Stopband attenuation in dB
/// @param fs  Sampling frequency in Hz
/// @return     Filter design with b, a coefficients
ButterworthFilterDesign designButterworthLowpass(double Wp, double Ws,
                                                  double Rp, double Rs,
                                                  double fs);

/// Apply a digital IIR filter (direct form II transposed) to a 1-D signal.
///
/// Uses zero initial conditions.
///
/// @param b  Numerator coefficients
/// @param a  Denominator coefficients (a[0] must be 1)
/// @param x  Input signal vector (length N)
/// @return   Filtered output vector (length N)
Eigen::VectorXd filter(const Eigen::VectorXd& b, const Eigen::VectorXd& a,
                       const Eigen::VectorXd& x);

/// Apply a digital IIR filter with user-supplied initial state.
///
/// @param b     Numerator coefficients
/// @param a     Denominator coefficients (a[0] must be 1)
/// @param x     Input signal vector (length N)
/// @param zi    Initial state vector (length na-1), multiplied by x[0]
/// @return      Filtered output vector (length N)
Eigen::VectorXd filterWithZi(const Eigen::VectorXd& b, const Eigen::VectorXd& a,
                             const Eigen::VectorXd& x,
                             const Eigen::VectorXd& zi);

/// Compute steady-state initial conditions for lfilter (MATLAB lfilter_zi).
///
/// Returns zi such that using filterWithZi(b, a, x, zi) produces zero
/// startup transient for a constant input: the output reaches its DC value
/// immediately.
///
/// @param b  Numerator coefficients
/// @param a  Denominator coefficients (a[0] must be 1)
/// @return   zi vector (length na-1)
Eigen::VectorXd computeFilterZi(const Eigen::VectorXd& b,
                                const Eigen::VectorXd& a);

/// Zero-phase forward-backward filtering (MATLAB filtfilt equivalent).
///
/// Applies the filter forward, reverses the filtered sequence, filters again,
/// and reverses the final output.  Edges are padded with odd reflection to
/// minimise start-up transients.
///
/// @param b  Numerator coefficients
/// @param a  Denominator coefficients (a[0] must be 1)
/// @param X  Input matrix: rows = time samples, cols = channels
/// @return   Zero-phase filtered matrix (same dimensions as X)
Eigen::MatrixXd filtfilt(const Eigen::VectorXd& b, const Eigen::VectorXd& a,
                         const Eigen::MatrixXd& X);

/// Compute first derivative via central difference.
///
/// For interior points:  dx_i = (x_{i+1} - x_{i-1}) / (2*Ts)
/// Boundary points copy the nearest interior result.
///
/// @param X   Input matrix: rows = time samples, cols = channels
/// @param Ts  Sampling period in seconds
/// @return    Derivative matrix (same dimensions as X)
Eigen::MatrixXd centralDifference(const Eigen::MatrixXd& X, double Ts);

}  // namespace signal_processing
