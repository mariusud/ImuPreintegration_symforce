// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

namespace sym {

/**
 * Residual for accelerometer/gyroscope Bias between factor.
 *     A wiener process (random walk model) where the bias changes at a constant rate
 *     over time. time_delta is the time between the two bias states.
 *
 *     residual = (time_delta * diagonal_information) * (bias_i - bias_j)
 *     jacobian: (3x6) jacobian of res wrt args prev_bias (3), next_bias (3)
 *     hessian: (6x6) Gauss-Newton hessian for args prev_bias (3), next_bias (3)
 *     rhs: (6x1) Gauss-Newton rhs for args prev_bias (3), next_bias (3)
 */
template <typename Scalar>
void BiasBetweenFactor(const Eigen::Matrix<Scalar, 3, 1>& diag_sqrt_info, const Scalar time_delta,
                       const Eigen::Matrix<Scalar, 3, 1>& prev_bias,
                       const Eigen::Matrix<Scalar, 3, 1>& next_bias,
                       Eigen::Matrix<Scalar, 3, 1>* const res = nullptr,
                       Eigen::Matrix<Scalar, 3, 6>* const jacobian = nullptr,
                       Eigen::Matrix<Scalar, 6, 6>* const hessian = nullptr,
                       Eigen::Matrix<Scalar, 6, 1>* const rhs = nullptr) {
  // Total ops: 28

  // Input arrays

  // Intermediate terms (13)
  const Scalar _tmp0 = -next_bias(0, 0) + prev_bias(0, 0);
  const Scalar _tmp1 = std::sqrt(time_delta);
  const Scalar _tmp2 = _tmp1 * diag_sqrt_info(0, 0);
  const Scalar _tmp3 = -next_bias(1, 0) + prev_bias(1, 0);
  const Scalar _tmp4 = _tmp1 * diag_sqrt_info(1, 0);
  const Scalar _tmp5 = -next_bias(2, 0) + prev_bias(2, 0);
  const Scalar _tmp6 = _tmp1 * diag_sqrt_info(2, 0);
  const Scalar _tmp7 = std::pow(diag_sqrt_info(0, 0), Scalar(2)) * time_delta;
  const Scalar _tmp8 = std::pow(diag_sqrt_info(1, 0), Scalar(2)) * time_delta;
  const Scalar _tmp9 = std::pow(diag_sqrt_info(2, 0), Scalar(2)) * time_delta;
  const Scalar _tmp10 = _tmp0 * _tmp7;
  const Scalar _tmp11 = _tmp3 * _tmp8;
  const Scalar _tmp12 = _tmp5 * _tmp9;

  // Output terms (4)
  if (res != nullptr) {
    Eigen::Matrix<Scalar, 3, 1>& _res = (*res);

    _res(0, 0) = _tmp0 * _tmp2;
    _res(1, 0) = _tmp3 * _tmp4;
    _res(2, 0) = _tmp5 * _tmp6;
  }

  if (jacobian != nullptr) {
    Eigen::Matrix<Scalar, 3, 6>& _jacobian = (*jacobian);

    _jacobian.setZero();

    _jacobian(0, 0) = _tmp2;
    _jacobian(1, 1) = _tmp4;
    _jacobian(2, 2) = _tmp6;
    _jacobian(0, 3) = -_tmp2;
    _jacobian(1, 4) = -_tmp4;
    _jacobian(2, 5) = -_tmp6;
  }

  if (hessian != nullptr) {
    Eigen::Matrix<Scalar, 6, 6>& _hessian = (*hessian);

    _hessian.setZero();

    _hessian(0, 0) = _tmp7;
    _hessian(3, 0) = -_tmp7;
    _hessian(1, 1) = _tmp8;
    _hessian(4, 1) = -_tmp8;
    _hessian(2, 2) = _tmp9;
    _hessian(5, 2) = -_tmp9;
    _hessian(3, 3) = _tmp7;
    _hessian(4, 4) = _tmp8;
    _hessian(5, 5) = _tmp9;
  }

  if (rhs != nullptr) {
    Eigen::Matrix<Scalar, 6, 1>& _rhs = (*rhs);

    _rhs(0, 0) = _tmp10;
    _rhs(1, 0) = _tmp11;
    _rhs(2, 0) = _tmp12;
    _rhs(3, 0) = -_tmp10;
    _rhs(4, 0) = -_tmp11;
    _rhs(5, 0) = -_tmp12;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym