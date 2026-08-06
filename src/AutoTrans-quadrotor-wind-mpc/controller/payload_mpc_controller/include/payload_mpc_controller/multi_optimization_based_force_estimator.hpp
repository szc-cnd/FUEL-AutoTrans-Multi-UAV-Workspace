/**
 * @file multi_optimization_based_force_estimator.hpp
 * @brief Sliding-window optimization based external force estimator.
 */
#pragma once

#include <float.h>
#include <chrono>
#include <deque>
#include <math.h>
#include "Eigen/Eigen"
#include "lbfgs.hpp"
#include "lowpassfilter2p.h"
#include "mpc_params.h"

namespace PayloadMPC
{

  class MultiOptForceEstimator
  {
    typedef struct
    {
      // force_balance = m_q * a_world - T * R * e3，世界系，单位 N。
      // a_world 使用 MAVROS IMU 机体系含重力加速度经姿态旋转得到，不额外减重力。
      Eigen::Vector3d force_balance;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } systemState;

    inline static double huber_loss(const double residual, double &gradient, const double delta = 1.0)
    {
      const double abs_r = fabs(residual);
      if (abs_r <= delta)
      {
        gradient = residual;
        return 0.5 * residual * residual;
      }
      gradient = delta * (residual > 0.0 ? 1.0 : -1.0);
      return delta * (abs_r - 0.5 * delta);
    }

    static double cost_function(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g)
    {
      double cost = 0.0;
      MultiOptForceEstimator &estimator = *(MultiOptForceEstimator *)instance;
      Eigen::Map<const Eigen::Matrix3Xd> fq_array(x.data(), 3, estimator.param_num_);
      Eigen::Map<Eigen::Matrix3Xd> fq_gradient(g.data(), 3, estimator.param_num_);
      fq_gradient.setZero();

      Eigen::Vector3d fq_avg = fq_array.rowwise().sum() / double(estimator.param_num_);
      Eigen::Matrix3Xd fq_diff = fq_array;
      fq_diff.colwise() -= fq_avg;
      cost += estimator.var_weight_ * fq_diff.squaredNorm();

      Eigen::Matrix3Xd fq_var_grad = fq_diff;
      Eigen::Vector3d temp_fq = fq_diff.rowwise().sum() / double(estimator.param_num_);
      fq_var_grad.colwise() -= temp_fq;
      fq_gradient += estimator.var_weight_ * 2.0 * fq_var_grad;

      int i = 0;
      for (const systemState &state : estimator.state_buffer)
      {
        Eigen::Vector3d r = fq_array.col(i) - state.force_balance;
        double huber_gradient;
        cost += huber_loss(r.x(), huber_gradient);
        fq_gradient(0, i) += huber_gradient;
        cost += huber_loss(r.y(), huber_gradient);
        fq_gradient(1, i) += huber_gradient;
        cost += huber_loss(r.z(), huber_gradient);
        fq_gradient(2, i) += huber_gradient;
        ++i;
      }

      return cost;
    }

    Eigen::Vector3d opt_fq_{Eigen::Vector3d::Zero()};
    Eigen::VectorXd opt_variable;
    double var_weight_{50.0};

  public:
    Eigen::Vector3d quad_acc_{Eigen::Vector3d::Zero()};
    Eigen::Vector4d rpm_{Eigen::Vector4d::Zero()};
    Eigen::Quaterniond quad_q_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d Thr_{Eigen::Vector3d::Zero()};
    double T_{0.0}, sqrt_kf_{0.0};

    std::deque<systemState> state_buffer;
    LowPassFilter2p<Eigen::Vector3d> fq_filter_;

    double mass_quad_{1.0};
    double sample_freq_fq_{333.333};
    double cutoff_freq_fq_{100.0};
    double max_force_{10.0}, max_force_sqr_{100.0};
    bool use_force_estimator_{false};
    int queue_size_{10};
    int param_num_{0};

    void init(MpcParams &params)
    {
      mass_quad_ = params.dyn_params_.mass_q;
      sqrt_kf_ = params.force_estimator_param_.sqrt_kf;
      use_force_estimator_ = params.force_estimator_param_.enable_force_estimation;
      max_force_ = params.force_estimator_param_.max_force;
      max_force_sqr_ = max_force_ * max_force_;
      sample_freq_fq_ = params.force_estimator_param_.sample_freq_fq;
      cutoff_freq_fq_ = params.force_estimator_param_.cutoff_freq_fq;
      var_weight_ = params.force_estimator_param_.var_weight;
      queue_size_ = params.force_estimator_param_.max_queue;

      fq_filter_.set_cutoff_frequency(sample_freq_fq_, cutoff_freq_fq_);
      fq_filter_.reset(Eigen::Vector3d::Zero());

      opt_variable.resize(3 * (queue_size_ + 1));
      opt_variable.setZero();
    }

    void enableForceEstimator()
    {
      use_force_estimator_ = true;
    }

    void disableForceEstimator()
    {
      use_force_estimator_ = false;
    }

    void reset()
    {
      state_buffer.clear();
      param_num_ = 0;
      opt_variable.setZero();
      opt_fq_.setZero();
      fq_filter_.reset(Eigen::Vector3d::Zero());
    }

    size_t sampleCount() const
    {
      return state_buffer.size();
    }

    bool windowFull() const
    {
      return queue_size_ > 0 && state_buffer.size() >= static_cast<size_t>(queue_size_);
    }

    void setSystemState(const Eigen::Vector3d &quad_acc_body,
                        const Eigen::Quaterniond &Rotwb,
                        const Eigen::Vector4d &Rpm)
    {
      // /mavros/imu/data 的 linear_acceleration 为 base_link 机体系、含重力，单位 m/s^2。
      quad_q_ = Rotwb.normalized();
      quad_acc_ = quad_q_ * quad_acc_body;

      rpm_ = Rpm;
      T_ = (sqrt_kf_ * rpm_).squaredNorm();
      Thr_ = T_ * quad_q_.toRotationMatrix().col(2);

      systemState state;
      state.force_balance = mass_quad_ * quad_acc_ - Thr_;

      state_buffer.emplace_back(state);
      while (state_buffer.size() > (size_t)queue_size_)
      {
        state_buffer.pop_front();
      }
    }

    bool caculate_force(Eigen::Vector3d &fl, Eigen::Vector3d &fq)
    {
      fl = Eigen::Vector3d::Zero();
      if (use_force_estimator_)
      {
        if (state_buffer.size() <= 2)
        {
          fq = Eigen::Vector3d::Zero();
          return false;
        }
        else
        {
          param_num_ = state_buffer.size();
          if (opt_variable.size() != 3 * param_num_)
          {
            opt_variable.resize(3 * param_num_);
            opt_variable.setZero();
          }
          if (opt_variable.maxCoeff() > 3 * max_force_ || opt_variable.minCoeff() < -3 * max_force_)
          {
            opt_variable.setZero();
          }

          lbfgs::lbfgs_parameter_t opt_param;
          opt_param.mem_size = 32;
          opt_param.g_epsilon = 1.0e-5;
          opt_param.delta = 1.0e-6;
          opt_param.past = 0;
          opt_param.max_iterations = 500;
          double final_cost;

          int error_code = lbfgs::lbfgs_optimize(opt_variable, final_cost, MultiOptForceEstimator::cost_function, nullptr, nullptr, this, opt_param);
          const bool optimizer_valid = error_code >= 0 || error_code == lbfgs::LBFGSERR_MAXIMUMITERATION;
          if (!optimizer_valid)
          {
            std::cout << "[Force estimator]error code: " << std::string(lbfgs::lbfgs_strerror(error_code)) << std::endl;
            opt_variable.setZero();
            opt_fq_.setZero();
            fq_filter_.reset(Eigen::Vector3d::Zero());
            fq.setZero();
            return false;
          }

          Eigen::Map<Eigen::Matrix3Xd> all_opt_fq(opt_variable.data(), 3, param_num_);
          opt_fq_ = all_opt_fq.col(param_num_ - 1);
          if (opt_fq_.squaredNorm() > max_force_sqr_)
          {
            opt_fq_ = opt_fq_.normalized() * max_force_;
            all_opt_fq.setZero();
          }
          fq = opt_fq_;
        }

        opt_fq_ = fq_filter_.apply(fq);
        fq = opt_fq_;
        if (!fq.allFinite())
        {
          opt_variable.setZero();
          opt_fq_.setZero();
          fq_filter_.reset(Eigen::Vector3d::Zero());
          fq.setZero();
          return false;
        }
        // 二阶低通可能在阶跃输入后产生过冲；滤波后再次限幅，保证发布值和 NMPC 补偿值
        // 的世界系合力始终不超过 max_force_（单位 N）。
        if (fq.squaredNorm() > max_force_sqr_)
        {
          fq = fq.normalized() * max_force_;
          opt_fq_ = fq;
        }
        return true;
      }
      else
      {
        fq = Eigen::Vector3d::Zero();
        return false;
      }
    }
  };

} // namespace PayloadMPC
