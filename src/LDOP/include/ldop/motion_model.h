#pragma once

#include <vector>

#include <Eigen/Dense>

namespace ldopcore {

enum class MotionModelType {
  // LDOT 中“人”的 CA 模型：观测仍是 [x,y,z]，但只估计水平速度/加速度。
  CA2D,
  // LDOP 当前跟踪主线默认模型：3D constant acceleration。
  CA3D,
  // LDOT 中“其他目标”的 3D constant velocity 模型。
  CV3D,
  // LDOT 中“车”的 constant turn rate and acceleration 非线性模型。
  CTRA,
};

struct CAParams {
  double jerk_sigma{1.0};
  // CA3D 使用 9 维，CA2D 只读取前 7 维。
  std::vector<double> init_cov{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> meas_noise{0.5, 0.5, 0.5};
  // CA2D 只有 z 位置，没有 z 速度/加速度，因此 z 轴使用独立过程噪声。
  double z_process_noise{0.01};
};

struct CVParams {
  double acc_sigma{1.0};
  std::vector<double> init_cov{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> meas_noise{0.5, 0.5, 0.5};
};

struct CTRAParams {
  std::vector<double> init_cov{1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.5};
  std::vector<double> process_noise{0.1, 0.1, 0.01, 0.5, 0.5, 0.1, 0.1};
  std::vector<double> meas_noise{0.5, 0.5, 0.5};
};

class MotionModel {
 public:
  MotionModel() = default;
  virtual ~MotionModel() = default;

  virtual MotionModelType type() const = 0;

  // 初始 detection 统一是位置观测 [x,y,z]。不同模型自己决定速度/加速度/yaw 如何初始化。
  virtual Eigen::VectorXd getInitState(const Eigen::VectorXd& detection) const = 0;
  virtual Eigen::MatrixXd getInitCovP() const = 0;
  virtual Eigen::MatrixXd getProcessNoiseQ() const = 0;
  virtual Eigen::MatrixXd getMeasNoiseR() const = 0;

  // 线性模型返回状态转移矩阵；EKF 模型返回非线性转移的一阶雅可比。
  virtual Eigen::MatrixXd getTransitionF(const Eigen::VectorXd& state) const = 0;
  // 线性模型返回观测矩阵；EKF 模型返回状态到观测映射的一阶雅可比。
  virtual Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd& state) const = 0;

  // 默认按线性模型处理；CTRA 这类非线性模型会覆写该函数。
  virtual Eigen::VectorXd stateTransition(const Eigen::VectorXd& state) const;
  virtual Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd& state) const = 0;

  virtual void normalizeYaw(Eigen::VectorXd& state) const = 0;
  virtual void normalizeYawInResidual(Eigen::VectorXd& residual) const = 0;

  // dt 已在 tracker 时间戳边界统一清洗/截断；模型这里只缓存本次预测步长，
  void setDt(double dt) { dt_ = dt; }
  double dt() const { return dt_; }
  int stateDim() const { return state_dim_; }
  int measurementDim() const { return measurement_dim_; }

 protected:
  static double wrapToPi(double angle);

  int state_dim_{0};
  int measurement_dim_{0};
  double dt_{0.1};
};

class CAMotionModel final : public MotionModel {
 public:
  CAMotionModel(const CAParams& params, bool use_3d);

  MotionModelType type() const override;
  Eigen::VectorXd getInitState(const Eigen::VectorXd& detection) const override;
  Eigen::MatrixXd getInitCovP() const override;
  Eigen::MatrixXd getProcessNoiseQ() const override;
  Eigen::MatrixXd getMeasNoiseR() const override;
  Eigen::MatrixXd getTransitionF(const Eigen::VectorXd& state) const override;
  Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd& state) const override;
  Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd& state) const override;
  void normalizeYaw(Eigen::VectorXd& state) const override;
  void normalizeYawInResidual(Eigen::VectorXd& residual) const override;

 private:
  bool use_3d_{true};
  CAParams params_;
};

class CVMotionModel final : public MotionModel {
 public:
  explicit CVMotionModel(const CVParams& params);

  MotionModelType type() const override;
  Eigen::VectorXd getInitState(const Eigen::VectorXd& detection) const override;
  Eigen::MatrixXd getInitCovP() const override;
  Eigen::MatrixXd getProcessNoiseQ() const override;
  Eigen::MatrixXd getMeasNoiseR() const override;
  Eigen::MatrixXd getTransitionF(const Eigen::VectorXd& state) const override;
  Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd& state) const override;
  Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd& state) const override;
  void normalizeYaw(Eigen::VectorXd& state) const override;
  void normalizeYawInResidual(Eigen::VectorXd& residual) const override;

 private:
  CVParams params_;
};

class CTRAMotionModel final : public MotionModel {
 public:
  explicit CTRAMotionModel(const CTRAParams& params);

  MotionModelType type() const override;
  Eigen::VectorXd getInitState(const Eigen::VectorXd& detection) const override;
  Eigen::MatrixXd getInitCovP() const override;
  Eigen::MatrixXd getProcessNoiseQ() const override;
  Eigen::MatrixXd getMeasNoiseR() const override;
  Eigen::VectorXd stateTransition(const Eigen::VectorXd& state) const override;
  Eigen::MatrixXd getTransitionF(const Eigen::VectorXd& state) const override;
  Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd& state) const override;
  Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd& state) const override;
  void normalizeYaw(Eigen::VectorXd& state) const override;
  void normalizeYawInResidual(Eigen::VectorXd& residual) const override;

 private:
  CTRAParams params_;
};

}  // namespace ldopcore
