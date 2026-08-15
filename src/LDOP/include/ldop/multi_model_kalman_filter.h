#pragma once

#include <deque>
#include <memory>

#include <Eigen/Dense>

#include <ldop/motion_model.h>

namespace ldopcore {

// ROS 参数快照，集中保存卡尔曼滤波器相关参数默认值；
// tracker 和 predictor 各自读取到自己的 Params 实例，再通过 buildMultiModelKalmanFilterConfig 转换为运行时配置。
struct MultiModelKalmanFilterParams {
  double default_dt{0.1};
  int adaptive_window_size{10};
  double adaptive_alpha{0.7};
  double adaptive_r_alpha{0.7};
  double adaptive_min_noise_ratio{0.2};
  bool enable_cov_limit{true};
  double max_pos_cov{1.0};
  double max_vel_cov{9.0};
  double max_acc_cov{25.0};
  double ca_human_jerk_sigma{1.0};
  std::vector<double> ca_human_init_cov{0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 10.0};
  std::vector<double> ca_human_meas_noise{0.1, 0.1, 0.1};
  double ca_human_z_process_noise{0.01};
  double ca_uav_jerk_sigma{1.0};
  std::vector<double> ca_uav_init_cov{0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0};
  std::vector<double> ca_uav_meas_noise{0.1, 0.1, 0.1};
  double cv_acc_sigma{3.0};
  std::vector<double> cv_init_cov{0.5, 0.5, 0.5, 2.0, 2.0, 2.0};
  std::vector<double> cv_meas_noise{0.3, 0.3, 0.3};
  std::vector<double> ctra_init_cov{0.1, 0.1, 0.1, 1.0, 10.0, 0.5, 1.0};
  std::vector<double> ctra_process_noise{0.1, 0.1, 0.01, 1.0, 10.0, 0.1, 1.0};
  std::vector<double> ctra_meas_noise{0.1, 0.1, 0.1};
};

struct MultiModelKalmanFilterConfig {
  // tracker 每帧会用真实时间戳覆盖 dt；这里是时间戳异常或新滤波器构造时的默认值。
  double default_dt{0.1};

  // 参数布局跟 LDOT 保持一致：不同类别/模型各自持有运动模型参数。
  CAParams ca_human;
  CAParams ca_uav;
  CVParams cv;
  CTRAParams ctra;

  // 自适应 Q/R 参数。window_size <= 1 表示关闭自适应估计，直接使用模型基础 Q/R。
  int adaptive_window_size{10};
  double adaptive_alpha{0.7};
  double adaptive_r_alpha{0.7};
  double adaptive_min_noise_ratio{0.2};

  // 协方差上限用于防止长时间 coast 或异常观测后状态不确定性无限膨胀。
  bool enable_cov_limit{true};
  double max_pos_cov{1.0};
  double max_vel_cov{9.0};
  double max_acc_cov{25.0};
};

class KalmanFilterBase {
 public:
  explicit KalmanFilterBase(std::shared_ptr<MotionModel> model);
  virtual ~KalmanFilterBase() = default;

  KalmanFilterBase(const KalmanFilterBase&) = delete;
  KalmanFilterBase& operator=(const KalmanFilterBase&) = delete;

  void setAdaptiveParams(int window_size,
                         double alpha,
                         double r_alpha,
                         double min_noise_ratio);
  void setCovLimitParams(bool enable, double max_pos_cov, double max_vel_cov, double max_acc_cov);

  virtual void initialize(const Eigen::VectorXd& detection);
  void initialize(const Eigen::Vector3d& detection);
  // 模型切换时需要把旧模型的世界坐标位置/速度迁移到新状态布局；
  // 这里显式接收完整状态，避免把“状态迁移”伪装成一次位置观测初始化。
  void initializeState(const Eigen::VectorXd& state);
  virtual void predict() = 0;
  virtual void update(const Eigen::VectorXd& measurement) = 0;
  void update(const Eigen::Vector3d& measurement);

  // 和 LDOT 一样，setDt 只把当前帧时间步长交给运动模型；Q/R 是否刷新由 predict/update 显式处理。
  void setDt(double dt);

  const Eigen::VectorXd& state() const { return state_; }
  const Eigen::MatrixXd& covariance() const { return covariance_; }
  MotionModelType modelType() const { return model_->type(); }
  bool isInitialized() const { return initialized_; }

  Eigen::Vector3d position() const;
  Eigen::Vector3d velocity() const;

 protected:
  void updateInnovationBuffer(const Eigen::VectorXd& innovation);
  Eigen::MatrixXd innovationSampleCovariance() const;
  void estimateAdaptiveR(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& measurement_matrix);
  void estimateAdaptiveQ(const Eigen::VectorXd& innovation,
                         const Eigen::MatrixXd& measurement_matrix,
                         const Eigen::MatrixXd& measurement_noise,
                         const Eigen::MatrixXd& kalman_gain);
  void clampCovariance();

  std::shared_ptr<MotionModel> model_;
  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd process_noise_;
  Eigen::MatrixXd measurement_noise_;

  int state_dim_{0};
  int measurement_dim_{0};
  bool initialized_{false};

  std::deque<Eigen::VectorXd> innovation_buffer_;
  int adaptive_window_size_{10};
  double adaptive_alpha_{0.7};
  double adaptive_r_alpha_{0.7};
  Eigen::MatrixXd min_process_noise_;
  Eigen::MatrixXd min_measurement_noise_;

  bool enable_cov_limit_{true};
  double max_pos_cov_{1.0};
  double max_vel_cov_{9.0};
  double max_acc_cov_{25.0};
};

class LinearKalmanFilter final : public KalmanFilterBase {
 public:
  explicit LinearKalmanFilter(std::shared_ptr<MotionModel> model);

  void predict() override;
  void update(const Eigen::VectorXd& measurement) override;
};

class ExtendedKalmanFilter final : public KalmanFilterBase {
 public:
  explicit ExtendedKalmanFilter(std::shared_ptr<MotionModel> model);

  void predict() override;
  void update(const Eigen::VectorXd& measurement) override;
};

// 从 ROS 参数快照构建运行时配置；tracker 和 predictor 各自调用，共享同一套构建逻辑。
MultiModelKalmanFilterConfig buildMultiModelKalmanFilterConfig(
    const MultiModelKalmanFilterParams& params);

// 仅构造运动模型实例，不创建 Kalman filter；predictor 做轨迹外推时使用。
std::shared_ptr<MotionModel> createMotionModel(
    MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config = {});

std::shared_ptr<KalmanFilterBase> createKalmanFilter(
    MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config = {});

std::shared_ptr<KalmanFilterBase> createKalmanFilter(
    bool is_human,
    bool is_vehicle,
    bool is_uav,
    bool is_other,
    const MultiModelKalmanFilterConfig& config = {});

}  // namespace ldopcore
