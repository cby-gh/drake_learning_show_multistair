#pragma once

#include <set>

#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

/// A constraint class to wrap the position component of a KinematicEvaluatorSet
/// 
/// In its most basic form, this constraint is of the form
///    lb <= phi(q) <= ub
/// corresponding to the __active__ constraints only.
///
/// Constraints can also be identified as "relative", where indices are
/// supplied with respect to the full constraint list. This is to eliminate
/// the need for the user to adjust indices depending on which constraints are
/// marked active/full.
///
/// Relative constraints are shifted by the value of some additional decision
/// variable alpha. The constraint is then
///  lb <= phi(q) - alpha <= ub
///
/// The decision variables for this constraint are q and alpha
template <typename T>
class KinematicPositionConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// This is the simplest form of the construtor, where the lower and upper
  /// bounds are both zero.
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_position", int mode_indexkp = 0);

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes three additional arguments: lower and upper bounds,
  /// and a vector of booleans identifying whether the ith __full__ constraint
  /// is relative.
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::set<int>& full_constraint_relative = {},
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_position", int mode_indexkp = 0);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
  std::set<int> full_constraint_relative_;
  int mode_indexkp_;
};

/// A constraint class to wrap the velocity component of a KinematicEvaluatorSet
/// 
/// This constraint is of the form
///    lb <= d/dt phi(q) <= ub
/// corresponding to the __active__ constraints only.
///
/// The decision variables for this constraint are q and v
///
/// Unlike KinematicPositionConstraint, there is no need for the relative flag.
template <typename T>
class KinematicVelocityConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// This is the simplest form of the construtor, where the lower and upper
  /// bounds are both zero.
  KinematicVelocityConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_velocity",int i_mode=0);

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes additional arguments: lower and upper bounds.
  KinematicVelocityConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_velocity", int i_mode=0);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
  int i_mode_;
};

/// A constraint class to wrap the acceleration component of a
/// KinematicEvaluatorSet
/// 
/// This constraint is of the form
///    lb <= d^2/dt^2 phi(q) <= u
/// corresponding to the __active__ constraints only.
///
/// To calculate acceleration, the decision variables for this constraint are
/// q, v, u and constraint force lambda (corresponding to the full set).
///
/// Unlike KinematicPositionConstraint, there is no need for the relative flag.
template <typename T>
class KinematicAccelerationConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// This is the simplest form of the construtor, where the lower and upper
  /// bounds are both zero.
  KinematicAccelerationConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_acceleration");

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes additional arguments: lower and upper bounds.
  KinematicAccelerationConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_acceleration");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
};

template <typename T>
class foot_location_Constraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// This is the simplest form of the construtor, where the lower and upper
  /// bounds are both zero.
  foot_location_Constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "foot_location_Constraint");

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes three additional arguments: lower and upper bounds,
  /// and a vector of booleans identifying whether the ith __full__ constraint
  /// is relative.
  foot_location_Constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::set<int>& full_constraint_relative = {},
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_position");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
  std::set<int> full_constraint_relative_;
};

template <typename T>
class floatfoot_collosionavoidence_Constraint : public solvers::NonlinearConstraint<T> 
{
 public:

  floatfoot_collosionavoidence_Constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "floatfoot should not hit the ground",
      int mode_index=0,
      int knot_index=0);

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes three additional arguments: lower and upper bounds,
  /// and a vector of booleans identifying whether the ith __full__ constraint
  /// is relative.
  floatfoot_collosionavoidence_Constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::set<int>& full_constraint_relative = {},
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "floatfoot should not hit the ground",
      int mode_index=0,
      int knot_index=0);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
  std::set<int> full_constraint_relative_;
  const int mode_index_;
  const int knot_index_;
};

template <typename T>
class quat2euler_constraint : public solvers::NonlinearConstraint<T> 
{
 public:

  quat2euler_constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "convert quaternion to static euler angle and then constrains each angle.",
      int mode_index=0,
      int knot_index=0);

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes three additional arguments: lower and upper bounds,
  /// and a vector of booleans identifying whether the ith __full__ constraint
  /// is relative.
  quat2euler_constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "convert quaternion to static euler angle and then constrains each angle",
      int mode_index=0,
      int knot_index=0);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
  const int mode_index_;
  const int knot_index_;
};

template <typename T>
class footpose_constraint : public solvers::NonlinearConstraint<T> 
{
 public:
  footpose_constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = " euler angle and then constrains each foot.",
      int mode_index=0,
      int knot_index=0);

  footpose_constraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "euler angle and then constrains each foot",
      int mode_index=0,
      int knot_index=0,
      int index=0);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
  const int mode_index_;
  const int knot_index_;
  const int constraint_index_;
};

}  // namespace multibody
}  // namespace dairlib
