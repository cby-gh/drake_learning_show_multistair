#include "multibody/kinematic/kinematic_constraints.h"
#include <drake/math/roll_pitch_yaw.h>
#include "multibody/multibody_utils.h"
#include <drake/multibody/tree/frame.h>

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::VectorX;
using solvers::NonlinearConstraint;

///
///  KinematicPositionConstraint
///
template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description, int mode_indexkp)
    : KinematicPositionConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          std::set<int>(), context, description, mode_indexkp) {}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::set<int>& full_constraint_relative,
    Context<T>* context, const std::string& description, int mode_indexkp)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + full_constraint_relative.size(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      full_constraint_relative_(full_constraint_relative),
      mode_indexkp_(mode_indexkp) {

  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
} 

template <typename T>
void KinematicPositionConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const 
{
  const auto& q = vars.head(plant_.num_positions());
  const auto& alpha = vars.tail(full_constraint_relative_.size());
  // std::cout<<"left_roll= "<<q(7)<<std::endl;
  // std::cout<<"right_roll= "<<q(8)<<std::endl;
  SetPositionsIfNew<T>(plant_, q, context_);
  //* y = VectorXd::Zero(6);
  DRAKE_DEMAND(vars.size()==full_constraint_relative_.size()+plant_.num_positions());
  //(*y).segment(0,3) = evaluators_.EvalActive(*context_);
  (*y) = evaluators_.EvalActive(*context_);
  const auto& left_support_crank_frame = plant_.GetFrameByName("left_crank_Link");
  const auto& right_support_crank_frame = plant_.GetFrameByName("right_crank_Link");
  const auto& world_frame = plant_.world_frame();
  VectorX<T> rpy(3);
  if(mode_indexkp_==0||mode_indexkp_==2)
  {
    drake::math::RotationMatrix< T > support_foot_pose_mr =plant_.CalcRelativeRotationMatrix(*context_, left_support_crank_frame,world_frame);
    drake::math::RollPitchYaw< T >RollPitchYaw_state(support_foot_pose_mr);
    rpy(0)=RollPitchYaw_state.roll_angle();
    rpy(1)=RollPitchYaw_state.pitch_angle();
    rpy(2)=RollPitchYaw_state.yaw_angle();
  }
  else if(mode_indexkp_==1)
  {
    drake::math::RotationMatrix< T > support_foot_pose_mr=plant_.CalcRelativeRotationMatrix(*context_, right_support_crank_frame,world_frame);
    drake::math::RollPitchYaw< T >RollPitchYaw_state(support_foot_pose_mr);
    rpy(0)=RollPitchYaw_state.roll_angle();
    rpy(1)=RollPitchYaw_state.pitch_angle();
    rpy(2)=RollPitchYaw_state.yaw_angle();
  }

  //(*y).segment(3,3) = rpy;
  //VectorX<T> mm = VectorXd::Zero(3);

  //drake::VectorX<T> footlocation  = evaluators_.EvalActive(*context_);
  //std::cout<<"footlocation"<<footlocation.size()<<std::endl;
  //mm(2)=footlocation(2);

    //std::cout<<"(*y)(*y):"<<(*y)<<std::endl;
    //std::cout<<std::endl;
    // if((*y)(1)>0.1)
    //      std::cout<<"ssssdadsadadadasdsdaddfggwftg"<<std::endl;
  // Add relative offsets, looping through the list of relative constraints
  auto it = full_constraint_relative_.begin();
   // std::cout<<"full_constraint_relative_:"<<full_constraint_relative_.size()<<std::endl;
   // std::cout<<"*it:"<<*it<<std::endl;
  for (uint i = 0; i < full_constraint_relative_.size(); i++) //目前应当是size = 2, 即0和1,即x和y
  {
        (*y)(*it) -= alpha(i);
        //mm(*it) = footlocation(i)-alpha(i);
        
        it++;
    // if(mode_indexkp_==0 || mode_indexkp_==2)
    // {
    //   if(i==0)
    //   {
    //     //std::cout<<"(*y)(*it)0 :"<<*it <<std::endl;  
    //     (*y)(*it) += alpha(i);
    //     it++;
    //   }
    //   else if(i==1)
    //   {
    //     //std::cout<<"(*y)(*it)0 :"<<*it <<std::endl;  
    //     (*y)(*it) += alpha(i)-0.1;
    //     it++;
    //   }
    // }
    // if(mode_indexkp_==1)
    // {
    //   if(i==0)
    //   {
    //     //std::cout<<"(*y)(*it)0 :"<<*it <<std::endl;  
    //     (*y)(*it) += alpha(i);
    //     it++;
    //   }
    //   else if(i==1)
    //   {
    //     //std::cout<<"(*y)(*it)0 :"<<*it <<std::endl;  
    //     (*y)(*it) += alpha(i)+0.1;
    //     it++;
    //   }
    // }
      //std::cout<<"(*y)(*it)1 :"<<(*y)(*it) <<std::endl;  
      //std::cout <<std::endl;  
  }
    //DRAKE_DEMAND((*y)==mm);//证明这两个方式是一样的
    //std::cout<<"(*y) = "<<(*y)<<std::endl;  
    //std::cout<<"mm = "<<mm<<std::endl;   

}

///
///  KinematicVelocityConstraint
///
template <typename T>
KinematicVelocityConstraint<T>::KinematicVelocityConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description,int i_mode)
    : KinematicVelocityConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          context, description , i_mode) {}

template <typename T>
KinematicVelocityConstraint<T>::KinematicVelocityConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description,int i_mode)
    : NonlinearConstraint<T>(evaluators.count_active(),//+3
          plant.num_positions() + plant.num_velocities(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      i_mode_(i_mode) {
  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicVelocityConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const 
{
  drake::MatrixX<T> J_w(3, plant_.num_velocities());
  const drake::multibody::Frame<T>& right_crank_frame = plant_.GetFrameByName("right_crank_Link");
  const drake::multibody::Frame<T>& left_crank_frame = plant_.GetFrameByName("left_crank_Link");
  const drake::multibody::Frame<T>& world = plant_.world_frame();
  SetPositionsAndVelocitiesIfNew<T>(plant_, x, context_);
  *y=VectorXd::Zero(6);
  if(i_mode_==0||i_mode_==2)
  {
    plant_.CalcJacobianAngularVelocity(*context_, drake::multibody::JacobianWrtVariable::kV, left_crank_frame, world, world, &J_w);
  }
  else if(i_mode_==1)
  {
    plant_.CalcJacobianAngularVelocity(*context_, drake::multibody::JacobianWrtVariable::kV, right_crank_frame, world, world, &J_w);
  }
  // (* y).segment(0,3)=J_w*plant_.GetVelocities(*context_);
  // (* y).segment(3,3)= evaluators_.EvalActiveTimeDerivative(*context_);
  (* y)= evaluators_.EvalActiveTimeDerivative(*context_);
}

///
///  KinematicAccelerationConstraint
///
template <typename T>
KinematicAccelerationConstraint<T>::KinematicAccelerationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : KinematicAccelerationConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          context, description) {}

template <typename T>
KinematicAccelerationConstraint<T>::KinematicAccelerationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + plant.num_velocities() + plant.num_actuators()
              + evaluators.count_full(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators) {
  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicAccelerationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const 
{
  const auto& x = vars.head(plant_.num_positions() + plant_.num_velocities());
  const auto& u = vars.segment(plant_.num_positions() + plant_.num_velocities(),
      plant_.num_actuators());
  const auto& lambda = vars.tail(evaluators_.count_full());
  multibody::setContext<T>(plant_, x, u, context_);

  *y = evaluators_.EvalActiveSecondTimeDerivative(context_, lambda);
}

///
///  foot_location_Constraint
///
template <typename T>
foot_location_Constraint<T>::foot_location_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : foot_location_Constraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          std::set<int>(), context, description) {}

template <typename T>
foot_location_Constraint<T>::foot_location_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::set<int>& full_constraint_relative,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(full_constraint_relative.size(),
          full_constraint_relative.size()*3,
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      full_constraint_relative_(full_constraint_relative) {

  // Create a new context if one was not provided
  if (context == nullptr) {
    
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
  std::cout<<full_constraint_relative.size()<<std::endl;
}
 
template <typename T>
void foot_location_Constraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, drake::VectorX<T>* y) const {
   //std::cout<<"vars.size():"<<vars.size()<<std::endl;   
  DRAKE_DEMAND(vars.size() == 6);
  const auto&  rel0_x = vars(0);//rel_x at mode 0
  const auto&  rel1_x = vars(2);//rel_x at mode 1
  const auto&  rel2_x = vars(4);//rel_x at mode 2
  const auto&  rel0_y = vars(1);//rel_y at mode 0
  const auto&  rel1_y = vars(3);//rel_y at mode 1
  const auto&  rel2_y = vars(5);//rel_y at mode 2
  drake::VectorX<T> vector_rel0(1);
  drake::VectorX<T> vector_rel1(1);  
  drake::VectorX<T> vector_rel2(1);  
  vector_rel0(0)=rel0_x;
  vector_rel1(0)=rel1_x;
  vector_rel2(0)=rel2_x; 
  * y=VectorXd::Zero(2);

  drake::VectorX<T> X(1);
    X<<0.125;
      (*y) << 0.5*(vector_rel2-vector_rel0)+vector_rel0-vector_rel1, vector_rel1-X;
    //(*y)(1) = vector_rel1-X;
}


///
///  floatfoot_collosionavoidence_Constraint
///
template <typename T>
floatfoot_collosionavoidence_Constraint<T>::floatfoot_collosionavoidence_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description, int mode_index, int knot_index)
    : floatfoot_collosionavoidence_Constraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_full()),
          VectorXd::Zero(evaluators.count_full()),
          std::set<int>(), context, description, mode_index,knot_index) {}

template <typename T>
floatfoot_collosionavoidence_Constraint<T>::floatfoot_collosionavoidence_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::set<int>& full_constraint_relative,
    Context<T>* context, const std::string& description , int mode_index, int knot_index)
    : NonlinearConstraint<T>(2,
          plant.num_positions() + 4,
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      full_constraint_relative_(full_constraint_relative),
      mode_index_(mode_index),
      knot_index_(knot_index)
{

    // Create a new context if one was not provided
    if (context == nullptr) {
      owned_context_ = plant_.CreateDefaultContext();
      context_ = owned_context_.get();
    } else {
      context_ = context;
    }
}
 
template <typename T>
void floatfoot_collosionavoidence_Constraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, drake::VectorX<T>* y) const
{
  const auto& q = vars.head(plant_.num_positions());
  const auto& alpha = vars.tail(4);
  SetPositionsIfNew<T>(plant_, q, context_);
  const Eigen::Vector3d pt(0.25, 0, -0.31);//0.13, 0, -0.31为落足点，所以足前端为+10cm
  const Eigen::Vector3d pt_another(0.25, 0, -0.31);//以foot作为落足点30//这里需要重新设置,保证精确0.10, 0, -0.29
  VectorX<T>pt_target(3);//以foot作为落足点30//这里需要重新设置,保证精确
  VectorX<T>pt_target_another(3);//以foot作为落足点30//这里需要重新设置,保证精确
  SetPositionsIfNew<T>(plant_, q, context_);
    //   *y = evaluators_.EvalFull(*context_);
    // std::cout<<y->size()<<std::endl;
    // *y = *y + alpha;
    * y=VectorXd::Zero(2);
    const drake::multibody::Frame<T>& left_ankle = plant_.GetFrameByName("left_tarsus_Link");//left_lower_leglink_left_toe
    const drake::multibody::Frame<T>& right_ankle = plant_.GetFrameByName("right_tarsus_Link");//left_lower_leglink_left_toe
    const drake::multibody::Frame<T>& left_lower_leg = plant_.GetFrameByName("left_tarsus_Link");//left_lower_leglink_left_toe
    const auto& right_lower_leg = plant_.GetFrameByName("right_tarsus_Link");//right_lower_leglink_right_toe
      
      const drake::multibody::Frame<T>& world = plant_.world_frame();
      const Eigen::Vector3d pt_A_(pt);//(, 0, -0.31)
      Eigen::MatrixXd m(3,1);
      const Eigen::Vector3d pt_B_(pt_another);//(, 0, -0.31)
      Eigen::MatrixXd n(3,1);
      //m<<pt_A_(0),pt_A_(1),pt_A_(2);
      //std::cout<<pt_A_<<std::endl;
      m.col(0)=pt_A_;
      n.col(0)=pt_B_;
      Eigen::MatrixXd X(1,1);
      Eigen::MatrixXd Z(1,1);

      if(mode_index_==0)
      {
        plant_.CalcPointsPositions(*context_,right_lower_leg,m.template cast<T>(),world,&pt_target);
        plant_.CalcPointsPositions(*context_,left_lower_leg,n.template cast<T>(),world,&pt_target_another);
        X<<0.120;//本来应该是125，障碍物在125,0.005的余量给规划器
        Z<<0.0500;//05001互补约束验证‘稍稍大一点’也可以通过缩放接触点位置实现
      }
      else if(mode_index_==1)
      {
        plant_.CalcPointsPositions(*context_,left_lower_leg,m.template cast<T>(),world,&pt_target);
        plant_.CalcPointsPositions(*context_,right_lower_leg,n.template cast<T>(),world,&pt_target_another);
        X<<0.120;
        Z<<0.0500;//05001 互补约束验证‘稍稍大一点’也可以通过缩放接触点位置实现
      }
      else if(mode_index_==2)
      {
        plant_.CalcPointsPositions(*context_,right_lower_leg,m.template cast<T>(),world,&pt_target);
        plant_.CalcPointsPositions(*context_,left_lower_leg,n.template cast<T>(),world,&pt_target_another);
        X<<0.245;
        Z<<0.1000;//10001 互补约束验证‘稍稍大一点’也可以通过缩放接触点位置实现
      }  
      //* y = -alpha + pt_target; 
      //slack vars版
      // (* y).segment(0,3) = -alpha.segment(0,3)+ pt_target ; 
      //(* y).segment(3,1) =  -alpha.segment(3,1)+pt_target_ankle.segment(1,1) ; 

      (* y)(0)= ((pt_target(0)-X.template cast<T>()(0,0))*(pt_target(2)-Z.template cast<T>()(0,0))); //-alpha[0]
       (* y)(1) = pt_target(1);
      
      //纯lbub版本,没有slack vars
      // (* y).segment(0,1) =  pt_target.segment(0,1) ; 
      // (* y).segment(1,1) = pt_target.segment(1,1) ; 
      // (* y).segment(2,1) = pt_target.segment(2,1); 
      // (* y).segment(3,1) = pt_target_ankle.segment(1,1) ; 

}

///
///  四元数转欧拉角之后的约束
///
template <typename T>
quat2euler_constraint<T>::quat2euler_constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description, int mode_index, int knot_index)
    : quat2euler_constraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_full()),
          VectorXd::Zero(evaluators.count_full()),
           context, description, mode_index,knot_index) {}

template <typename T>
quat2euler_constraint<T>::quat2euler_constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description , int mode_index, int knot_index)
    : NonlinearConstraint<T>(3,7,
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      mode_index_(mode_index),
      knot_index_(knot_index)
{
    // Create a new context if one was not provided
    if (context == nullptr) {
      owned_context_ = plant_.CreateDefaultContext();
      context_ = owned_context_.get();
    } else {
      context_ = context;
    }
}
 
template <typename T>
void quat2euler_constraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, drake::VectorX<T>* y) const
{
  const auto& quat_vars= vars.head(4);
  const auto& alpha = vars.tail(3);//euler_angle slack
  //SetPositionsIfNew<T>(plant_, q, context_);
  Eigen::Quaternion Quaternion_state(quat_vars(0),quat_vars(1), quat_vars(2), quat_vars(3));//生成eigen 四元数
  drake::math::RollPitchYaw< T >  RollPitchYaw_state(Quaternion_state.normalized());
  VectorX<T> rpy(3);
  rpy(0)=RollPitchYaw_state.roll_angle();//.template cast<T>()
  rpy(1)=RollPitchYaw_state.pitch_angle();
  rpy(2)=RollPitchYaw_state.yaw_angle();
  DRAKE_DEMAND(rpy(0)<3.14);
  DRAKE_DEMAND(rpy(1)<3.14);
  DRAKE_DEMAND(rpy(2)<3.14);
  // std::cout<<"roll= "<<rpy(0)<<std::endl;
  // std::cout<<"pitch= "<<rpy(1)<<std::endl;
  // std::cout<<"yaw= "<<rpy(2)<<std::endl;
  // (*y)(0)= rpy(0) - alpha(0);
  // (*y)(1)= rpy(1) - alpha(1);
  // (*y)(2)= rpy(2) - alpha(2);
  (*y)=rpy;
}

///
///  足端姿态约束
///
template <typename T>
footpose_constraint<T>::footpose_constraint(
    const MultibodyPlant<T>& plant,
    Context<T>* context, const std::string& description, int mode_index, int knot_index)
    : footpose_constraint<T>(plant,
          VectorXd::Zero(3),
          VectorXd::Zero(3),
           context, description, mode_index,knot_index, 0) {}

template <typename T>
footpose_constraint<T>::footpose_constraint(
    const MultibodyPlant<T>& plant,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description , int mode_index, int knot_index, int index)
    : NonlinearConstraint<T>(3,18,
          lb, ub, description),
      plant_(plant), 
      mode_index_(mode_index),
      knot_index_(knot_index),
      constraint_index_(index)
{
    // Create a new context if one was not provided
    if (context == nullptr) {
      owned_context_ = plant_.CreateDefaultContext();
      context_ = owned_context_.get();
    } else {
      context_ = context;
    }
}
 
template <typename T>
void footpose_constraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, drake::VectorX<T>* y) const
{
  int nq = plant_.num_positions();
  const auto& world_frame = plant_.world_frame();
  const auto& right_foot_frame = plant_.GetFrameByName("right_crank_Link");
  const auto& left_foot_frame = plant_.GetFrameByName("left_crank_Link");
  const auto& q= vars.head(nq);
  const auto& alpha = vars.tail(3);//euler_angle slack w.r.t. world frame
  SetPositionsIfNew<T>(plant_, q, context_);
  VectorX<T> rpy(3);
  if(mode_index_==0 || mode_index_==2)
  {
    drake::math::RotationMatrix< T > right_foot_pose = plant_.CalcRelativeRotationMatrix(*context_,world_frame,right_foot_frame);
    drake::math::RollPitchYaw< T >RollPitchYaw_state(right_foot_pose);
    rpy(0)=RollPitchYaw_state.roll_angle();
    rpy(1)=RollPitchYaw_state.pitch_angle();
    rpy(2)=RollPitchYaw_state.yaw_angle();
  }
  else if(mode_index_==1)
  {
    drake::math::RotationMatrix< T > left_foot_pose = plant_.CalcRelativeRotationMatrix(*context_,world_frame,left_foot_frame);
    drake::math::RollPitchYaw< T >RollPitchYaw_state(left_foot_pose);
    rpy(0)=RollPitchYaw_state.roll_angle();
    rpy(1)=RollPitchYaw_state.pitch_angle();
    rpy(2)=RollPitchYaw_state.yaw_angle();
  }
  (*y)=rpy-alpha;
}


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicVelocityConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicAccelerationConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::foot_location_Constraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::floatfoot_collosionavoidence_Constraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::quat2euler_constraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::footpose_constraint)
}  // namespace multibody
}  // namespace dairlib
