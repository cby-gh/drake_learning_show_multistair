#include <ctime>
#include <string>  
#include <iostream>
#include <typeinfo> 
#include <fstream>// c++文件操作
#include <iomanip>// 设置输出格式
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h> 

//#include "periodic_input.h"
//#include "drake/multibody/multibody_utils.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
# include "multibody/multibody_utils.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "common/find_resource.h"
//#include "dairlib/systems/vector_scope.h"// direct import from /home/cby/dairlib, see details in cmakelist 
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph_inspector.h"
#include <gflags/gflags.h>
#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
//#include "drake/multibody/tree/multibody_tree.h" 
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/frame.h"

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/constraint.h>
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/math/rotation_matrix.h>
#include <drake/math/roll_pitch_yaw.h>
#include "drake/solvers/choose_best_solver.h"
//#include "common/find_resource.h"
//#include "systems/trajectory_optimization/dircon/dircon.h"
//#include "multibody/kinematic/world_point_evaluator.h"
//#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "drake/common/drake_assert.h"
#include "extensionx_trajectory/extension11_trajectory_fixed_ankle_allmotor.h"

#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
DEFINE_double(strideLength, 0.125, "The stride length.");
DEFINE_double(duration, 1.0, "The stride duration");//total time
DEFINE_bool(autodiff, false, "Double or autodiff version");
std::vector<Eigen::VectorXd> global_init_x;
std::vector<Eigen::VectorXd> global_init_u;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using std::map;
using std::string;
using std::vector;

using drake::multibody::MultibodyPlant;//释放到当前作用域
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
//using drake::math::RigidTransform;
using drake::trajectories::PiecewisePolynomial;
//using drake::systems::DiagramBuilder;
using drake::multibody::JointIndex;
using drake::multibody::FrameIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::multibody::JointActuator;
using drake::multibody::JointActuatorIndex;

void print_localtime() {
  std::time_t result = std::time(nullptr);
  std::cout << std::asctime(std::localtime(&result));
}

bool solve_result=0;
namespace dairlib {
namespace {

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;
std::vector<MatrixXd> result_x;
std::vector<MatrixXd> result_u;   
std::vector<MatrixXd> result_u_prior;   
std::vector<MatrixXd> result_x_prior;    

template <typename T>
void position_check(drake::multibody::MultibodyPlant<T> *plant, drake::systems::Context<T>* context_check,
const drake::VectorX<T> state,drake::VectorX<T> *pt_target_check,int knot_index)
{
            int mode_index_=0;
            if(knot_index<=8)
            mode_index_=0;
            if(knot_index>=10 && knot_index<=27)
            mode_index_=1;
            if(knot_index>=28 && knot_index<=37)
            mode_index_=2;
            
            //plant.SetPositions(context, q);
            context_check->SetContinuousState(state.head(15));
            const Eigen::Vector3d pt_float_contact(0.13, 0, -0.31);//以foot作为落足点30//这里需要重新设置,保证精确0.10, 0, -0.29
            drake::VectorX<T>pt_target_contact(3);//以foot作为落足点30//这里需要重新设置,保证精确
            const drake::multibody::Frame<T>& left_ankle = plant->GetFrameByName("left_tarsus_Link");//left_lower_leglink_left_toe
            const drake::multibody::Frame<T>& right_ankle = plant->GetFrameByName("right_tarsus_Link");//left_lower_leglink_left_toe
            const drake::multibody::Frame<T>& world_check = plant->world_frame();
            Eigen::MatrixXd m(3,1);
            m.col(0)=pt_float_contact;
            if(mode_index_==0)
            {
              plant->CalcPointsPositions(*context_check,right_ankle,m,world_check,&pt_target_contact);
            }
            else if(mode_index_==1)
            {
              plant->CalcPointsPositions(*context_check,left_ankle,m,world_check,&pt_target_contact);
            }
            else if(mode_index_==2)
            {
              plant->CalcPointsPositions(*context_check,right_ankle,m,world_check,&pt_target_contact);
            }  
            *pt_target_check=pt_target_contact;
}



template <typename T>
void runDircon(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    double stride_length,
    double duration,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_l_traj,
    vector<PiecewisePolynomial<double>> init_lc_traj,
    vector<PiecewisePolynomial<double>> init_vc_traj) 
 {
    auto start1 = std::chrono::high_resolution_clock::now();
    drake::systems::DiagramBuilder<double> builder;
    MultibodyPlant<T>& plant = *plant_ptr;
    //std::move means take the owner ship && AddSystem return a bare pointer.
    drake::geometry::SceneGraph<double>& scene_graph =*builder.AddSystem(std::move(scene_graph_ptr));

    std::unique_ptr<Context<T>> context = plant.CreateDefaultContext();
    drake::VectorX<T> positions = plant.GetPositions(*context, ModelInstanceIndex(2));//在multibody sysyem中MII0给world,MII1给了drake自建,后续的
    drake::VectorX<T> velocities = plant.GetVelocities(*context, ModelInstanceIndex(2));//才会给urdf引入的instance
    std::cout<<"positions.rows()"<<positions.rows()<<std::endl;//15 
    std::cout<<"positions.cols()"<<positions.cols()<<std::endl;
    std::cout<<"positions.size()"<<positions.size()<<std::endl;
    std::cout<<"velocities.rows()"<<velocities.rows()<<std::endl;//14
    std::cout<<"velocities.cols()"<<velocities.cols()<<std::endl;
    std::cout<<"velocities.size()"<<velocities.size()<<std::endl;
    std::vector<JointIndex> joint_indices0 = plant.GetJointIndices(ModelInstanceIndex(0));
    std::cout<<"Model world has "<<joint_indices0.size()<<" joints"<<std::endl;
    std::vector<JointIndex> joint_indices1 = plant.GetJointIndices(ModelInstanceIndex(1));
    std::cout<<"Model default has "<<joint_indices1.size()<<" joints"<<std::endl;
    std::vector<JointIndex> joint_indices2 = plant.GetJointIndices(ModelInstanceIndex(2));
    std::cout<<"Model robot has "<<joint_indices2.size()<<" joints"<<std::endl;
    //N.B. we have add a weld joint between base and world, so there is additional one joint, so the size is 12, 11 previous
    //can also check the position index is the same order in joint order.
    for(int ii=0; ii<joint_indices2.size();ii++)//15
    {
        const drake::multibody::Joint<T>&  joint = plant.get_joint(joint_indices2[ii]);
        std::cout<<"joint_indices2["<<ii<<"]: "<<joint_indices2[ii]<<"->"<<joint.name()<<std::endl;
        if(joint.num_positions()>0)
        std::cout<<"joint.position_start(): "<<joint.position_start()<<std::endl;
    }
    std::cout<<std::endl;
    const drake::multibody::Body<T>& floating_base = plant.GetBodyByName("base_link");//flypart
    std::cout<<floating_base.is_floating()<<std::endl;
    std::cout<<floating_base.has_quaternion_dofs()<<std::endl;
    std::cout<<"context.to_string0:"<<(*context).to_string()<<std::endl;//29
    drake::VectorX<T> test(29);
    test<<0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28;
    context->SetContinuousState(test);
    std::cout<<"to_stringto_stringto_stringto_stringto_string1:"<<(*context).to_string()<<std::endl;//29
    auto positions_map = multibody::makeNameToPositionsMap(plant);//return a std::map<std::string, int>
    auto velocities_map = multibody::makeNameToVelocitiesMap(plant);//且state顺序并没有改变
    //first = key, second = value
    std::cout<<"to_stringto_stringto_stringto_stringto_string2:"<<(*context).to_string()<<std::endl;
    for (auto const& element : positions_map)
       cout << "positions_map:" <<element.first << " = " << element.second << endl;
    std::cout<<std::endl;
    for (auto const& element : velocities_map)
       cout << "velocities_map:"  << element.first << " = " << element.second << endl;
    std::cout<<std::endl;
    
      // for(drake::multibody::FrameIndex i(0);i<plant.num_frames();i++ )
      // {
      //   std::cout<<"frame_name: "<<plant.get_frame(i).name()<<std::endl;
      // }

    //红左蓝右，这里的frame是material frame，为该body的根坐标系，也是body_frame相当于ros中的父joint坐标系的位置。
    const drake::multibody::Frame<T>& left_lower_leg = plant.GetFrameByName("left_tarsus_Link");//left_lower_leglink_left_toe
    const auto& right_lower_leg = plant.GetFrameByName("right_tarsus_Link");//right_lower_leglink_right_toe
    
    //pt_A the contact point on the body与地面接触的点
    //足端点相对于关节膝关节joint的位置/问题是初始位置还是摆动位置，这两个位置相对与膝关节是一个位置，这是关于膝关节关节坐标系讲的
    //问题2：如果是欠驱动怎么办
    //Vector3d pt(0, 0, -0.5);
    //Vector3d pt(0, 0, -0.27);
    Vector3d pt(0.13, 0, -0.31);//以foot作为落足点30//这里需要重新设置,保证精确0., 0, -0.31 
    //Vector3d pt(0, 0, 0);//以toe作为落足点
    Vector3d ones(1,0,0);
    double mu = 3;
    //左落足点相关计算类,相关运动学计算器，也蕴含了各种约束，包括了地面接触时的碰撞点的判定，即xyz位置信息。
    //但是在这一步还没有进行各种约束的选择，只是设定了被激活的方向和初始化
    //应当是计算落足点在世界坐标系下的位姿
    auto left_foot_eval = multibody::WorldPointEvaluator<T>(plant, pt,
                                                            left_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});//0 1 2对应所有xyz方向， 激活方向0 1 2 = x z
    left_foot_eval.set_frictional(); //is_frictional_ = true; 启用接触摩擦模型
    left_foot_eval.set_mu(mu);//mu_=mu=1
    // auto floatright_foot_eval_m0 = multibody::WorldPointEvaluator<T>(plant, pt,
    //                                                         right_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 2});//0 1 2对应所有xyz方向， 激活方向0 2 = x z


  auto right_foot_eval = multibody::WorldPointEvaluator<T>(plant, pt, 
                                                           right_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  right_foot_eval.set_frictional();//meaens     constraints.push_back(solvers::CreateConicFrictionConstraint(this->mu(), 2));
  right_foot_eval.set_mu(mu);
// auto floatleft_foot_eval_m1 = multibody::WorldPointEvaluator<T>(plant, pt,
//                                                             left_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 2});//0 1 2对应所有xyz方向， 激活方向0 1 2 = x y z

  auto left_foot_eval2 = multibody::WorldPointEvaluator<T>(plant, pt,
                                                        left_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});//0 1 2对应所有xyz方向， 激活方向0 1 2 = x y z
  left_foot_eval2.set_frictional(); //is_frictional_ = true; 启用接触摩擦模型
  left_foot_eval2.set_mu(mu);//mu_=mu=1  
  // auto floatright_foot_eval_m2 = multibody::WorldPointEvaluator<T>(plant, pt,
  //                                                         right_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 2});//0 1 2对应所有xyz方向， 激活方向0 2 = x z

  auto evaluators_left = multibody::KinematicEvaluatorSet<T>(plant);//初始化一个运动学(约束)集合
  int left_foot_eval_index =  evaluators_left.add_evaluator(&left_foot_eval);//加入左脚的运动学计算类，并返回当前运动学计算器在vector中的索引
  cout << "left_foot_eval_index: " << " = " << left_foot_eval_index << endl;//左足端点计算器的索引 0 
  // int floatright_foot_eval_m0_index =  evaluators_left.add_evaluator(&floatright_foot_eval_m0);//加入左脚的运动学计算类，并返回当前运动学计算器在vector中的索引
  // cout << "floatright_foot_eval_m0_index: " << " = " << floatright_foot_eval_m0_index << endl;//左足端点计算器的索引 0 

  auto evaluators_right = multibody::KinematicEvaluatorSet<T>(plant);
  int right_foot_eval_index =  evaluators_right.add_evaluator(&right_foot_eval);
  cout << "right_foot_eval_index: " << " = " << right_foot_eval_index << endl;//右足端点计算器的索引 0 
  // int floatleft_foot_eval_m1_index =  evaluators_right.add_evaluator(&floatleft_foot_eval_m1);//加入左脚的运动学计算类，并返回当前运动学计算器在vector中的索引
  // cout << "floatleft_foot_eval_m1_index: " << " = " << floatleft_foot_eval_m1_index << endl;//左足端点计算器的索引 0 


  auto evaluators_left2 = multibody::KinematicEvaluatorSet<T>(plant);//初始化一个运动学(约束)集合
  int left_foot_eval_index2 =  evaluators_left2.add_evaluator(&left_foot_eval2);//加入左脚的运动学计算类，并返回当前运动学计算器在vector中的索引
  cout << "left_foot_eval_index2: " << " = " << left_foot_eval_index2 << endl;//左足端点计算器的索引 0 
  // int floatright_foot_eval_m2_index =  evaluators_left2.add_evaluator(&floatright_foot_eval_m2);//加入左脚的运动学计算类，并返回当前运动学计算器在vector中的索引
  // cout << "floatright_foot_eval_m2_index: " << " = " << floatright_foot_eval_m2_index << endl;//左足端点计算器的索引 0 


  int num_knotpoints_mode0 = 10;
   int num_knotpoints_mode1 = 20;
   int num_knotpoints_mode2 = 10;
 //double min_T = .1;//The minimum total duration of the mode (default 0)
  //double max_T = 3;//The maximum total duration of the mode (default 0)
  double  min_T_mode0 = 0.25;//在这里直接修改左右相时间相等或者在类里添加时间相等约束
  double max_T_mode0 = 0.25;//
  double  min_T_mode1 = 0.5;//在这里直接修改左右相时间相等或者在类里添加时间相等约束
  double max_T_mode1 = 0.5;//
  double  min_T_mode2 = 0.25;//在这里直接修改左右相时间相等或者在类里添加时间相等约束
  double max_T_mode2 = 0.25;//
//Each DirconMode object refers to a single hybrid mode of the optimization  
  auto mode_left = DirconMode<T>(evaluators_left, num_knotpoints_mode0, min_T_mode0, max_T_mode0);//定义dircon的左腿mode,该初始化仅定义了一些成员变量，没有额外的操作
  mode_left.MakeConstraintRelative(0, 0);  
  mode_left.MakeConstraintRelative(0, 1); // x-coordinate在特定的腿的运动学计算器中的激活的方向选中更加激活的方向(relative direction)，x方向足端要向
  //前前进rel_offset的具体,所以x方向比active方向更特殊,标记为relative, 第一个arg为运动学计算器,实际上就是计算第一个约束,第二个为该约束里比active
  //更特殊的方向
  auto mode_right = DirconMode<T>(evaluators_right, num_knotpoints_mode1, min_T_mode1, max_T_mode1);//定义dircon的右腿mode
  mode_right.MakeConstraintRelative(0, 0);
  mode_right.MakeConstraintRelative(0, 1);

  auto mode_left2 = DirconMode<T>(evaluators_left2, num_knotpoints_mode2, min_T_mode2, max_T_mode2);//定义dircon的右腿mode
  mode_left2.MakeConstraintRelative(0, 0);//Relative constraints 针对的是mode
  mode_left2.MakeConstraintRelative(0, 1);//其含义是落足点的三维位置坐标中的xy可以不为零,在DIRCON中按照mode声明relative vars
    
    //左足支撑相(先迈右脚，蓝色)和右足支撑相
  auto sequence = DirconModeSequence<T>(plant);//初始化一个空的序列
  sequence.AddMode(&mode_left);//在表示序列的std::vector中插入指向mode的指针
  sequence.AddMode(&mode_right);
  sequence.AddMode(&mode_left2);

  auto trajopt = Dircon<T>(sequence);  
  std::cout<< "trajopt.num_vars()after create dircon: "<<  trajopt.num_vars()<<std::endl;//1982
  std::cout<< "trajopt.GetAllConstraints().size() after create dircon:"<<  trajopt.GetAllConstraints().size()<<std::endl;//414
  std::cout<< "trajopt.GetAllCosts().size() after create dircon:"<<  trajopt.GetAllCosts().size()<<std::endl;//12
//Adds a duration constraint on the total duration of the trajectory. 
 trajopt.AddDurationBounds(duration, duration);//duration = 1.0s for 2 mode duration of the trajectory. 
  
//set solver
   trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),"Print file", "/home/cby/drake_learning/snopt.out");
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),"Major iterations limit", 400);//200
  int max_iter = 100000;
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major iterations limit", max_iter);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),"Iterations limit", 100000);  // QP subproblems
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",0); 
  // double tol =1e-3;
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),"Major feasibility tolerance", 10*tol);  // target complementarity gap
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major optimality tolerance", 10*tol);  // target nonlinear constraint violation
  //  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),"Minor feasibility tolerance", 10*tol);  // target complementarity gap

  // std::cout << "\nChoose the best solver: "<< drake::solvers::ChooseBestSolver(trajopt).name() << std::endl;
  // std::cout<< " drake::solvers::SnoptSolver::id().name()"<< drake::solvers::SnoptSolver::id().name()<<std::endl; now is SNOPT
  //Set the initial guess for the trajectory decision variables. 
  for (int j = 0; j < sequence.num_modes(); j++) //进行到初始值设置
  {
    cout << "num_modes:" << " = " << sequence.num_modes() << endl;//当前的modes数目 3
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::SetInitialTrajectory(init_u_traj, init_x_traj);//有了逆运动规划,暂时不用轨迹进行初始化,直接初始化knots
    //且验证了SetInitialTrajectory与SetInitialGuess几乎一样,只有最后一个不一样,trajectory没有
       /// Set the initial guess for the force variables for a specific mode
        /// traj_init_l contact forces lambda (interpreted at knot points)
         ///traj_init_lc contact forces (interpreted at collocation points)
    // traj_init_vc velocity constraint slack variables (at collocation)
    trajopt.SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j], init_vc_traj[j]);//0.054
    cout << "trajopt.SetInitialForceTrajectory rows:" << " = " << init_l_traj[j].rows() << endl;//输出的矩阵形式
    cout << "trajopt.SetInitialForceTrajectory cols:" << " = " << init_l_traj[j].cols() << endl;//3*1
  }
  for(int i=0;i<38;i++)
  {
    auto xi = trajopt.state(i);
    auto ui = trajopt.input(i);
    cout <<  trajopt.GetInitialGuess(xi)<<endl;
    cout <<  trajopt.GetInitialGuess(ui)<<endl;
  }
  // for (int i = 0; i <  38; i++) 
  // {
  //   auto xi = trajopt.state(i);
  //   auto ui = trajopt.input(i);
  //   trajopt.SetInitialGuess(xi, global_init_x[i]);
  //   trajopt.SetInitialGuess(ui, global_init_u[i]);
  // }
  // for(int i=0;i<38;i++)
  // {
  //   auto xi = trajopt.state(i);
  //   auto ui = trajopt.input(i);
  //   cout <<  trajopt.GetInitialGuess(xi)<<endl;
  //   cout <<  trajopt.GetInitialGuess(ui)<<endl;
  // }
// Periodicity constraints
// la = 7
// ra = 6
// lk = 5
// rk = 4
// lh = 3
// rh = 2
// planar_x = 0
// planar_z = 1
  // auto rel_vars0 = trajopt.offset_vars(0);
  // auto rel_vars1 = trajopt.offset_vars(1);
  // auto rel_vars2 = trajopt.offset_vars(2);
  // trajopt.AddLinearConstraint(rel_vars0 <=  rel_vars1);
  // trajopt.AddLinearConstraint(rel_vars1 <=  rel_vars2);
    //标准约束原始约束    
   auto x0 = trajopt.initial_state();//mode0开始state
   auto xf = trajopt.final_state();//mode2结束state
   auto x_exact=trajopt.state();//state(index) 每一个state在轨迹上
   auto x = trajopt.state();
   int nq = plant.num_positions();
  
  //constraints includes periodic constraints, joint limit constrints, bodypart limit constraints, middle constrints, algorithm constriants
  //maybe includes input constraints, obstacle constrints  and other constriants
  //Periodic constraints include Floating base constraints and joint constraints
  //Floating base position periodicity::begining and ending
  //1.floating base  quaternion constraints
  trajopt.AddLinearConstraint(x0(positions_map["base_qw"]) == xf(positions_map["base_qw"]));
  trajopt.AddLinearConstraint(x0(positions_map["base_qx"]) == xf(positions_map["base_qx"]));
  trajopt.AddLinearConstraint(x0(positions_map["base_qy"]) == xf(positions_map["base_qy"]));
  trajopt.AddLinearConstraint(x0(positions_map["base_qz"]) == xf(positions_map["base_qz"]));

  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_qx"]) <= 0.05);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_qx"]) >= -0.05);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_qy"]) <= 0.05);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_qy"]) >= -0.05);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_qz"]) <= 0.1);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_qz"]) >= -0.1);

  // trajopt.AddLinearConstraint(x0(positions_map["base_qw"]) == 1.0);
  //2. floating base translational constriants 
  trajopt.AddLinearConstraint(x0(positions_map["base_x"]) == 0);//用buonding constriants
   trajopt.AddLinearConstraint(xf(positions_map["base_x"]) >= 2.0*stride_length);//2.0*stride_length
   trajopt.AddLinearConstraint(xf(positions_map["base_x"]) <= 3.0*stride_length);//2.0*stride_length
  //trajopt.AddLinearConstraint(xf(positions_map["base_x"]) == 3.0*stride_length);//2.0*stride_length
  trajopt.AddLinearConstraint(x0(positions_map["base_y"]) ==  xf(positions_map["base_y"]));
  trajopt.AddLinearConstraint(x0(positions_map["base_z"]) + 0.01<= xf(positions_map["base_z"]));
  //trajopt.AddLinearConstraint(x0(positions_map["base_y"]) == 0);//用buonding constriants
  //Floating base velocity periodicity
  //1.floating base  angular velocity constraints
  trajopt.AddLinearConstraint(x0(nq + velocities_map["base_wx"]) ==xf(nq + velocities_map["base_wx"]));
  trajopt.AddLinearConstraint(x0(nq + velocities_map["base_wy"]) ==xf(nq + velocities_map["base_wy"]));
  trajopt.AddLinearConstraint(x0(nq + velocities_map["base_wz"]) ==xf(nq + velocities_map["base_wz"]));
  //2.floating base  translational velocity constriants 
  trajopt.AddLinearConstraint(x0(nq + velocities_map["base_vx"]) ==xf(nq + velocities_map["base_vx"]));
  trajopt.AddLinearConstraint(x0(nq + velocities_map["base_vy"]) ==xf(nq + velocities_map["base_vy"]));
  trajopt.AddLinearConstraint(x0(nq + velocities_map["base_vz"]) ==xf(nq + velocities_map["base_vz"]));

  //periodic  joint position constraints::begin-end
  // trajopt.AddLinearConstraint(x0(positions_map["joint_left_hip"]) ==0.0);
  // trajopt.AddLinearConstraint(x0(positions_map["joint_right_hip"]) ==0.0);
  // trajopt.AddLinearConstraint(x0(positions_map["joint_left_knee"]) ==0.0);
  // trajopt.AddLinearConstraint(x0(positions_map["joint_right_knee"]) ==0.0);
  // trajopt.AddLinearConstraint(xf(positions_map["joint_right_hip"]) ==0.0);  
  // trajopt.AddLinearConstraint(xf(positions_map["joint_left_hip"]) ==0.0);
  // trajopt.AddLinearConstraint(xf(positions_map["joint_right_knee"]) ==0.0);
  // trajopt.AddLinearConstraint(xf(positions_map["joint_left_knee"]) ==0.0);
  // trajopt.AddLinearConstraint(x0(positions_map["joint_left_rollmotor_yawmotor"])==0.0);
  // trajopt.AddLinearConstraint(x0(positions_map["joint_right_rollmotor_yawmotor"])==0.0);
  //trajopt.AddLinearConstraint(x0(positions_map["joint_left_yawmotor_pitchmotor"])==0.0);
  //trajopt.AddLinearConstraint(x0(positions_map["joint_right_yawmotor_pitchmotor"])==0.0);

  trajopt.AddLinearConstraint(x0(positions_map["joint_left_rollmotor_yawmotor"]) ==xf(positions_map["joint_left_rollmotor_yawmotor"]) );
  trajopt.AddLinearConstraint(x0(positions_map["joint_left_yawmotor_pitchmotor"]) ==xf(positions_map["joint_left_yawmotor_pitchmotor"]) );
  trajopt.AddLinearConstraint(x0(positions_map["joint_left_pitchmotor_thigh"]) ==xf(positions_map["joint_left_pitchmotor_thigh"]) );
  trajopt.AddLinearConstraint(x0(positions_map["joint_left_knee"]) ==xf(positions_map["joint_left_knee"]));
  trajopt.AddLinearConstraint(x0(positions_map["joint_right_rollmotor_yawmotor"]) ==xf(positions_map["joint_right_rollmotor_yawmotor"]));
  trajopt.AddLinearConstraint(x0(positions_map["joint_right_yawmotor_pitchmotor"]) ==xf(positions_map["joint_right_yawmotor_pitchmotor"]));
  trajopt.AddLinearConstraint(x0(positions_map["joint_right_pitchmotor_thigh"]) ==xf(positions_map["joint_right_pitchmotor_thigh"])); 
  trajopt.AddLinearConstraint(x0(positions_map["joint_right_knee"]) ==xf(positions_map["joint_right_knee"]));

  trajopt.AddLinearConstraint(xf(positions_map["joint_left_rollmotor_yawmotor"]) >=-0.1 );
  trajopt.AddLinearConstraint(xf(positions_map["joint_left_rollmotor_yawmotor"]) <=0.1 );
  trajopt.AddLinearConstraint(xf(positions_map["joint_right_rollmotor_yawmotor"]) >=-0.1 );
  trajopt.AddLinearConstraint(xf(positions_map["joint_right_rollmotor_yawmotor"]) <=0.1);


  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_knee"]) >= -0.8);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_knee"]) <= 0.8);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_knee"]) >= -0.8);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_knee"]) <= 0.8);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_pitchmotor_thigh"]) >= -0.4);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_pitchmotor_thigh"]) <= 0.8); 
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_pitchmotor_thigh"]) >= -0.4);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_pitchmotor_thigh"]) <= 0.8);  
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_rollmotor_yawmotor"]) >= -0.1);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_rollmotor_yawmotor"]) <= 0.1); 
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_yawmotor_pitchmotor"]) >= -0.1);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_yawmotor_pitchmotor"]) <= 0.1);  
  // trajopt.AddLinearConstraint(x0(positions_map["joint_left_rollmotor_yawmotor"]) ==xf(positions_map["joint_right_rollmotor_yawmotor"]) );
  // trajopt.AddLinearConstraint(x0(positions_map["joint_right_rollmotor_yawmotor"]) ==xf(positions_map["joint_left_rollmotor_yawmotor"]) );
  // trajopt.AddLinearConstraint(x0(positions_map["joint_left_yawmotor_pitchmotor"]) ==xf(positions_map["joint_right_yawmotor_pitchmotor"]) );
  // trajopt.AddLinearConstraint(x0(positions_map["joint_right_yawmotor_pitchmotor"]) ==xf(positions_map["joint_left_yawmotor_pitchmotor"]) );
  //periodic  joint velocity constraints::begin-end
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_rollmotor_yawmotordot"])  ==xf(nq + velocities_map["joint_left_rollmotor_yawmotordot"]) );
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_yawmotor_pitchmotordot"])  ==xf(nq + velocities_map["joint_left_yawmotor_pitchmotordot"]) );
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_pitchmotor_thighdot"])  ==xf(nq + velocities_map["joint_left_pitchmotor_thighdot"]) );
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_kneedot"])  ==xf(nq + velocities_map["joint_left_kneedot"]));
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_rollmotor_yawmotordot"])  ==xf(nq + velocities_map["joint_right_rollmotor_yawmotordot"]) );
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_yawmotor_pitchmotordot"])  ==xf(nq + velocities_map["joint_right_yawmotor_pitchmotordot"]) );
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_pitchmotor_thighdot"])  ==xf(nq + velocities_map["joint_right_pitchmotor_thighdot"]) );
  trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_kneedot"])  == xf(nq + velocities_map["joint_right_kneedot"]) );
  
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_rollmotor_yawmotordot"])  ==-xf(nq + velocities_map["joint_right_rollmotor_yawmotordot"]) );
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_rollmotor_yawmotordot"])  ==-xf(nq + velocities_map["joint_left_rollmotor_yawmotordot"]) );
  // //trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_yawmotor_pitchmotordot"])  ==-xf(nq + velocities_map["joint_left_yawmotor_pitchmotordot"]) );
  // //trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_yawmotor_pitchmotordot"])  ==-xf(nq + velocities_map["joint_right_yawmotor_pitchmotordot"]) );

  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_pitchmotor_thighdot"])  ==xf(nq + velocities_map["joint_right_pitchmotor_thighdot"]) );
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_pitchmotor_thighdot"])  ==xf(nq + velocities_map["joint_left_pitchmotor_thighdot"]) );
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_kneedot"])  ==xf(nq + velocities_map["joint_right_kneedot"]));
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_kneedot"])  == xf(nq + velocities_map["joint_left_kneedot"]) ); 
  // //joint limit constrints
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_rollmotor_yawmotor"]) >= 0.00);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_rollmotor_yawmotor"]) <= 1.5);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_rollmotor_yawmotor"]) >= -1.5);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_rollmotor_yawmotor"]) <= 0.00); 
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_yawmotor_pitchmotor"]) == 0.0);
  //trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_yawmotor_pitchmotor"]) <= 0.5);  




  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_rollmotor_yawmotor"]) >= -1.5);
  //trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_rollmotor_yawmotor"]) <= -0.01);
   trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_yawmotor_pitchmotor"]) == 0.0);
   //trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_yawmotor_pitchmotor"]) <= 0.5);
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_yawmotor_pitchmotor"]) <= 1.5);  


  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_rollmotor_yawmotor"]) >=-0.2 );
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_rollmotor_yawmotor"]) <=0.2 );
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_rollmotor_yawmotor"]) >=-0.1 );
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_rollmotor_yawmotor"]) <=0.1 );



  // trajopt.AddLinearConstraint(trajopt.state_vars(0,0)[7] ==-trajopt.state_vars(1,10)[8]);
  // trajopt.AddLinearConstraint(trajopt.state_vars(0,0)[11] ==trajopt.state_vars(1,10)[12]);
  // trajopt.AddLinearConstraint(trajopt.state_vars(0,0)[13] ==trajopt.state_vars(1,10)[14]);





  //  //bodypart limit constraints
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_x"])  >= 0);  
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_y"])  >= -0.2);  
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_y"])  <= 0.2);  
  // trajopt.AddConstraintToAllKnotPoints(x(positions_map["base_z"])  >= 0.5);  
  // //trajopt.AddConstraintToAllKnotPoints(x(nq + velocities_map["base_vx"])  >= 0);
  // //other bodypart constraints:like floating base quaternion constraints and foot constraints is included in algorithm.

  // //input constriants  
  // std::cout<<"sequence.count_knotpoints():"<< sequence.count_knotpoints()<<std::endl;
  // for (int i = 0; i < sequence.count_knotpoints(); i++) 
  // {
  //   auto ui = trajopt.input(i);
  //   trajopt.AddBoundingBoxConstraint(VectorXd::Constant(plant.num_actuators(), -300),VectorXd::Constant(plant.num_actuators(), +300), ui);
  // }


  // // floating base constraints middle point
  // trajopt.AddLinearConstraint(x0(positions_map["base_qw"]) == trajopt.state_vars(1,9)[0]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_qx"]) == -trajopt.state_vars(1,9)[1]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_qy"]) ==trajopt.state_vars(1,9)[2]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_qz"]) == -trajopt.state_vars(1,9)[3]);
  // // trajopt.AddLinearConstraint(x0(nq + velocities_map["base_wx"]) ==trajopt.state_vars(1,9)[15]);正负号有待考察
  // // trajopt.AddLinearConstraint(x0(nq + velocities_map["base_wy"]) ==-trajopt.state_vars(1,9)[16]);
  // // trajopt.AddLinearConstraint(x0(nq + velocities_map["base_wz"]) ==trajopt.state_vars(1,9)[17]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_y"]) == -trajopt.state_vars(1,9)[5]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_z"]) == trajopt.state_vars(1,9)[6]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_vx"]) ==trajopt.state_vars(1,9)[18]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_vy"]) ==-trajopt.state_vars(1,9)[19]);
  // trajopt.AddLinearConstraint(x0(positions_map["base_vz"]) == trajopt.state_vars(1,9)[20]);
  // trajopt.AddLinearConstraint(trajopt.state_vars(0,9)[4] ==stride_length);
  // trajopt.AddLinearConstraint(trajopt.state_vars(1,9)[4] ==2*stride_length);
  // trajopt.AddLinearConstraint(trajopt.state_vars(1,19)[4] ==3*stride_length);//4*stride_length即为终点约束,之前加过了

  // // joint constraints middle point
  // //注意中点处的非对称关节,下述
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_rollmotor_yawmotor"])  ==  -trajopt.state_vars(1,9)[7]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_yawmotor_pitchmotor"])  ==  -trajopt.state_vars(1,9)[9]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_rollmotor_yawmotor"])  ==  -trajopt.state_vars(1,9)[8]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_yawmotor_pitchmotor"])  ==  -trajopt.state_vars(1,9)[10]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_rollmotor_yawmotordot"])  ==  -trajopt.state_vars(1,9)[21]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_yawmotor_pitchmotordot"])  ==  -trajopt.state_vars(1,9)[23]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_rollmotor_yawmotordot"])  ==  -trajopt.state_vars(1,9)[22]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_yawmotor_pitchmotordot"])  ==  -trajopt.state_vars(1,9)[24]);
  // //注意中点处的非对称关节,上述
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_pitchmotor_thigh"])  ==  trajopt.state_vars(1,9)[11]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_pitchmotor_thigh"])  ==  trajopt.state_vars(1,9)[12]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_knee"])  == trajopt.state_vars(1,9)[13] );
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_knee"])  == trajopt.state_vars(1,9)[14] );

  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_pitchmotor_thighdot"])  ==  trajopt.state_vars(1,9)[25]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_pitchmotor_thighdot"])  ==  trajopt.state_vars(1,9)[26]);
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_kneedot"])  == trajopt.state_vars(1,9)[27] );
  // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_kneedot"])  == trajopt.state_vars(1,9)[28] );

  //   //constraint on initial floating base
  //   trajopt.AddConstraint(x0(0) == 1);

  for (int i = 0; i <  38; i++) 
  {
    auto xi = trajopt.state(i);
    if ((trajopt.GetInitialGuess(xi.head(4)).norm() == 0) ||
        std::isnan(trajopt.GetInitialGuess(xi.head(4)).norm())) {
      trajopt.SetInitialGuess(xi(0), 1);
      trajopt.SetInitialGuess(xi(1), 0);
      trajopt.SetInitialGuess(xi(2), 0);
      trajopt.SetInitialGuess(xi(3), 0);
    }
  }


// //加入了三维,这里失效了
// // planarwalker trajopt.force_vars(0, i) 是3 对应lambdax lambday lambdax 
//  for (int i = 0; i < 10; i++) {
// //force_vars是每个knot point处的力变量，设置每个knot处的y方向的约束力为0，因为只有平面   
//     trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(0, i)(1));
//     //cout << "trajopt.force_vars(0, i)(1):" << " = " << trajopt.force_vars(0, i).size() << endl;
//     trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(1, i)(1));
//     trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(1, 10+i)(1));
//     //cout << "trajopt.force_vars(1, i)(1):" << " = " << trajopt.force_vars(1, i).size() << endl;//右足端点计算器的索引 0 
//     trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(2, i)(1));
//     //cout << "trajopt.force_vars(0, i)(1):" << " = " << trajopt.force_vars(2, i).size() << endl;
//   }
  auto u = trajopt.input();
  drake::solvers::VectorXDecisionVariable x_left_hip(38);
  drake::solvers::VectorXDecisionVariable x_dot_left_hip(38);
  drake::solvers::VectorXDecisionVariable x_right_hip(38);
  drake::solvers::VectorXDecisionVariable x_dot_right_hip(38);
  drake::solvers::VectorXDecisionVariable x_left_knee(38);
  drake::solvers::VectorXDecisionVariable x_dot_left_knee(38);
  drake::solvers::VectorXDecisionVariable x_right_knee(38);
  drake::solvers::VectorXDecisionVariable x_dot_right_knee(38);
  drake::solvers::VectorXDecisionVariable x_right_ankle(38);
  drake::solvers::VectorXDecisionVariable x_dot_right_ankle(38);
  drake::solvers::VectorXDecisionVariable x_left_ankle(38);
  drake::solvers::VectorXDecisionVariable x_dot_left_ankle(38);
  for(int i=0;i<38;i++)
  {
    x_left_hip[i]=trajopt.state(i)[8];//x_vars()是project的,不能在class instace中使用
    x_right_hip[i]=trajopt.state(i)[7];//x_vars()是project的,不能在class instace中使用
    x_left_knee[i]=trajopt.state(i)[10];//x_vars()是project的,不能在class instace中使用
    x_right_knee[i]=trajopt.state(i)[9];//x_vars()是project的,不能在class instace中使用
    x_dot_left_hip[i]=trajopt.state(i)[18];//x_vars()是project的,不能在class instace中使用
    x_dot_right_hip[i]=trajopt.state(i)[17];//x_vars()是project的,不能在class instace中使用
    x_dot_left_knee[i]=trajopt.state(i)[20];//x_vars()是project的,不能在class instace中使用
    x_dot_right_knee[i]=trajopt.state(i)[19];//x_vars()是project的,不能在class instace中使用

  }
  const int n_v=14;
  const int n_q=15;
  const double w_q_hip_roll  = 8.0;
  const double w_q_hip_yaw  = 4.0;
  const double w_q_hip_pitch  = 1.0;
  const double w_q_knee=1.0;
  const double w_q_quat=3.0;
  const double w_lambda_diff=1.0;
  const double w_u_diff=1.0; 
  const double w_v_diff=1.0; 
  const double w_q_quatdot=3.0;
  const double w_q_hip_rolldot = 4.0;
  const double w_q_hip_yawdot = 4.0;
  const double R = 1;  // Cost on input 
  const double Q = 2;  // Cost on total velocity
  const double O = 2;  // Cost on total position 
  const double N = 3;  // quaterion



  const double L = 10;  // x_left_hip
  const double M = 8;  // x_right_hip
  const double C_LA = 1;  // left ankle
  const double C_RA = 1;  // right ankle

  const double P = 8;  // x_dot_left_hip
  const double S = 8;  // x_dot_right_hip
  const double V = 2;  // x_dot_left_knee
  const double U = 2;  // x_dot_right_knee
  const double C_LA_dot = 1;  // left ankle_dot
  const double C_RA_dot = 1;  // right ankle_dot
  //添加手工代价
  //基础
  trajopt.AddRunningCost(u.transpose()*R*u);
  trajopt.AddRunningCost(x.segment(1,14).transpose()*O*x.segment(1,14));
  trajopt.AddRunningCost(x.segment(15,14).transpose()*Q*x.segment(15,14));
  trajopt.AddRunningCost(x.segment(1,3).transpose()*N*x.segment(1,3));
  // trajopt.AddRunningCost(x(23)*V*x(23));
  // trajopt.AddRunningCost(x(24)*V*x(24));
  //强调
  
  //   //add cost on force difference wrt time
    // for (int i = 0; i <9; i++) 
    // {
    //   auto lambda0 = trajopt.force_vars(0, i);
    //   auto lambda1 = trajopt.force_vars(0, i + 1);
    //   trajopt.AddCost(w_lambda_diff *(lambda0 - lambda1).dot(lambda0 - lambda1));
    // }
    // for (int i = 0; i <19; i++) 
    // {
    //   auto lambda0 = trajopt.force_vars(1, i);
    //   auto lambda1 = trajopt.force_vars(1, i + 1);
    //   trajopt.AddCost(w_lambda_diff *(lambda0 - lambda1).dot(lambda0 - lambda1));
    // }
    // for (int i = 0; i <9; i++) 
    // {
    //   auto lambda0 = trajopt.force_vars(2, i);
    //   auto lambda1 = trajopt.force_vars(2, i + 1);
    //   trajopt.AddCost(w_lambda_diff *(lambda0 - lambda1).dot(lambda0 - lambda1));
    // }
    // add cost on vel difference wrt time
    for (int i = 0; i < 37; i++) 
    {
      auto v0 = trajopt.state(i).tail(n_v);
      auto v1 = trajopt.state(i + 1).tail(n_v);
      trajopt.AddCost(w_v_diff*(v0 - v1).dot(v0 - v1));
    }
    // add cost on input difference wrt time
    if (w_u_diff) 
    {
      for (int i = 0; i < 37; i++) 
      {
        auto u0 = trajopt.input(i);
        auto u1 = trajopt.input(i + 1);
        trajopt.AddCost(w_u_diff * (u0 - u1).dot(u0 - u1));
      }
    }

  // int w_pitch_knee=100;
  // for (int i = 0; i < 19; i++) 
  // {
  //   if(i==18)
  //   {
  //   auto leftpitch = trajopt.state(i).segment(11, 1);
  //   auto rightpitch = trajopt.state(i).segment(12, 1);
  //   auto leftknee = trajopt.state(i).segment(13, 1);
  //   auto rightknee = trajopt.state(i).segment(14, 1);
  //   trajopt.AddCost((leftpitch- rightpitch).dot(leftpitch- rightpitch) *w_pitch_knee );
  //   trajopt.AddCost((leftknee- rightknee).dot(leftknee- rightknee) * w_pitch_knee);
  //   }
  // }


  // //w_q_hip_roll cost
  for (int i = 0; i < 38; i++) 
  {
    auto q = trajopt.state(i).segment(7, 2);
    trajopt.AddCost(w_q_hip_roll * q.transpose() * q);
  }
  //w_q_hip_yaw cost
  for (int i = 0; i < 38; i++) 
  {
    auto q = trajopt.state(i).segment(9, 2);
    trajopt.AddCost(w_q_hip_yaw * q.transpose() * q);
  }  

    //w_q_hip_roll dot  cost
  for (int i = 0; i < 38; i++) 
  {
    auto q = trajopt.state(i).segment(21, 2);
    trajopt.AddCost(w_q_hip_rolldot * q.transpose() * q);
  }
  //w_q_hip_yaw dot  cost
  for (int i = 0; i < 38; i++) 
  {
    auto q = trajopt.state(i).segment(23, 2);
    trajopt.AddCost(w_q_hip_yawdot * q.transpose() * q);
  }

  //   //w_q_hip_pitch cost
  // for (int i = 0; i < 38; i++) 
  // {
  //   auto q = trajopt.state(i).segment(11, 2);
  //   trajopt.AddCost(w_q_hip_pitch * q.transpose() * q);
  // }



  //   //w_q_knee cost
  // for (int i = 0; i < 38; i++) 
  // {
  //   auto q = trajopt.state(i).segment(13, 2);
  //   trajopt.AddCost(w_q_knee * q.transpose() * q);
  // }
  //w_q_quat cost
    for (int i = 0; i < 38; i++) 
  {
    auto q = trajopt.state(i).segment(1, 3);
    trajopt.AddCost(w_q_quat * q.transpose() * q);
  }

  //w_q_quatdot cost
    for (int i = 0; i < 38; i++) 
  {
    auto q = trajopt.state(i).segment(15, 3);
    trajopt.AddCost(w_q_quatdot * q.transpose() * q);
  }


  std::cout<< "trajopt.num_vars()after manually setting: "<<  trajopt.num_vars()<<std::endl;//??
  std::cout<< "trajopt.GetAllConstraints().size() after manually settingss:"<<  trajopt.GetAllConstraints().size()<<std::endl;//??
  std::cout<< "trajopt.GetAllCosts().size() after manually setting:"<<  trajopt.GetAllCosts().size()<<std::endl;//??   
  std::vector<unsigned int> visualizer_poses;
  visualizer_poses.push_back(3);
  visualizer_poses.push_back(3);
  visualizer_poses.push_back(3);
//std::cout << "222" << std::endl;//finding_path2 <<std::endl;//drake/examples/PlanarWalker/PlanarWalker2.urdf
  trajopt.CreateVisualizationCallback(
      drake::FindResourceOrThrow("drake/examples/v10/threeD_v2_flaypart_show_multistair.urdf"),//e
      visualizer_poses, 0.2);//something happen here as i can't use FindResourceOrThrow from dairlib, please check here
  //std::cout << "\nChoose the best solver: " << drake::solvers::ChooseBestSolver(trajopt).name() << std::endl;
  //std::cout << "Solver: " << result.get_solver_id().name() << "\n\n"std::endl;
  std::cout<<"trajopt.GetAllConstraints().size():"<<trajopt.GetAllConstraints().size()<<std::endl;
  std::cout<<"trajopt.decision_variables().size():"<<trajopt.decision_variables().size()<<std::endl;
  auto finish1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_parse = finish1 - start1;//求解时长
  std::cout << "Parse1 time:" << elapsed_parse.count() <<std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  //std::cout<<"在solve()函数中具体实施某些动力学约束的实现"<<std::endl;
    std::cout<<"print_localtime"<<std::endl;
    print_localtime();
    std::cout<<"trajopt.initial_guess().size(): "<<trajopt.initial_guess().size()<<std::endl;
  const auto result = Solve(trajopt, trajopt.initial_guess());//规划器求解结果 trajopt.initial_guess():Getter for the initial guess. 
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;//求解时长
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  std::cout << "herehere"  <<std::endl;
  std::cout <<"result.is_success():"<<result.is_success()<<std::endl;
  std::cout << "result.get_solver_id().name()"<<result.get_solver_id().name()<<std::endl;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;  
  // visualizer
    //GetStateAndDerivativeSamples这个函数存在维度错误
  
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =trajopt.ReconstructStateTrajectory(result);
  const drake::trajectories::PiecewisePolynomial<double> pp_utraj =trajopt.ReconstructInputTrajectory(result);
  for(int i=0;i<trajopt.decision_variables().size();i++)
  {
    std::cout<<"index: "<<i<<std::endl;
    std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(i)<<std::endl;
    std::cout<<trajopt.decision_variable(i)<<"="<<result.GetSolution()[i]<<std::endl;
  }
      std::cout<<"trajopt.decision_variable(4): "<<trajopt.decision_variable(4)<<std::endl;
    std::cout<<trajopt.decision_variable(4)<<"="<<result.GetSolution()[4]<<std::endl;
        std::cout<<"trajopt.decision_variable(1071): "<<trajopt.decision_variable(1071)<<std::endl;
    std::cout<<trajopt.decision_variable(1071)<<"="<<result.GetSolution()[1071]<<std::endl;


  double max_u=0.0;
  for(int i = 1102;i<1102+304;i++)
  {
    if(max_u<std::abs(result.GetSolution()[i]))
    max_u=std::abs(result.GetSolution()[i]);
    std::cout<<trajopt.decision_variable(i)<<"="<<result.GetSolution()[i]<<std::endl;
  }
  double max_left_hip_roll; 
  double max_left_hip_yaw; 
  double max_left_hip_pitch; 
  double max_right_hip_roll; 
  double max_right_hip_yaw; 
  double max_right_hip_pitch; 
  double max_left_knee; 
  double max_right_knee; 

  double min_left_hip_roll; 
  double min_left_hip_yaw; 
  double min_left_hip_pitch; 
  double min_right_hip_roll; 
  double min_right_hip_yaw; 
  double min_right_hip_pitch; 
  double min_left_knee; 
  double min_right_knee; 

  double max_base_roll;
  double max_base_pitch;
  double max_base_yaw;
  double min_base_roll;
  double min_base_pitch;
  double min_base_yaw;

  //for(int i = 0;i<1102;i+=29)//有问题
  for(int i = 0;i<1102;i+=29)//有问题
  {
    drake::Quaternion<double> Quaternion_state(result.GetSolution()[i],result.GetSolution()[i+1], result.GetSolution()[i+2], result.GetSolution()[i+3]);//生成eigen 四元数
    drake::math::RollPitchYaw< double >  RollPitchYaw_state(Quaternion_state.normalized());
    drake::VectorX<double> rpy(3);
    rpy(0)=RollPitchYaw_state.roll_angle();//.template cast<T>()
    rpy(1)=RollPitchYaw_state.pitch_angle();
    rpy(2)=RollPitchYaw_state.yaw_angle();

    if(max_base_roll<rpy(0))
    max_base_roll=rpy(0);
    if(min_base_roll>rpy(0))
    min_base_roll=rpy(0);

    if(max_base_pitch<rpy(1))
    max_base_pitch=rpy(1);
    if(min_base_pitch>rpy(1))
    min_base_pitch=rpy(1);

    if(max_base_yaw<rpy(2))
    max_base_yaw=rpy(2);
    if(min_base_yaw>rpy(2))
    min_base_yaw=rpy(2);

    if(max_left_hip_roll<result.GetSolution()[7])
    max_left_hip_roll=result.GetSolution()[7];
    if(min_left_hip_roll>result.GetSolution()[7])
    min_left_hip_roll=result.GetSolution()[7];
    if(max_right_hip_roll<result.GetSolution()[8])
    max_right_hip_roll=result.GetSolution()[8];
    if(min_right_hip_roll>result.GetSolution()[8])
    min_right_hip_roll=result.GetSolution()[8];
    
    if(max_left_hip_yaw<result.GetSolution()[9])
    max_left_hip_yaw=result.GetSolution()[9];
    if(min_left_hip_yaw>result.GetSolution()[9])
    min_left_hip_yaw=result.GetSolution()[9];
    if(max_right_hip_yaw<result.GetSolution()[10])
    max_right_hip_yaw=result.GetSolution()[10];
    if(min_right_hip_yaw>result.GetSolution()[10])
    min_right_hip_yaw=result.GetSolution()[10];

    if(max_left_hip_pitch<result.GetSolution()[11])
    max_left_hip_pitch=result.GetSolution()[11];
    if(min_left_hip_pitch>result.GetSolution()[11])
    min_left_hip_pitch=result.GetSolution()[11];
    if(max_right_hip_pitch<result.GetSolution()[12])
    max_right_hip_pitch=result.GetSolution()[12];
    if(min_right_hip_pitch>result.GetSolution()[12])
    min_right_hip_pitch=result.GetSolution()[12];

    if(max_left_knee<result.GetSolution()[13])
    max_left_knee=result.GetSolution()[13];
    if(min_left_knee>result.GetSolution()[13])
    min_left_knee=result.GetSolution()[13];
    if(max_right_knee<result.GetSolution()[14])
    max_right_knee=result.GetSolution()[14];
    if(min_left_knee>result.GetSolution()[14])
    min_left_knee=result.GetSolution()[14];
    // std::cout<<trajopt.decision_variable(i)<<"="<<result.GetSolution()[i]<<std::endl;
  }
  std::cout <<"result.is_success():"<<result.is_success()<<std::endl;
  std::cout<<"|max_u|"<<"="<<max_u<<std::endl;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;  
  std::cout<<"max_base_roll"<<"="<<max_base_roll<<std::endl;
  std::cout<<"min_base_roll"<<"="<<min_base_roll<<std::endl;
  std::cout<<"max_base_yaw"<<"="<<max_base_yaw<<std::endl;
  std::cout<<"max_base_yaw"<<"="<<min_base_yaw<<std::endl;
  std::cout<<"max_base_pitch"<<"="<<max_base_pitch<<std::endl;
  std::cout<<"min_base_pitch"<<"="<<min_base_pitch<<std::endl;
  std::cout<<"max_left_hip_roll"<<"="<<max_left_hip_roll<<std::endl;
  std::cout<<"min_left_hip_roll"<<"="<<min_left_hip_roll<<std::endl;
  std::cout<<"max_left_hip_yaw"<<"="<<max_left_hip_yaw<<std::endl;
  std::cout<<"min_left_hip_yaw"<<"="<<min_left_hip_yaw<<std::endl;
  std::cout<<"max_left_hip_pitch"<<"="<<max_left_hip_pitch<<std::endl;
  std::cout<<"min_left_hip_pitch"<<"="<<min_left_hip_pitch<<std::endl;

  std::cout<<"max_right_hip_roll"<<"="<<max_right_hip_roll<<std::endl;
  std::cout<<"min_right_hip_roll"<<"="<<min_right_hip_roll<<std::endl;
  std::cout<<"max_right_hip_yaw"<<"="<<max_right_hip_yaw<<std::endl;
  std::cout<<"min_right_hip_yaw"<<"="<<min_right_hip_yaw<<std::endl;
  std::cout<<"max_right_hip_pitch"<<"="<<max_right_hip_pitch<<std::endl;
  std::cout<<"min_right_hip_pitch"<<"="<<min_right_hip_pitch<<std::endl;

  std::cout<<"max_left_knee"<<"="<<max_left_knee<<std::endl;
  std::cout<<"min_left_knee"<<"="<<min_left_knee<<std::endl;
  std::cout<<"max_right_knee"<<"="<<max_right_knee<<std::endl;
  std::cout<<"min_right_knee"<<"="<<min_right_knee<<std::endl;

  std::cout <<"result.is_success():"<<result.is_success()<<std::endl;
   std::cout<<"max_u"<<"="<<max_u<<std::endl;
   for(int i=0;i<trajopt.decision_variables().size();i++)
   {
     std::cout<<"trajopt.decision_variable("<<i<<"): "<<trajopt.decision_variable(i)<<std::endl;//lammda0
   }
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1443)<<std::endl;//lammda0
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1473)<<std::endl;//lammdac0
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1500)<<std::endl;//gammac0
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1678)<<std::endl;//lammda1
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1738)<<std::endl;//lammdac1
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1795)<<std::endl;//gammac1
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(2170)<<std::endl;//lammda2
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(2200)<<std::endl;//lammdac2
   std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(2227)<<std::endl;//gammac2

  // std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1536)<<std::endl;//rel_offset_vars[0][0]=rel_x at mode0 when ankle is fixed
  // std::cout<<"result.GetSolution(trajopt.decision_variable(729)): "<<result.GetSolution(trajopt.decision_variable(729))<<std::endl;
  //  std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1777)<<std::endl;//rel_offset_vars[1][0]=rel_x at mode1 when ankle is fixed
  // std::cout<<"result.GetSolution(trajopt.decision_variable(910)): "<<result.GetSolution(trajopt.decision_variable(910))<<std::endl;
  // std::cout<<"trajopt.decision_variable(i): "<<trajopt.decision_variable(1949)<<std::endl;//rel_offset_vars[2][0]=rel_x at mode2 when ankle is fixed
  // std::cout<<"result.GetSolution(trajopt.decision_variable(1004)): "<<result.GetSolution(trajopt.decision_variable(1004))<<std::endl;
  // std::cout<< "trajopt.num_vars()trajopt.num_vars(): "<<  trajopt.num_vars()<<std::endl;//??
  //std::cout<< "trajopt.decision_variables()trajopt.decision_variables(): "<<  trajopt.decision_variables()<<std::endl;
  Eigen::MatrixXd force_value_mode0=trajopt.GetForceSamplesByMode(result,0);
  Eigen::MatrixXd force_value_mode1=trajopt.GetForceSamplesByMode(result,1);
  Eigen::MatrixXd force_value_mode2=trajopt.GetForceSamplesByMode(result,2);
  Eigen::MatrixXd force_c_value_mode0=trajopt.GetForce_C_SamplesByMode(result,0);
  Eigen::MatrixXd force_c_value_mode1=trajopt.GetForce_C_SamplesByMode(result,1);
  Eigen::MatrixXd force_c_value_mode2=trajopt.GetForce_C_SamplesByMode(result,2);
  Eigen::MatrixXd slack_value_mode0=trajopt.GetGamma_C_SamplesByMode(result,0);
  Eigen::MatrixXd slack_value_mode1=trajopt.GetGamma_C_SamplesByMode(result,1);
  Eigen::MatrixXd slack_value_mode2=trajopt.GetGamma_C_SamplesByMode(result,2);
  std::cout<< "force_value_mode0.size(): "<<  force_value_mode0.rows()<<"x"<<force_value_mode0.cols()<<std::endl;//应当是3x10
  std::cout<< "force_value_mode0.data(): "<<  force_value_mode0<<std::endl;//3x10
  std::cout<< "force_value_mode1.size(): "<<  force_value_mode1.rows()<<"x"<<force_value_mode1.cols()<<std::endl;//3x20
  std::cout<< "force_value_mode1.data(): "<<  force_value_mode1<<std::endl;//3x20
  std::cout<< "force_value_mode2.size(): "<<  force_value_mode2.rows()<<"x"<<force_value_mode2.cols()<<std::endl;//3x20
  std::cout<< "force_value_mode2.data(): "<<  force_value_mode2<<std::endl;//3x10  
  
  std::cout<< "force_c_value_mode0.size(): "<<  force_c_value_mode0.rows()<<"x"<<force_c_value_mode0.cols()<<std::endl;//3x9
  std::cout<< "force_c_value_mode0.data(): "<<  force_c_value_mode0<<std::endl;
  std::cout<< "force_c_value_mode1.size(): "<<  force_c_value_mode1.rows()<<"x"<<force_c_value_mode1.cols()<<std::endl;//3x19
  std::cout<< "force_c_value_mode1.data(): "<<  force_c_value_mode1<<std::endl;
  std::cout<< "force_c_value_mode2.size(): "<<  force_c_value_mode2.rows()<<"x"<<force_c_value_mode2.cols()<<std::endl;//3x9
  std::cout<< "force_c_value_mode2.data(): "<<  force_c_value_mode2<<std::endl;
  
  std::cout<< "slack_value_mode0.size(): "<<  slack_value_mode0.rows()<<"x"<<slack_value_mode0.cols()<<std::endl;//3x9
  std::cout<< "slack_value_mode0.data(): "<<  slack_value_mode0<<std::endl;
  std::cout<< "slack_value_mode1.size(): "<<  slack_value_mode1.rows()<<"x"<<slack_value_mode1.cols()<<std::endl;//3x19
  std::cout<< "slack_value_mode1.data(): "<<  slack_value_mode1<<std::endl;
  std::cout<< "slack_value_mode2.size(): "<<  slack_value_mode2.rows()<<"x"<<slack_value_mode2.cols()<<std::endl;//3x9
  std::cout<< "slack_value_mode2.data(): "<<  slack_value_mode2<<std::endl; 
  //std::cout<<"trajopt.decision_variable(0).to_string()"<<trajopt.decision_variable(0).to_string()<<std::endl;//398   
  //std::cout<<"trajopt.decision_variable(578).to_string()"<<trajopt.decision_variable(578).to_string()<<std::endl;//398   
  std::cout<<"trajopt.decision_variable_values:"<<result.GetSolution()<<std::endl;//579个值lammda0
  auto decision_variables_value = result.GetSolution();
  std::cout<<"decision_variables_value.size()"<<decision_variables_value.size()<<std::endl;

    //读取第一mode中的lammda lammda_c gamma_c
    std::fstream out_lammda_data_file0;
    std::fstream out_lammda_c_data_file0;
    std::fstream out_gamma_c_data_file0;
    out_lammda_data_file0.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/lammda_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::trunc);//输出且写入清空
    out_lammda_c_data_file0.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/lammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::trunc);
    out_gamma_c_data_file0.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/gammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::trunc);
    out_lammda_data_file0 << std::fixed;
    out_lammda_c_data_file0 << std::fixed;//不用科学计数法
    out_gamma_c_data_file0 << std::fixed;//不用科学计数法

  Eigen::MatrixXd lammda_value_mode0(3,10);
  Eigen::MatrixXd lammda_c_value_mode0(3,9);
  Eigen::MatrixXd gamma_c_value_mode0(3,9);
  Eigen::MatrixXd lammda_value_mode1(3,20);
  Eigen::MatrixXd lammda_c_value_mode1(3,19);
  Eigen::MatrixXd gamma_c_value_mode1(3,19);
  Eigen::MatrixXd lammda_value_mode2(3,10);
  Eigen::MatrixXd lammda_c_value_mode2(3,9);
  Eigen::MatrixXd gamma_c_value_mode2(3,9);

  //以下代买各个决策变量在总体变量list中的index需要重新计算
   //以下代买各个决策变量在总体变量list中的index需要重新计算
    //以下代买各个决策变量在总体变量list中的index需要重新计算
  int lammda_index0 = 1443;
  for(int i=0;i<10;i++)
  {
    for(int j=0;j<3;j++)
    {
      //lammda_value_mode0(j,i) = trajopt.GetForceSamplesByMode(result,0)[i,j];
      lammda_value_mode0(j,i) = decision_variables_value[lammda_index0];
      out_lammda_data_file0 << lammda_value_mode0(j,i)<< std::endl;//四位有效数字<< std::setprecision(4) 
      lammda_index0++;
      std::cout<<"lammda_value_mode0(j,i)"<<lammda_value_mode0(j,i)<<std::endl;
    }
  }
  out_lammda_data_file0.close();  
  //判断从decision variables来的值是不是等于其他method来的,应当yes
  if(lammda_value_mode0==force_value_mode0)
  std::cout<<"yes lammda_value_mode0==force_value_mode0"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no lammda_value_mode0!=force_value_mode0"<<std::endl;//579个值lammda0   


   int lammda_c_index0 = 1473;
    for(int i=0;i<9;i++)
    {
      for(int j=0;j<3;j++)
      {
        //lammda_c_value_mode0(j,i) = trajopt.GetForce_C_SamplesByMode(result,0)[];
        lammda_c_value_mode0(j,i) = decision_variables_value[lammda_c_index0];
        out_lammda_c_data_file0 << lammda_c_value_mode0(j,i)<<std::endl;//四位有效数字<< std::setprecision(4) 
        lammda_c_index0++;
      }
    }
  lammda_c_index0=lammda_c_index0-3;
    for(int j=0;j<3;j++)
    {
      out_lammda_c_data_file0  << decision_variables_value[lammda_c_index0]<<std::endl;//四位有效数字<< std::setprecision(4)
      lammda_c_index0++;
    }
  out_lammda_c_data_file0.close();
  if(lammda_c_value_mode0==force_c_value_mode0)
  std::cout<<"yes lammda_c_value_mode0==force_c_value_mode0"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no lammda_c_value_mode0!=force_c_value_mode0"<<std::endl;//579个值lammda0   


   int gamma_c_index0 = 1500;
    for(int i=0;i<9;i++)
    {
      for(int j=0;j<3;j++)
      {
        gamma_c_value_mode0(j,i) = decision_variables_value[gamma_c_index0];
        out_gamma_c_data_file0  << gamma_c_value_mode0(j,i)<< std::endl;//四位有效数字<< std::setprecision(4)
        gamma_c_index0++;
      }
    }
  gamma_c_index0=gamma_c_index0-3;
    for(int j=0;j<3;j++)
    {
      out_gamma_c_data_file0 << decision_variables_value[gamma_c_index0]<<std::endl;//四位有效数字<< std::setprecision(4) 
      gamma_c_index0++;
    }
    out_gamma_c_data_file0.close();  
    if(gamma_c_value_mode0==slack_value_mode0)
    std::cout<<"yes gamma_c_value_mode0==slack_value_mode0"<<std::endl;//579个值lammda0  
    else
    std::cout<<"no gamma_c_value_mode0!=slack_value_mode0"<<std::endl;//579个值lammda0   


    //读取mode1中的lammda lammda_c gamma_c
    std::fstream out_lammda_data_file1;
    std::fstream out_lammda_c_data_file1;
    std::fstream out_gamma_c_data_file1;  
    out_lammda_data_file1.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/lammda_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::app);
    out_lammda_c_data_file1.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/lammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::app);
    out_gamma_c_data_file1.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/gammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::app);
    out_lammda_data_file1 << std::fixed;
    out_lammda_c_data_file1 << std::fixed;//不用科学计数法
    out_gamma_c_data_file1 << std::fixed;//不用科学计数法
    int lammda_index1 = 1678;
    for(int i=0;i<20;i++)
    {
      for(int j=0;j<3;j++)
      {
        lammda_value_mode1(j,i) = decision_variables_value[lammda_index1];
        out_lammda_data_file1 << lammda_value_mode1(j,i)<<std::endl;//四位有效数字<< std::setprecision(4) 
        lammda_index1++;
        std::cout<<"lammda_value_mode1(j,i)"<<lammda_value_mode1(j,i)<<std::endl;
      }
    }
    out_lammda_data_file1.close();
    if(lammda_value_mode1==force_value_mode1)
    std::cout<<"yes lammda_value_mode1==force_value_mode1"<<std::endl;//579个值lammda0  
    else
    std::cout<<"no lammda_value_mode1!=force_value_mode1"<<std::endl;//579个值lammda0   

   int lammda_c_index1 = 1738;
    for(int i=0;i<19;i++)
    {
      for(int j=0;j<3;j++)
      {
        lammda_c_value_mode1(j,i) = decision_variables_value[lammda_c_index1];
        out_lammda_c_data_file1  << lammda_c_value_mode1(j,i)<<std::endl;//四位有效数字<< std::setprecision(4)
        lammda_c_index1++;
      }
    }
    lammda_c_index1=lammda_c_index1-3;
    for(int j=0;j<3;j++)
    {
      out_lammda_c_data_file1 << decision_variables_value[lammda_c_index1]<<std::endl;//四位有效数字<< std::setprecision(4) 
      lammda_c_index1++;
    }
    out_lammda_c_data_file1.close();
  if(lammda_c_value_mode1==force_c_value_mode1)
    std::cout<<"yes lammda_c_value_mode1==force_c_value_mode1"<<std::endl;//579个值lammda0  
  else
    std::cout<<"no lammda_c_value_mode1!=force_c_value_mode1"<<std::endl;//579个值lammda0   

   int gamma_c_index1 = 1795;
    for(int i=0;i<19;i++)
    {
      for(int j=0;j<3;j++)
      {
        gamma_c_value_mode1(j,i) = decision_variables_value[gamma_c_index1];
        out_gamma_c_data_file1  << gamma_c_value_mode1(j,i)<<std::endl;//四位有效数字<< std::setprecision(4)
        gamma_c_index1++;
      }
    }
    gamma_c_index1=gamma_c_index1-3;
    for(int j=0;j<3;j++)
    {
      out_gamma_c_data_file1<< decision_variables_value[gamma_c_index1]<<std::endl;//四位有效数字 << std::setprecision(4) 
      gamma_c_index1++;
    }
    out_gamma_c_data_file1.close();
    if(gamma_c_value_mode1==slack_value_mode1)
    std::cout<<"yes gamma_c_value_mode1==slack_value_mode1"<<std::endl;//579个值lammda0  
    else
    std::cout<<"no gamma_c_value_mode1!=slack_value_mode1"<<std::endl;//579个值lammda0   


    std::fstream out_lammda_data_file2;
    std::fstream out_lammda_c_data_file2;
    std::fstream out_gamma_c_data_file2;  
    out_lammda_data_file2.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/lammda_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::app);
    out_lammda_c_data_file2.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/lammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::app);
    out_gamma_c_data_file2.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/gammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::app);
    out_lammda_data_file2 << std::fixed;
    out_lammda_c_data_file2 << std::fixed;//不用科学计数法
    out_gamma_c_data_file2 << std::fixed;//不用科学计数法
    int lammda_index2 = 2170;
    for(int i=0;i<10;i++)
    {
      for(int j=0;j<3;j++)
      {
        lammda_value_mode2(j,i) = decision_variables_value[lammda_index2];
        out_lammda_data_file2 << lammda_value_mode2(j,i)<< std::endl;//四位有效数字<< std::setprecision(4) 
        lammda_index2++;
        std::cout<<"lammda_value_mode2(j,i)"<<lammda_value_mode2(j,i)<<std::endl;
      }
    }
  out_lammda_data_file2.close();  
  //判断从decision variables来的值是不是等于其他method来的,应当yes
  if(lammda_value_mode2==force_value_mode2)
  std::cout<<"yes lammda_value_mode2==force_value_mode2"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no lammda_value_mode2!=force_value_mode2"<<std::endl;//579个值lammda0   


   int lammda_c_index2 = 2200;
    for(int i=0;i<9;i++)
    {
      for(int j=0;j<3;j++)
      {
        lammda_c_value_mode2(j,i) = decision_variables_value[lammda_c_index2];
        out_lammda_c_data_file2  << lammda_c_value_mode2(j,i)<<std::endl;//四位有效数字<< std::setprecision(4)
        lammda_c_index2++;
      }
    }
   lammda_c_index2=lammda_c_index2-3;
    for(int j=0;j<3;j++)
    {
      out_lammda_c_data_file2<< decision_variables_value[lammda_c_index2]<<std::endl;//四位有效数字 << std::setprecision(4) 
      lammda_c_index2++;
    }
  out_lammda_c_data_file2.close();
  if(lammda_c_value_mode2==force_c_value_mode2)
  std::cout<<"yes lammda_c_value_mode2==force_c_value_mode2"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no lammda_c_value_mode2!=force_c_value_mode2"<<std::endl;//579个值lammda0   


    int gamma_c_index2 = 2227;
    for(int i=0;i<9;i++)
    {
      for(int j=0;j<3;j++)
      {
        gamma_c_value_mode2(j,i) = decision_variables_value[gamma_c_index2];
        out_gamma_c_data_file2 << gamma_c_value_mode2(j,i)<< std::endl;//四位有效数字<< std::setprecision(4) 
        gamma_c_index2++;
      }
    }
    gamma_c_index2=gamma_c_index2-3;
    for(int j=0;j<3;j++)
    {
      out_gamma_c_data_file2  << decision_variables_value[gamma_c_index2]<<std::endl;//四位有效数字<< std::setprecision(4)
      gamma_c_index2++;
    }
  out_gamma_c_data_file2.close();  
  if(gamma_c_value_mode2==slack_value_mode2)
  std::cout<<"yes gamma_c_value_mode2==slack_value_mode2"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no gamma_c_value_mode2!=slack_value_mode2"<<std::endl;//579个值lammda0   
  //以上代买各个决策变量在总体变量list中的index需要重新计算对于fixedankle完成
   //以上代买各个决策变量在总体变量list中的index需要重新计算
    //以上代买各个决策变量在总体变量list中的index需要重新计算



 //multibody::connectTrajectoryVisualizer(plant_double_ptr,&builder, &scene_graph, pp_xtraj);
//目前未知,不算上面这个函数,仅注册了scene_graph系统,diagram中仅有一个

auto traj_source_new = builder.AddSystem<drake::dairlib::cby::extension6_trajectory<double>>(pp_xtraj, plant_double_ptr);//<double>
traj_source_new->Length=result.GetSolution()[1077];
std::cout<<trajopt.decision_variable(1077)<<"="<<result.GetSolution()[1077]<<std::endl;
std::cout<<"traj_source_new->Length= "<<traj_source_new->Length<<std::endl;
auto to_pose =builder.AddSystem<drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(*plant_double_ptr);
builder.Connect(traj_source_new->get_output_port(), to_pose->get_input_port());
builder.Connect(to_pose->get_output_port(),  scene_graph.get_source_pose_port(plant_double_ptr->get_source_id().value()));
drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
std::cout<<"hao many system now:"<<builder.GetMutableSystems().size()<<std::endl;
for(int i = 0; i<builder.GetMutableSystems().size();i++)
{
  std::cout << "system name:"  <<builder.GetMutableSystems()[i]->GetMemoryObjectName()<<std::endl;
}
    //std::cout << "3"  <<std::endl;
 auto diagram = builder.Build();

    
   double trajectory_time_u =  pp_utraj.end_time()-pp_utraj.start_time();
   double trajectory_time_x =  pp_xtraj.end_time()-pp_xtraj.start_time();
    // auto trajectory_duration =  pp_utraj.duration(1);
     //const std::vector<double>& trajectory_breaks=  pp_utraj.breaks();
    std::cout << "trajectory_time_u:"  <<trajectory_time_u<<std::endl;
    std::cout << "trajectory_time_x:"  <<trajectory_time_x<<std::endl;
    //100Hz均匀取值
    double times = 0.0;
    for(int m = 0; m<200;m++)
    {
        std::cout << "time:"  <<times<<std::endl;
        std::cout << pp_utraj.value(times).rows()<<"x"  <<pp_utraj.value(times).cols()<<std::endl;
        std::cout <<"u = "<<pp_utraj.value(times)<<std::endl;
        // std::cout <<"u1 = "<<pp_utraj.value(times)[1,0]<<std::endl;
        // std::cout <<"u2 = "<<pp_utraj.value(times)[2,0]<<std::endl;
        // std::cout <<" u3 = "<<pp_utraj.value(times)[3,0]<<std::endl;
        result_u.push_back(pp_utraj.value(times));
        result_x.push_back(pp_xtraj.value(times));
        times += 0.005;//20hz
        //times += 0.01;100hz
    }   
    // //读取每个knotpoint值,共38个0.2m/s
    // times = 0.0;
    // for(int m = 0; m<38;m++)
    // {
    //     result_u_prior.push_back(pp_utraj.value(times));
    //     result_x_prior.push_back(pp_xtraj.value(times));
    //     if(m<37)
    //     times += 2/37;   
    //     else if(m==37)
    //     times=2.0;
    // } 

    times = 0.0;
    for(int m = 0; m<38;m++)
    {
        result_u_prior.push_back(pp_utraj.value(times));
        result_x_prior.push_back(pp_xtraj.value(times));
        if(m<37)
        times += (1.0/18);   
        else if(m==37) 
        times=1.0;
        //std::cout<<times<<std::endl;
        std::unique_ptr<Context<T>> context_check = plant.CreateDefaultContext();
        drake::VectorX<T> pt_target_check;
        drake::VectorX<T> state_check = result_x_prior[m];
            int mode_index_=0;
            if(m<=8)
            mode_index_=0;
            if(m>=10 && m<=27)
            mode_index_=1;
            if(m>=28 && m<=37)
            mode_index_=2;
            //plant.SetPositions(context, q);
            context_check->SetContinuousState(state_check.head(29));
            const Eigen::Vector3d pt_float_contact(0.13, 0, -0.31);//以foot作为落足点30//这里需要重新设置,保证精确0.10, 0, -0.29
            drake::VectorX<T>pt_target_contact(3);//以foot作为落足点30//这里需要重新设置,保证精确
            const drake::multibody::Frame<T>& left_ankle = plant.GetFrameByName("left_tarsus_Link");//left_lower_leglink_left_toe
            const drake::multibody::Frame<T>& right_ankle = plant.GetFrameByName("right_tarsus_Link");//left_lower_leglink_left_toe
            const drake::multibody::Frame<T>& world_check = plant.world_frame();
            Eigen::MatrixXd mn(3,1);
            mn.col(0)=pt_float_contact;
            if(mode_index_==0)
            {
              plant.CalcPointsPositions(*context_check,right_ankle,mn.template cast<T>(),world_check,&pt_target_contact);
            }
            else if(mode_index_==1)
            {
              plant.CalcPointsPositions(*context_check,left_ankle,mn.template cast<T>(),world_check,&pt_target_contact);
            }
            else if(mode_index_==2)
            {
              plant.CalcPointsPositions(*context_check,right_ankle,mn.template cast<T>(),world_check,&pt_target_contact);
            }  
            pt_target_check=pt_target_contact;
            std::cout<<"pt_target_check(z)= "<<pt_target_check(2)<<std::endl;
    }   
    std::fstream out_x_data_file;
    std::fstream out_u_data_file;
    out_u_data_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/u_planed_outsearch.txt", std::ios::out | std::ios::trunc);
    out_x_data_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/x_planed_outsearch.txt", std::ios::out | std::ios::trunc);
    out_x_data_file << std::fixed;
    out_u_data_file << std::fixed;//不用科学计数法
    for(int i=0;i<result_u_prior.size();i++)
  {
        out_u_data_file  << result_u_prior[i]<< std::endl;//四位有效数字<< std::setprecision(4)
        out_x_data_file << result_x_prior[i]<< std::endl;//std::setprecision(4)显示4位<< std::setprecision(4) 
  }
    out_x_data_file.close();  
    out_u_data_file.close(); 
    std::cout << "result_u.size():"  <<result_u.size()<<std::endl;//4*150      完整一步时是4*200
    std::cout << "result_x.size():"  <<result_x.size()<<std::endl;//12*150   完整一步时是20*200
    std::cout << "result_u[0].size():"  <<result_u[0].size()<<std::endl;
    std::cout << "result_x[0].size():"  <<result_x[0].size()<<std::endl;
   
      for(int t = 0;t<200;t++)
    {

        std::cout << "tttttttttt: "  <<t<<std::endl;
        std::cout << "result_x:left_leg_hipdot: "  <<result_x[t](9)<<std::endl;
        std::cout << "result_x:right_hip_pindot: "  <<result_x[t](8)<<std::endl;
         std::cout << "result_x:left_knee_pindot: "  <<result_x[t](11)<<std::endl;
         std::cout << "result_x:right_knee_pindot: "  <<result_x[t](10)<<std::endl;
    } 
         std::cout << "rhdot0: "  <<pp_xtraj.value(0)(8,0)<<std::endl;        
         std::cout << "lhdot0: "  <<pp_xtraj.value(0)(9,0)<<std::endl;      
         std::cout << "rkdot0: "  <<pp_xtraj.value(0)(10,0)<<std::endl;        
         std::cout << "lkdot0: "  <<pp_xtraj.value(0)(11,0)<<std::endl;    
         std::cout << "rhdot1: "  <<pp_xtraj.value(1.5)(8,0)<<std::endl;        
         std::cout << "lhdot1: "  <<pp_xtraj.value(1.5)(9,0)<<std::endl;      
         std::cout << "rkdot1: "  <<pp_xtraj.value(1.5)(10,0)<<std::endl;        
         std::cout << "lkdot1: "  <<pp_xtraj.value(1.5)(11,0)<<std::endl;    
         
     for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:left_leg_hip:"  <<  t<<":"<<result_u[t](0)<<std::endl;
    }    
    
        for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:right_leg_hip:" <<  t<<":" <<result_u[t](1)<<std::endl;
            
    }  

    for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:hip_pin: t"<<  t<<":"<<result_u[t](2)<<std::endl;
    }    
    for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:right_knee_pin:" <<  t<<":" <<result_u[t](3)<<std::endl;
    }     
     std::cout <<"result.is_success():"<<result.is_success()<<std::endl;
     std::cout << "result.get_solver_id().name()"<<result.get_solver_id().name()<<std::endl;
     std::cout << "Solve time:" << elapsed.count() <<std::endl;  
        std::cout<<"max_u"<<"="<<max_u<<std::endl;
      if(result.is_success()==0)
      {
        solve_result=0;
        std::cout << "InfeasibleConstraints.size():" << result.GetInfeasibleConstraintNames(trajopt).size() <<std::endl;  
        std::cout << "InfeasibleConstraints.size():" << result.GetInfeasibleConstraints(trajopt).size() <<std::endl;  
        for(int i=0;i<result.GetInfeasibleConstraintNames(trajopt).size();i++)
        {
          std::cout << "InfeasibleConstraints name:" << result.GetInfeasibleConstraintNames(trajopt)[i] <<std::endl;
          // std::cout << "InfeasibleConstraints:" << result.GetInfeasibleConstraints(trajopt)[i].to_string() <<std::endl;  
          // std::cout << "InfeasibleConstraints num:" << result.GetInfeasibleConstraints(trajopt)[i].evaluator()->num_constraints() <<std::endl;  
        }
      }
  else if(result.is_success()==1)
  {
      solve_result=1;
      while (true) 
      {
        drake::systems::Simulator<double> simulator(*diagram);
        simulator.set_target_realtime_rate(0.5);
        simulator.Initialize();
        simulator.AdvanceTo(10);//pp_xtraj.end_time()
    }  
  }
        while (true) //无论求解是否成功,都可以展示
      {
        drake::systems::Simulator<double> simulator(*diagram);
        simulator.set_target_realtime_rate(0.5);
        simulator.Initialize();
        simulator.AdvanceTo(10);//pp_xtraj.end_time()
    }  
}

}  // namespace
}  // namespace dairlib



std::string get_greet(const std::string& who) {
  return "Hello " + who;
}

/*****这里我们引入一个数学规划求解各个时刻的广义坐标作为位置先验*****/

vector<VectorXd> GetInitGuessForQ(int mode_index, int mode_length, double mode_stride_length, double ground_incline,
                                  const MultibodyPlant<double>& plant,
                                  bool visualize_init_guess = false,
                                  VectorXd q_ik_guess_mode = VectorXd::Zero(15),
                                  VectorXd* prior_q_last_guess = NULL) 
 {
    int n_q = plant.num_positions();
    int n_v = plant.num_velocities();
    int n_x = n_q + n_v;
    std::map<string, int> positions_map = dairlib::multibody::makeNameToPositionsMap(plant);
      //   for (auto const& element : positions_map)
      //  std::cout << "positions_map:" <<element.first << " = " << element.second << std::endl;
    std::vector<VectorXd> q_init_guess;
    VectorXd q_ik_guess = VectorXd::Zero(n_q);
    Eigen::Vector4d quat(2000.06, -0.339462, -0.609533, -0.760854);
    
        for(int i=((mode_index>=1)?1:0);i<mode_length;i++)//逐点的求取逆运动学初始轨迹
        {
          VectorXd q_sol_normd(n_q);//上一次的求解结果
          //if(mode_index==0 && i==0)
          if(mode_index==0)
          {
              q_ik_guess << quat.normalized(), 0.0, 0.0, 0.9,
              0.0, 0.0, 
              0.0, 0.0, 
              0.0, 0.0, 
              0.0, 0.0;
          }
          //else if(mode_index >=1 && i==1)
          else if(mode_index >=1)
          {
            q_ik_guess<<q_ik_guess_mode.head(4).normalized(),q_ik_guess_mode.tail(11);
          }
          // else
          // {
          //   //q_ik_guess<<q_sol_normd;效果不是很好,需要对四元数的各个分量分开约束
          //   q_ik_guess<<q_ik_guess_mode.head(4).normalized(),q_ik_guess_mode.tail(11);
          // }
          double eps = 1e-3;
          Eigen::Vector3d eps_vec = eps * VectorXd::Ones(3);
          Eigen::Vector3d pelvis_pos(0.0, 0.0, 0.9);//依次递增的基座position 
          double stance_toe_pos_x = 0.0;
          double swing_toe_pos_x = 0.0;
          double swing_toe_pos_y = 0.0;
          Vector3d stance_toe_pos;

          if(mode_index==0)//mode0
          {
            //pelvis_pos[0]=0.0*FLAGS_strideLength+FLAGS_strideLength * i / (mode_length- 1);//基座在mode0走了0.25L
            pelvis_pos[0]=0.0*FLAGS_strideLength+0.5*FLAGS_strideLength * i / (mode_length- 1);//基座在mode0走了0.25L
            stance_toe_pos_x = 0.0;//mode0 支撑足的x在0.0L处
            stance_toe_pos(0)=stance_toe_pos_x;
            stance_toe_pos(1)=0.10;
            stance_toe_pos(2)=0.00 + tan(-ground_incline) * stance_toe_pos_x;//ground_incline=0
            //swing_toe_pos_x =0+2*FLAGS_strideLength* i / (mode_length - 1);//mode0浮动足的x坐标,浮动足走了0.5L,right leg 4*FLAGS_strideLength
            swing_toe_pos_x =0+1*FLAGS_strideLength* i / (mode_length - 1);//mode0浮动足的x坐标,浮动足走了0.5L,right leg 4*FLAGS_strideLength
            swing_toe_pos_y=-0.10;
          }
          else if (mode_index == 1)//mode1
          {
            //pelvis_pos[0]=1*FLAGS_strideLength+2*FLAGS_strideLength * i / (mode_length- 1);//基座在mode1走了0.5L,结束时处于0.75L
            pelvis_pos[0]=0.5*FLAGS_strideLength+1*FLAGS_strideLength * i / (mode_length- 1);//基座在mode1走了0.5L,结束时处于0.75L
            //stance_toe_pos_x = 2*FLAGS_strideLength;//mode1支撑足在0.5L处 4*FLAGS_strideLength
            stance_toe_pos_x = 1*FLAGS_strideLength;//mode1支撑足在0.5L处 2*FLAGS_strideLength
            stance_toe_pos(0)=stance_toe_pos_x;
            stance_toe_pos(1)=-0.10;
            stance_toe_pos(2)=0.00 + tan(-ground_incline) * stance_toe_pos_x;
            //swing_toe_pos_x =0+4*FLAGS_strideLength* i / (mode_length - 1);//浮动足走了1L left leg 4*FLAGS_strideLength
            swing_toe_pos_x =0+2*FLAGS_strideLength* i / (mode_length - 1);//浮动足走了1L left leg 2*FLAGS_strideLength
            swing_toe_pos_y=0.10;
          }
          else if(mode_index == 2)//mode2
          {
            //pelvis_pos[0]=3*FLAGS_strideLength+FLAGS_strideLength * i / (mode_length- 1);//基座在mode2走了0.25L,从0.75L-->1.0L
            pelvis_pos[0]=1.5*FLAGS_strideLength+0.5*FLAGS_strideLength * i / (mode_length- 1);//基座在mode2走了0.25L,从0.75L-->1.0L
            //stance_toe_pos_x = 4*FLAGS_strideLength;//mode2支撑足在1L处
            stance_toe_pos_x = 2*FLAGS_strideLength;//mode2支撑足在1L处
            stance_toe_pos(0)=stance_toe_pos_x;
            stance_toe_pos(1)=0.10;
            stance_toe_pos(2)=0.00 + tan(-ground_incline) * stance_toe_pos_x;
            //swing_toe_pos_x =2*FLAGS_strideLength + 2*FLAGS_strideLength* i / (mode_length - 1);//浮动足走了1L left leg 4*FLAGS_strideLength
            swing_toe_pos_x =1*FLAGS_strideLength + 1*FLAGS_strideLength* i / (mode_length - 1);//浮动足走了1L left leg 4*FLAGS_strideLength
            swing_toe_pos_y=-0.10;
          }
          Vector3d swing_toe_pos(swing_toe_pos_x, swing_toe_pos_y, 0.01 + 0.02 * (-std::abs((i - mode_length / 2.0) / (mode_length / 2.0)) + 1) );

          const auto& world_frame = plant.world_frame();
          const auto& pelvis_frame = plant.GetFrameByName("base_link");//flypart
          // const auto& toe_left_frame = plant.GetFrameByName("left_crank_Link");
          // const auto& toe_right_frame = plant.GetFrameByName("right_crank_Link");
          const auto& toe_left_frame = plant.GetFrameByName("left_tarsus_Link");
          const auto& toe_right_frame = plant.GetFrameByName("right_tarsus_Link");
          Vector3d pelvis_bound;
          pelvis_bound(0)=eps*20;
          pelvis_bound(1)=eps*20;
          pelvis_bound(2)=eps*50;
          drake::multibody::InverseKinematics ik(plant);
          ik.AddPositionConstraint(pelvis_frame, Vector3d(0, 0, 0), world_frame, pelvis_pos - pelvis_bound, pelvis_pos +pelvis_bound);
          ik.AddOrientationConstraint(pelvis_frame, drake::math::RotationMatrix<double>(), world_frame, drake::math::RotationMatrix<double>(), 100*eps);
          if(mode_index==0 || mode_index == 2) 
          {
          ik.AddPositionConstraint(toe_left_frame, Vector3d(0.13, 0, -0.31), world_frame, stance_toe_pos - eps_vec, stance_toe_pos + eps_vec);
          ik.AddPositionConstraint(toe_right_frame, Vector3d(0.13, 0, -0.31), world_frame, swing_toe_pos - eps_vec, swing_toe_pos + eps_vec);
          }
          else if(mode_index==1)
          {
          ik.AddPositionConstraint(toe_right_frame, Vector3d(0.13, 0, -0.31), world_frame, stance_toe_pos - eps_vec, stance_toe_pos + eps_vec);
          ik.AddPositionConstraint(toe_left_frame, Vector3d(0.13, 0, -0.31), world_frame, swing_toe_pos - eps_vec, swing_toe_pos + eps_vec);
          }
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_rollmotor_yawmotor")) >= -0.5);
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_rollmotor_yawmotor")) <= 0.5);
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_rollmotor_yawmotor")) >= -0.5);
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_rollmotor_yawmotor")) <= 0.5);
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_yawmotor_pitchmotor")) >= -1.5);
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_yawmotor_pitchmotor")) <= 1.5);
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_yawmotor_pitchmotor")) >= -1.5);
          ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_yawmotor_pitchmotor")) <= 1.5);
          
   // //添加了关节限位约束的逆运动学数学规划 才是有意义的
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(plant);
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("base_z"))  >= 0);  
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_pitchmotor_thigh")) >= -0.4);//joint_right_knee    
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_pitchmotor_thigh")) <= 0.4);  //0.8
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_knee")) >= -0.8);
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_left_knee")) <= 0.8);
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_pitchmotor_thigh")) >= -0.4);
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_pitchmotor_thigh")) <= 0.4); 
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_knee")) >= -0.8);
  ik.get_mutable_prog()->AddLinearConstraint((ik.q())(positions_map.at("joint_right_knee")) <= 0.8);
          ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_ik_guess);
          const auto result = Solve(ik.prog());
          std::cout<<result.is_success()<<std::endl;
          if(result.is_success()==false)
          {
            for(int i=0;i<result.GetInfeasibleConstraintNames(ik.prog()).size();i++)
            {
              std::cout << "InfeasibleConstraints name:" << result.GetInfeasibleConstraintNames(ik.prog())[i] <<std::endl;
              std::cout << "InfeasibleConstraints to_string:" << result.GetInfeasibleConstraints(ik.prog())[i].to_string()<<std::endl;
            }
          }
        
           DRAKE_DEMAND(result.is_success() == true);
          const auto q_sol = result.GetSolution(ik.q());
          q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
          q_init_guess.push_back(q_sol_normd);
          if(mode_index==0 && i==9)
          {
            *prior_q_last_guess = q_init_guess.back();
          }
          if(mode_index==1 && i==19)
          {
            *prior_q_last_guess = q_init_guess.back();
          }
          if(mode_index==2 && i==9)
          {
            *prior_q_last_guess = q_init_guess.back();
          }
        visualize_init_guess=false;
        if (visualize_init_guess) 
        {
            // Build temporary diagram for visualization
            drake::systems::DiagramBuilder<double> builder_ik;
            SceneGraph<double>& scene_graph_ik = *builder_ik.AddSystem<SceneGraph>();
            scene_graph_ik.set_name("scene_graph_ik");
            MultibodyPlant<double> plant_ik(0.0);
            Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));
            dairlib::multibody::addFlatTerrain(&plant_ik, &scene_graph_ik, 3.0, 3.0, ground_normal);
            Parser parser(&plant_ik, &scene_graph_ik);
            string full_name = drake::FindResourceOrThrow("drake/examples/v10/threeD_v2_flaypart_show_multistair.urdf");
            parser.AddModelFromFile(full_name);
            plant_ik.Finalize();

            // Visualize
            VectorXd x_const = VectorXd::Zero(n_x);
            x_const.head(n_q) = q_sol;
            PiecewisePolynomial<double> pp_xtraj(x_const);
            dairlib::multibody::connectTrajectoryVisualizer(&plant_ik, &builder_ik,&scene_graph_ik, pp_xtraj);
            auto diagram = builder_ik.Build();
            drake::systems::Simulator<double> simulator(*diagram);
            simulator.set_target_realtime_rate(0.5);
            simulator.Initialize();
            simulator.AdvanceTo(1.0 / mode_length);
         }
        }
          return q_init_guess;
}
/**/

/*****这里我们利用广义坐标的位置先验生成广义速度先验*****/
// Get v by finite differencing q
std::vector<VectorXd> GetInitGuessForV(int mode, VectorXd prior_mode_last_knot, const vector<VectorXd>& q_seed, double mode_timestep,const MultibodyPlant<double>& plant) 
{
  double dt=0.0;
  if(mode==0)
  {
    dt=mode_timestep/(10-1);
  }
  else if(mode==1)
  {
    dt=mode_timestep/(20-1);
  }
  else if(mode==2)
  {
    dt=mode_timestep/(10-1);
  }
  DRAKE_DEMAND(dt >= 0.0);
    vector<VectorXd> qdot_seed;
    for (unsigned int i = 0; i < q_seed.size(); i++) 
    {
      if(mode==0)
      {
        if (i == 0) 
        {
          qdot_seed.push_back((q_seed[i + 1] - q_seed[i]) / dt);
        }
         else if (i == q_seed.size() - 1) 
        {
          qdot_seed.push_back((q_seed[i] - q_seed[i - 1]) / dt);
        }
        else 
        {
          VectorXd v_plus = (q_seed[i + 1] - q_seed[i]) / dt;
          VectorXd v_minus = (q_seed[i] - q_seed[i - 1]) / dt;
          qdot_seed.push_back((v_plus + v_minus) / 2);
        }
      }
      else if(mode>=1)
      {
        if (i == 0) 
        {
          qdot_seed.push_back((- prior_mode_last_knot +q_seed[i]) / dt);
        }
         else if (i == q_seed.size() - 1) 
        {
          qdot_seed.push_back((q_seed[i] - q_seed[i - 1]) / dt);
        }
        else 
        {
          VectorXd v_plus = (q_seed[i + 1] - q_seed[i]) / dt;
          VectorXd v_minus = (q_seed[i] - q_seed[i - 1]) / dt;
          qdot_seed.push_back((v_plus + v_minus) / 2);
        }
      }
    }
    // Convert qdot to v
    std::vector<VectorXd> v_seed;
    for (unsigned int i = 0; i < q_seed.size(); i++) 
    {
      auto context = plant.CreateDefaultContext();
      plant.SetPositions(context.get(), q_seed[i]);
      VectorXd v(plant.num_velocities());
      plant.MapQDotToVelocity(*context, qdot_seed[i], &v);
      v_seed.push_back(v);
    }
    return v_seed;
}
/**/

int main(int argc, char* argv[]) 
{
    auto start0 = std::chrono::high_resolution_clock::now();
    std::string words = get_greet("CBY project developer!!!");
    std::cout <<words<<std::endl;
    std::cout<<"Current local time: ";
    print_localtime();
    gflags::ParseCommandLineFlags(&argc, &argv, true);  
    std::srand(time(0));  
    auto plant = std::make_unique<drake::multibody::MultibodyPlant<double>>(0.0);
    auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
    auto scene_graph = std::make_unique<SceneGraph<double>>();

    Parser parser(plant.get()); //unique_ptr get classical pointer且没有给scenegraph因为plant对象仅用来规划
    Parser parser_vis(plant_vis.get(), scene_graph.get()); 
  
    std::string full_name = drake::FindResourceOrThrow("drake/examples/v10/threeD_v2_flaypart_show_multistair.urdf");//
    drake::FindResourceResult ResourceResult = drake::FindResource("drake/examples/v10/threeD_v2_flaypart_show_multistair.urdf");//
    std::optional<std::string> full_name_same =  ResourceResult.get_absolute_path();   
    if(full_name_same)
    {
        std::cout<<"full path & name: "<<*full_name_same<<std::endl;//.. means another one
        std::cout << "Success to find urdf file!"  <<std::endl;
        std::string our_urdf_path = "/opt/drake/share/drake/examples";
        std::cout <<"LOCATION: " <<"please add our model here: " << our_urdf_path <<std::endl;    
        std::cout << "We have Ourbot now!"  <<std::endl;
        std::cout <<std::endl;
    }
    else
    {
       std::cout<<"NOT FOUND ERROR: "<<*ResourceResult.get_error_message()<<"\n"<<std::endl;
       std::cout <<std::endl;
    } 
  
    //Parses the SDF or URDF file named in arguement and adds one model to plant. 
    ModelInstanceIndex  mii_p = parser.AddModelFromFile(full_name);
    ModelInstanceIndex  mii_pv =  parser_vis.AddModelFromFile(full_name);
    parser_vis.AddModelFromFile(drake::FindResourceOrThrow("drake/examples/v10/warehouse_show_multistair.sdf"));
    //modelindex in multibody plant system
    auto num_model_instances = plant->num_model_instances();
    std::cout << "Baisc-info1 Multibodyplant 'plant' has "  << num_model_instances<< " modles"<<std::endl;  
    std::cout<<"mii_plant is: "<<mii_p<<std::endl;
    
    auto num_model_instances2 = plant_vis->num_model_instances();
    std::cout << "Baisc-info1 Multibodyplant 'plant_vis' has "  << num_model_instances2<< " modles"<<std::endl;    
    std::cout<<"mii_plant_vis is: "<<mii_pv<<std::endl;
    std::cout<<"Baisc-info2: "<<mii_pv<<std::endl;
    for (ModelInstanceIndex i(0); i<num_model_instances;i++)
    {
        const std::string&  ModelInstance_name = plant->GetModelInstanceName(i);
        ModelInstanceIndex  ModelInstance = plant->GetModelInstanceByName(ModelInstance_name);
        std::cout << "\t model_name:"  <<ModelInstance_name<< std::endl; 
    }
    bool yes=plant->HasModelInstanceNamed("threeD_v2_flaypart_show_multistair");
    std::cout << "\t model_index "<< "threeD_v2_flaypart_show_multistair" <<" has been installed ?: "  <<yes<< std::endl; 
    std::cout <<std::endl;
    //joints 9
    auto num_joints = plant->num_joints();
    std::cout << "Baisc-info3 v10 has "  << num_joints << " Joints"<<std::endl;  
    for(JointIndex i(0); i<num_joints;i++)
    {
        const drake::multibody::Joint<double>&  joint = plant->get_joint(i);
        std::cout << "\tJoints:"  <<joint.name()<< std::endl;         
    }
    std::cout << std::endl;   
    
    //frames 32(1world+16body frame+15 joint frame)
    auto num_frames = plant->num_frames();
    //前12个是每个link算一个 
     std::cout << "Baisc-info3 v10 has "  << num_frames << " frames"<<std::endl;  
     for(FrameIndex i(0); i<plant->num_frames();i++)
     {
         const drake::multibody::Frame<double>& each_frame= plant->get_frame(i);
         std::cout << "\tFrame:"  <<each_frame.name()<< std::endl;  
     }
     std::cout << std::endl;   
    
    //base相当于一个地板(基座)，固定在世界坐标系下
    //将地板(基座) 与世界坐标系合一 
    const drake::multibody::BodyFrame<double>& world_frame = plant->world_frame();
    const drake::multibody::Frame<double>& base_frame= plant->GetFrameByName("base_link");//以flypart作为base  flypart
    const drake::multibody::BodyFrame<double>& world_frame_vis = plant_vis->world_frame();
    const drake::multibody::Frame<double>& base_frame_vis= plant_vis->GetFrameByName("base_link");//flypart
    //plant->WeldFrames(world_frame, base_frame, drake::math::RigidTransform<double>());
    //plant_vis->WeldFrames(world_frame_vis, base_frame_vis, drake::math::RigidTransform<double>());  
    //after all elements in the model (joints, bodies, force elements, constraints, etc.) , 
    plant->Finalize();
    plant_vis->Finalize();
    const drake::multibody::Body<double>&  b1=plant->GetBodyByName("base_link");//flypart
    const drake::multibody::Body<double>&b2=plant->GetBodyByName("base_link");
    std::cout<<"Is quadrotor_base  floating?:"<<b2.is_floating()<<std::endl;
    std::cout<<"Is quadrotor_base  floating?:"<<b1.is_floating()<<std::endl;
    std::cout<<"Is quadrotor_base  floating?:"<<b2.has_quaternion_dofs()<<std::endl;
    std::cout<<"Is quadrotor_base  floating?:"<<b1.has_quaternion_dofs()<<std::endl;
    std::cout << "Topological information configuration complete: plant, plant_vis " <<std::endl;  
    std::cout <<std::endl;         
    
    //state q+q_dot and initialize to 0. note configuration_q_dot  != configuration_v
    int configuration_q = plant->num_positions();//15
    int configuration_v = plant->num_velocities();//14
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(configuration_q + configuration_v);  //29
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(plant->num_actuators());  //8
    std::cout << "plant->num_actuators: "  <<plant->num_actuators()<<std::endl;  
    std::cout << "plant->num_actuated_dofs: "  <<plant->num_actuated_dofs()<<std::endl;  
    std::cout << "Baisc-info4 State dimensions: "  <<(configuration_q + configuration_v)<<std::endl;  
    std::cout <<std::endl;

    //8,lh-roll,lh-pitch,lh-yaw,rh-roll,lh-roll,lh-pitch,ph-yaw,lk,rk for ourbot
    int nu = plant->num_actuators();
    std::cout << "Baisc-info5 Actuators num: "  << nu <<std::endl;  
    for (JointActuatorIndex i(0); i<nu;i++){
          const JointActuator<double>&  joint_actuators = plant->get_joint_actuator(i);
          std::cout << "\t actuator_index:"  <<i<< std::endl; 
          std::cout << "\t actuator_name:"  <<joint_actuators.name()<< std::endl; 
          const drake::multibody::Joint<double>&  actuator_joint = (plant->get_joint_actuator(i)).joint();
          std::cout << "\t joint_name:"  <<actuator_joint.name()<< std::endl; 
      }

    //dimensions of state    
    int nx = plant->num_positions() + plant->num_velocities();
    int N = 10;//knots of mode0
    int M = 20;//对与第二个mode给他20个点,因为他的时长是mode0的两倍
    int L = 10;//knots of mode2
    int i = 0;
    //GRF information 地面接触力，xyz三个方向 
    //一个初始向量，给力初始samples用的
    Eigen::VectorXd init_l_vec(3);
    init_l_vec << 0, 0, 5.93*9.81;//20质量 initial lammda10  8.93

    std::vector<MatrixXd> init_x;
    std::vector<MatrixXd> init_u;
    std::vector<drake::trajectories::PiecewisePolynomial<double>> init_l_traj;//地面接触力，xyz三个方向 traj_init_l contact forces lambda (interpreted at knot points)
    std::vector<PiecewisePolynomial<double>> init_lc_traj;//traj_init_lc contact forces constraint slack variables(interpreted at collocation points)
    std::vector<PiecewisePolynomial<double>> init_vc_traj;  //traj_init_vc velocity constraint slack variables (at collocation)
    // std::cout << "Data type of init_vc_traj: "  <<typeid(init_vc_traj).name()<< std::endl; //i is the alias of int, c=char
    // Initialize state trajectory 0-18:19个值
    //定义break points and knot points. N.B. 这二十个点是两个mode组成的sequence的完整体
    //N.B.as it use a ZeroOrderHold to synthesis an initial x trajectory and u trajectory, so it need 19 point values
    std::vector<double> init_time;
    std::fstream out_ramdom_x_file;//输出获得先验轨迹的初始随机轨迹
    std::fstream out_ramdom_u_file;

    //out_ramdom_x_file和out_ramdom_u_file用来存储随机值
    out_ramdom_x_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/random_x_search112_fixed_ankle.txt", std::ios::out | std::ios::trunc);
    out_ramdom_u_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/random_u_search112_fixed_ankle.txt", std::ios::out | std::ios::trunc);

    // out_ramdom_x_file << std::fixed;
    // out_ramdom_u_file << std::fixed;//不用科学计数法
    VectorXd q_ik_guess_mode= VectorXd::Zero(plant->num_positions());
    VectorXd prior_q_last_guess0(plant->num_positions());
    VectorXd prior_q_last_guess1(plant->num_positions());
    VectorXd prior_q_last_guess2(plant->num_positions());
    std::vector<VectorXd> inkinematic_posirion_prior0 = GetInitGuessForQ(0, 10, FLAGS_strideLength, 0.0, *plant, true, q_ik_guess_mode, &prior_q_last_guess0);
    q_ik_guess_mode<<prior_q_last_guess0;
    std::vector<VectorXd> inkinematic_posirion_prior1 = GetInitGuessForQ(1, 20, FLAGS_strideLength, 0.0, *plant, true, q_ik_guess_mode, &prior_q_last_guess1); 
    q_ik_guess_mode<<prior_q_last_guess1;
    std::vector<VectorXd> inkinematic_posirion_prior2 = GetInitGuessForQ(2, 10, FLAGS_strideLength, 0.0, *plant, true, q_ik_guess_mode, &prior_q_last_guess2);
    // GetInitGuessForQ(int mode_index, int mode_length, double mode_stride_length, double ground_incline,
    // const MultibodyPlant<double>& plant,
    // bool visualize_init_guess = false)     ;
    std::vector<VectorXd> inkinematic_posirion_prior_total;
    for(int i=0;i<inkinematic_posirion_prior0.size();i++)
    {
      inkinematic_posirion_prior_total.push_back(inkinematic_posirion_prior0[i]);
    }
    for(int i=0;i<inkinematic_posirion_prior1.size();i++)
    {
      inkinematic_posirion_prior_total.push_back(inkinematic_posirion_prior1[i]);
    }
    // for(int i=0;i<inkinematic_posirion_prior2.size();i++)
    // {
    //   inkinematic_posirion_prior_total.push_back(inkinematic_posirion_prior2[i]);
    // }
    std::vector<VectorXd> inkinematic_velocity_prior0 = GetInitGuessForV(0, VectorXd::Zero(plant->num_positions()), inkinematic_posirion_prior0, 0.25,*plant);
    std::vector<VectorXd> inkinematic_velocity_prior1 = GetInitGuessForV(1, prior_q_last_guess0, inkinematic_posirion_prior1, 0.5,*plant); 
    std::vector<VectorXd> inkinematic_velocity_prior2 = GetInitGuessForV(2, prior_q_last_guess1, inkinematic_posirion_prior2, 0.25,*plant);
    std::vector<VectorXd> inkinematic_velocity_prior_total;
    for(int i=0;i<inkinematic_velocity_prior0.size();i++)
    {
      inkinematic_velocity_prior_total.push_back(inkinematic_velocity_prior0[i]);
    }
    for(int i=0;i<inkinematic_velocity_prior1.size();i++)
    {
      inkinematic_velocity_prior_total.push_back(inkinematic_velocity_prior1[i]);
    }
    // for(int i=0;i<inkinematic_velocity_prior2.size();i++)
    // {
    //   inkinematic_velocity_prior_total.push_back(inkinematic_velocity_prior2[i]);
    // }
    // DRAKE_DEMAND(inkinematic_posirion_prior_total.size() == N+M+L-3+1);
    // DRAKE_DEMAND(inkinematic_velocity_prior_total.size() == N+M+L-3+1);
    // DRAKE_DEMAND(inkinematic_posirion_prior_total[0].size() == 15);
    // DRAKE_DEMAND(inkinematic_velocity_prior_total[0].size() == 14);
    for (int i = 0; i < 38; i++)//38knots, 37段polynomial,37timesteps
    {
        init_time.push_back(i*0.027);//1seconds for 37 timesteps
        init_x.push_back(x0 + .1*VectorXd::Random(nx));//初始状态轨迹随机初始化
      //   init_x[i].block<15,1>(0,0)=inkinematic_posirion_prior_total[i];
      //   init_x[i].block<14,1>(15,0)=inkinematic_velocity_prior_total[i];
      //   if(init_x[i].block<4,1>(0,0).norm()==0|| std::isnan(init_x[i].block<4,1>(0,0).norm()))
      //  {
        
          init_x[i](0,0)=1;//qw
          init_x[i](1,0)=0;//qx
          init_x[i](2,0)=0;//qy
          init_x[i](3,0)=0;//qz
      std::cout<<"init_x="<<init_x[i]<<std::endl;
        //}
        init_u.push_back(VectorXd::Random(nu));//u[i] 需要存储一个较优的先验
        std::cout<<"init_u="<<init_u[i]<<std::endl;
        //init_u[i]=VectorXd::Zero(nu);
        global_init_x.push_back(init_x[i]);
        global_init_u.push_back(init_u[i]);
        //std::cout<<"init_xcols="<<init_x[i].cols()<<"init_xrows="<<init_x[i].rows()<<std::endl;
        //init_x.push_back(x0);//初始状态轨迹0初始化 x0已经是0无解
        //init_u.push_back(u0);//初始输入轨迹0初始化 u0已经是0无解
        std::cout<<"init_x[i]:"<<init_x[i]<<std::endl;
        std::cout<<"init_u[i]:"<<init_u[i]<<std::endl;
        for(int j=0;j<nx;j++)
        { 
            out_ramdom_x_file << init_x[i](j)<<std::endl;//四位有效数字<< std::setprecision(4) 
        }
        for(int j=0;j<nu;j++)
        { 
            out_ramdom_u_file  << init_u[i](j)<<std::endl;//四位有效数字<< std::setprecision(4)
        }
    }
        out_ramdom_x_file.close();
        out_ramdom_u_file.close();

        int result_only_u = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_u_search112_fixed_ankle.txtxxx", 0);
        //int result_only_u = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/u_planed_outsearch.txtxxx", 0);
        if(result_only_u==0)
        {
          double data_u_ik;
          std::fstream in_u_txt_file_ik;
          in_u_txt_file_ik.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_u_search112_fixed_ankle.txt", std::ios::out | std::ios::in);
          //in_u_txt_file_ik.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/u_planed_outsearch.txt", std::ios::out | std::ios::in);
          //in_u_txt_file_ik << std::fixed;  
          DRAKE_DEMAND(init_u.size()==init_x.size());
          for(int i=0;i<init_u.size();i++)
          {
            for(int j=0;j<nu;j++)
            {
              in_u_txt_file_ik>>data_u_ik;
              init_u[i](j)= data_u_ik;
             //init_u[i](j)= 0;
              std::cout<<"init_u[i](j): "<<init_u[i](j)<<std::endl;
            }
          }
          in_u_txt_file_ik.close();
        }

        int result_only_x = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_x_search112_fixed_ankle.txtxxx", 0);
        if(result_only_x==0)//在这里我们可以引入缩放的鸵鸟状态先验 random_x_search112_fixed_ankle
        {
          double data_x_ik;
          std::fstream in_x_txt_file_ik;
          in_x_txt_file_ik.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_x_search112_fixed_ankle.txt", std::ios::out | std::ios::in);
          //in_x_txt_file_ik << std::fixed;  
          DRAKE_DEMAND(init_u.size()==init_x.size());
          for(int i=0;i<init_u.size();i++)
          {
            for(int j=0;j<nx;j++)
            {
              in_x_txt_file_ik>>data_x_ik;
              init_x[i](j)= data_x_ik;
              std::cout<<"init_x[i](j): "<<init_x[i](j)<<std::endl;
            }
          }
          in_x_txt_file_ik.close();
        }

   std::cout<<"init_time.size(): "<<init_time.size()<<std::endl;
    //random_x_search112_fixed_ankle.txt 与 random_u_search112_fixed_ankle.txt,用来人为指定那个随机初始化的值,即一个较好的随机值
    //u_planed_outsearch.txt与 x_planed_outsearch.txt为规划后的轨迹作为先验
   int result1 = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_x_search112_fixed_anklexxx.txt", 0);//加入先验条件时,需要撤去文件名称的xxx
   int result2 = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_u_search112_fixed_anklexxx.txt", 0);
  if(result1==0&&result2==0)
  {
      std::cout<<"exist prior files xu"<<std::endl;
      std::fstream in_x_txt_file;
      std::fstream in_u_txt_file;
      in_x_txt_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_x_search112_fixed_ankle.txt", std::ios::out | std::ios::in);
      in_u_txt_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/random_u_search112_fixed_ankle.txt", std::ios::out | std::ios::in);
      in_x_txt_file << std::fixed; 
      in_u_txt_file << std::fixed;    /**/
      double data_x; 
      double data_u;
      for(int i=0;i<init_x.size();i++)
      {
        for(int j=0;j<nx;j++)
        {
          in_x_txt_file>>data_x;
          init_x[i](j)= data_x;
          std::cout<<"init_x[i](j): "<<init_x[i](j)<<std::endl;
        }
         for(int j=0;j<nu;j++)
        {
          in_u_txt_file>>data_u;
          init_u[i](j)= data_u;
          std::cout<<"init_u[i](j): "<<init_u[i](j)<<std::endl;
        }
      }
        in_x_txt_file.close();
        in_u_txt_file.close();
  }


    std::cout << "Baisc-info6 breaks of  init_time trajectory:"<<init_time.size()<<std::endl;
    std::cout << "Baisc-info6 Dimensions of init_x at  each knot point:"<<init_x[i].size()<<std::endl;
    std::cout << "Baisc-info6 segments of   init_x trajectory:"<<init_x.size()<<std::endl;
    std::cout << "Baisc-info6 Dimensions of init_u at  each knot point:"<<init_u[0].size()<<std::endl;
    std::cout << "Baisc-info6 segments of init_u trajectory:"<<init_u.size()<<std::endl;
    std::cout <<"TIME BREAK POINT~X KNOT POINT~U KNOT POINT"<<std::endl; 
    std::cout <<std::endl;  
    
    //reminder: x is cubic spline and u is first-order 
    std::cout << "The initial time sequence(break points) and x init trajectory and u init trajectory (knot points on x(t), u(t) )has set."<<std::endl;
  
    auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
    auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

    //在一个breakpoint处多项式的输出的值的形状
    Eigen::Index  row_x = init_x_traj.rows();
    Eigen::Index  col_x = init_x_traj.cols();
    Eigen::Index  row_u = init_u_traj.rows();
    Eigen::Index  col_u = init_u_traj.cols();
    std::cout << "Baisc-info7 Output shape(rows x cols) of init_x_traj at break points: "  <<row_x<<" x "<<col_x<<std::endl;
    std::cout << "Baisc-info7 Output shape(rows x cols) of init_u_traj at break points: "  <<row_u<<" x "<<col_u<<std::endl; 
    //分段多项式的段数
    std::cout << "Baisc-info8 segments  of init_u_traj at break points: "  <<init_u_traj.get_number_of_segments()<<std::endl;
    std::cout << "Baisc-info8 segments  of init_x_traj at break points: "  <<init_x_traj.get_number_of_segments()<<std::endl;
    
    for(int i =0; i<38;i++)
    {
      std::cout<<"init_time[i]= "<<init_time[i]<<"  i="<<i<<std::endl;
      std::cout<<"init_u_traj.value(init_time[i])= "<<init_u_traj.value(i*init_time[i])<<std::endl;
      std::cout<<"init_x_traj.value(init_time[i])= "<<init_x_traj.value(init_time[i])<<std::endl;
    }
    int current_lamda_ptr = 0;
    int current_lamda_c_ptr = 0;
    int current_gamma_c_ptr = 0;
    int lamda_ptr=0;
    int lamda_c_ptr=0;
    int gamma_c_ptr=0;
    
    for (int j = 0; j < 3; j++) //3 modes
    {
        if(j==0)//寻找当前mode初始数据在文件的位置
        {
           lamda_ptr=0;
           lamda_c_ptr=0;
           gamma_c_ptr=0;
        }
        else if(j==1)
        {
           lamda_ptr=current_lamda_ptr;//这个值要重新测
           lamda_c_ptr=current_lamda_c_ptr;//这个值要重新测
           gamma_c_ptr=current_gamma_c_ptr;//这个值要重新测
        }
        else if(j==2)
        {
           lamda_ptr=current_lamda_ptr;//这个值要重新测
           lamda_c_ptr=current_lamda_c_ptr;//这个值要重新测
           gamma_c_ptr=current_gamma_c_ptr;//这个值要重新测
        }        
        std::vector<MatrixXd> init_l_j;//lammda contraint force
        std::vector<MatrixXd> init_lc_j;//lammda_c force correction
        std::vector<MatrixXd> init_vc_j;//gamma velocity correction
        std::vector<double> init_time_j;//breaks
        if(j==0||j==2)
        {
            for (int i = 0; i < N; i++) 
            {
              // init_time_j.push_back(i*.2);05263
                init_time_j.push_back(i*0.02777);//0.05263
                init_l_j.push_back(init_l_vec);//0 0 8.9*9.81
                init_lc_j.push_back(init_l_vec);//0 0 8.9*9.81
                init_vc_j.push_back(VectorXd::Zero(3));//000
            }
        }
        else if(j==1)
        {
            for (int i = 0; i < M; i++) 
            {
              // init_time_j.push_back(i*.2);05263
                init_time_j.push_back(i*0.02632);
                init_l_j.push_back(init_l_vec);//0 0 8.9*9.81
                init_lc_j.push_back(init_l_vec);//0 0 8.9*9.81
                init_vc_j.push_back(VectorXd::Zero(3));//000
            }          
        }
            std::cout << "init_l_j:init_lc_j:init_vc_j.size() "  <<init_l_j.size()<<init_lc_j.size()<<init_vc_j.size()<<std::endl;
            int result3 = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/lammda_planed_outsereach_select112_fixed.txtxxx", 0);//加入先验条件时,需要撤去文件名称的xxx
            int result4 = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/lammda_c_planed_outsereach_select112_fixed.txtxxx", 0);
            int result5 = access("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/gammda_c_planed_outsereach_select112_fixed.txtxxx", 0);
          if(result3==0&&result4==0&&result5==0)
          {
              std::cout<<"exist prior files l lc rc"<<std::endl;
              std::fstream in_lammda_data_file;
              std::fstream in_lammda_c_data_file;
              std::fstream in_gamma_c_data_file;

              in_lammda_data_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/lammda_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::in);
              in_lammda_c_data_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/lammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::in);
              in_gamma_c_data_file.open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/gammda_c_planed_outsereach_select112_fixed.txt", std::ios::out | std::ios::in);
              in_lammda_data_file << std::fixed;
              in_lammda_c_data_file << std::fixed;    
              in_gamma_c_data_file << std::fixed;    
              in_lammda_data_file.seekg(lamda_ptr,std::ios::beg);
              in_lammda_c_data_file.seekg(lamda_c_ptr,std::ios::beg);
              in_gamma_c_data_file.seekg(gamma_c_ptr,std::ios::beg);
              double data_lammda;
              double data_lammda_c;
              double data_gamma_c;
              for(int m=0;m<init_l_j.size();m++)
              {
                  for(int n=0;n<3;n++)
                  {
                      in_lammda_data_file>>data_lammda;
                      init_l_j[m](n)= data_lammda;
                      //std::cout<<"init_l_j[m](n): "<<init_l_j[m](n)<<std::endl;
                  }
              }
              std::cout<<"in_lammda_data_file.tellg()"<<in_lammda_data_file.tellg()<<std::endl;//这里返回的位置作为下一个mode的开始位置,          
              current_lamda_ptr = in_lammda_data_file.tellg();
              for(int m=0;m<init_lc_j.size();m++)
              {
                for(int n=0;n<3;n++)
                {
                  in_lammda_c_data_file>>data_lammda_c;
                  init_lc_j[m](n)= data_lammda_c;
                  std::cout<<"init_lc_j[m](n): "<<init_lc_j[m](n)<<std::endl;
                }
              }
              std::cout<<"in_lammda_c_data_file.tellg()"<<in_lammda_c_data_file.tellg()<<std::endl;   
              current_lamda_c_ptr = in_lammda_c_data_file.tellg();
              for(int m=0;m<init_vc_j.size();m++)
              {
                for(int n=0;n<3;n++)
                {
                  in_gamma_c_data_file>>data_gamma_c;
                  init_vc_j[m](n)= data_gamma_c;
                  //std::cout<<"init_vc_j[m](n): "<<init_vc_j[m](n)<<std::endl;
                }
              }
              std::cout<<"in_gamma_c_data_file.tellg()"<<in_gamma_c_data_file.tellg()<<std::endl;   
              current_gamma_c_ptr = in_gamma_c_data_file.tellg();
              in_gamma_c_data_file.close();
              in_lammda_c_data_file.close();
              in_lammda_data_file.close();
          }

      std::cout << "Baisc-info11111 size of  init_time_j "  <<init_time_j.size()<<std::endl;  
      std::cout << "Baisc-info9 size of init_l_j[0] "  <<init_l_j[0].size()<<std::endl;  
      std::cout << "Baisc-info9 size of init_lc_j[0] "  <<init_lc_j[0].size()<<std::endl;  
      std::cout << "Baisc-info9 size of init_vc_j[0] "  <<init_vc_j[0].size()<<std::endl; 
      //零阶力轨迹 生成轨迹
      // traj_init_l contact forces λ  (interpreted at knot points)
      //traj_init_lc contact λ'  (interpreted at collocation points)
      //traj_init_vc velocity constraint slack variables  γ (at collocation)
      auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);//9+19+9
      auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
      auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

      //三维的轨迹束，共三段，三个多维的轨迹放在一个std::vector里
      init_l_traj.push_back(init_l_traj_j);//两个mode下，9+19段，
      init_lc_traj.push_back(init_lc_traj_j);
      init_vc_traj.push_back(init_vc_traj_j);
      std::cout << "Baisc-info10 Num_modes of init_l_traj  "  << init_l_traj.size()<<std::endl;  
      std::cout << "Baisc-info10 Output shape of init_l_traj_j: nx1 n="<<init_l_traj_j.rows()<<std::endl;         
      std::cout << "Baisc-info10 segments  of init_l_traj at break points: "  <<init_l_traj[j].get_number_of_segments()<<std::endl;
      std::cout << "Baisc-info11 Num_modes of init_lc_traj  "  << init_lc_traj.size()<<std::endl;  
      std::cout << "Baisc-info11 Output shape of init_lc_traj_j: nx1 n="<<init_l_traj_j.rows()<<std::endl;         
      std::cout << "Baisc-info11 segments  of init_lc_traj at break points: "  <<init_lc_traj[j].get_number_of_segments()<<std::endl;
      std::cout << "Baisc-info12 Num_modes of init_vc_traj  "  << init_vc_traj.size()<<std::endl;  
      std::cout << "Baisc-info12 Output shape of init_vc_traj_j: nx1 n="<<init_vc_traj_j.rows()<<std::endl;         
      std::cout << "Baisc-info12 segments  of init_vc_traj at break points: "  <<init_vc_traj[j].get_number_of_segments()<<std::endl;
          std::cout<<"init_l_j.size():"<<init_l_j.size()<<std::endl;     
    }
    std::cout<<std::endl;         

  if (FLAGS_autodiff)
  {
      std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(*plant);
      dairlib::runDircon<drake::AutoDiffXd>(
      std::move(plant_autodiff), plant_vis.get(), std::move(scene_graph),
      FLAGS_strideLength, FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
      } 
  else 
  {
        // FLAGS_duration = 1 FLAGS_strideLength = 0.1
      auto finish0 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_parse0 = finish0 - start0;//解析时长
      std::cout << "Parse0 time:" << elapsed_parse0.count() <<std::endl;
      dairlib::runDircon<double>(
          std::move(plant), plant_vis.get(), std::move(scene_graph),
          1*FLAGS_strideLength, 1*FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
          init_lc_traj, init_vc_traj);
  }
  while(solve_result==0)
  {
        for (int i = 0; i < M+N-1; i++)
        {
            init_x[i]=(x0 + .1*VectorXd::Random(nx));//初始状态轨迹在knotpoint上的值x[i]
            init_u[i]=(VectorXd::Random(nu));//u[i]
       } 
    
        auto init_x_traj_new = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
        auto init_u_traj_new = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);
        for (int j = 0; j < 2; j++) 
        {
            std::vector<MatrixXd> init_l_j;//lammda contraint force
            std::vector<MatrixXd> init_lc_j;//lammda_c force correction
            std::vector<MatrixXd> init_vc_j;//gamma velocity correction
            std::vector<double> init_time_j;//breaks
            int nP=0;
            if(j==0)
            nP=10;
            else if(j==1)
            nP=20;
            for (int i = 0; i < nP; i++) 
            {
               // init_time_j.push_back(i*.2);05263
                init_time_j.push_back(i*0.05263);
                init_l_j.push_back(init_l_vec);//0 0 20*9.81
                init_lc_j.push_back(init_l_vec);//0 0 20*9.81
                init_vc_j.push_back(VectorXd::Zero(3));//000
            }
            auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
            auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
            auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);
            init_l_traj.push_back(init_l_traj_j);//两个mode下，各十段，共二十段，
            init_lc_traj.push_back(init_lc_traj_j);//两个mode下，各十段，
            init_vc_traj.push_back(init_vc_traj_j);//两个mode下，
       }    
           dairlib::runDircon<double>(std::move(plant), plant_vis.get(), std::move(scene_graph),1*FLAGS_strideLength,
                                      1*FLAGS_duration, init_x_traj_new, init_u_traj_new, init_l_traj,init_lc_traj, init_vc_traj);
  }
}/**/