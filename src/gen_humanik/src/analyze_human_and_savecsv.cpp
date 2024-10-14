
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_scene/planning_scene.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"


#include "human_upper_body_human_left_temp_ikfast_solver.cpp"
#include "human_upper_body_human_right_temp_ikfast_solver.cpp"


using namespace std;
using namespace Eigen;

typedef std::chrono::high_resolution_clock Clock;


vector<Matrix3d> GetOrientationCandidate_Superfibonacci(const int &n)
{
	vector<Matrix3d> res;

	const double sn = 1.533751168755204288118041;
	double mc0 = 1.0/sqrt(2.0);
	double mc1 = 1.0/sn;

	double s,ab,r,R,theta,phi;
	double dn = 1.0/(double)n;
	for (int i = 0; i < n; i++)
	{
			s = (double)i+0.5;
			ab = 2.0 * M_PI * s;
			theta = ab * mc0;
			phi = ab * mc1;
			s *= dn;
			r = sqrt(s);
			R = sqrt(1.0-s);
			
			Quaterniond quat(r*cos(theta), R*sin(phi), R*cos(phi), r*sin(theta));

			res.push_back(quat.toRotationMatrix());
	}
	return res;
}

typedef Eigen::Matrix<double,7,1> Vector7d; 

class LeftArm
{
public:
    MatrixXd joint_limits;
    LeftArm()
    {
        MatrixXd joint_limit(7,2); // JOINT LIMIT OF PANDA
        joint_limit.col(0) << -0.5236, -1.5708, -1.3962, 0, -1.4835, -1.4835, -0.2618;
        joint_limit.col(1) << 3.14159, 0.8727, 1.3962, 2.7925, 1.5708, 1.4835, 0.7854;

        joint_limits = joint_limit;
    }

    bool CheckJointLimit(const Vector7d &jointangles)
    {
        bool in_joint_limit = true;
        for (int i{0}; i < jointangles.size(); ++i)
        {
            if (jointangles(i) <= joint_limits(i,0) || jointangles(i) >= joint_limits(i,1))
            {
                // cout<<"JOINT LIMIT DETECTED!!"<<endl;
                in_joint_limit = false;
            }
        }
        return in_joint_limit;
    }

    std::vector<Vector7d> getAllValidSolutions(const Vector3d &desired_position, const Matrix3d &desired_orientation)
    {
        IkReal eetrans_des[3] = {desired_position(0), desired_position(1), desired_position(2)};
        IkReal eerot[9] = {desired_orientation(0,0), desired_orientation(0,1), desired_orientation(0,2), 
                           desired_orientation(1,0), desired_orientation(1,1), desired_orientation(1,2), 
                           desired_orientation(2,0), desired_orientation(2,1), desired_orientation(2,2)};

        IkSolutionList<IkReal> solutions;
        std::vector<IkReal> vfree(leftarm::GetNumFreeParameters());
        std::vector<Vector7d> ValidSolutions;

        if (leftarm::GetNumFreeParameters() == 1) // FOR THE CASE OF HUMAN_LEFTARM
        {
            // for (double freeangle{joint_limits(*(leftarm::GetFreeParameters()), 0)}; freeangle <= joint_limits(*(leftarm::GetFreeParameters()), 1); freeangle += 0.01)
			int resolution = 100;
            for (double freeangle{joint_limits(*(leftarm::GetFreeParameters()), 0)}; freeangle <= joint_limits(*(leftarm::GetFreeParameters()), 1); freeangle += (joint_limits(*(leftarm::GetFreeParameters()), 1) - joint_limits(*(leftarm::GetFreeParameters()), 0)) / static_cast<double>(resolution))
            {
                vfree[0] = freeangle;

                bool bSuccess = leftarm::ComputeIk(eetrans_des, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

                std::vector<IkReal> solvalues(leftarm::GetNumJoints());
                for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) 
                {
                    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
                    std::vector<IkReal> vsolfree(sol.GetFree().size());
                    sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

                    // - JOINT LIMIT CHECK!!
                    Vector7d Solution;
                    Solution << solvalues[0], solvalues[1], solvalues[2], solvalues[3], solvalues[4], solvalues[5], solvalues[6];

                    if(CheckJointLimit(Solution))
                    {
                        ValidSolutions.push_back(Solution);
                    }
                }
            }
        }
        return ValidSolutions;
    }

};

class RightArm
{
public:
    MatrixXd joint_limits;
    RightArm()
    {
        MatrixXd joint_limit(7,2); // JOINT LIMIT OF PANDA
        joint_limit.col(0) << -0.5236, -0.8727, -1.3962, 0, -1.5708, -1.4835, -0.7854;
        joint_limit.col(1) << 3.14159, 1.5708, 1.3962, 2.7925, 1.4835, 1.4835, 0.2618;

        joint_limits = joint_limit;
    }

    bool CheckJointLimit(const Vector7d &jointangles)
    {
        bool in_joint_limit = true;
        for (int i{0}; i < jointangles.size(); ++i)
        {
            if (jointangles(i) <= joint_limits(i,0) || jointangles(i) >= joint_limits(i,1))
            {
                // cout<<"JOINT LIMIT DETECTED!!"<<endl;
                in_joint_limit = false;
            }
        }
        return in_joint_limit;
    }

    std::vector<Vector7d> getAllValidSolutions(const Vector3d &desired_position, const Matrix3d &desired_orientation)
    {
        IkReal eetrans_des[3] = {desired_position(0), desired_position(1), desired_position(2)};
        IkReal eerot[9] = {desired_orientation(0,0), desired_orientation(0,1), desired_orientation(0,2), 
                           desired_orientation(1,0), desired_orientation(1,1), desired_orientation(1,2), 
                           desired_orientation(2,0), desired_orientation(2,1), desired_orientation(2,2)};

        IkSolutionList<IkReal> solutions;
        std::vector<IkReal> vfree(rightarm::GetNumFreeParameters());
        std::vector<Vector7d> ValidSolutions;

        if (rightarm::GetNumFreeParameters() == 1) // FOR THE CASE OF HUMAN_RIGHTARM
        {
            // for (double freeangle{joint_limits(*(rightarm::GetFreeParameters()), 0)}; freeangle <= joint_limits(*(rightarm::GetFreeParameters()), 1); freeangle += 0.01)
			int resolution = 100;
            for (double freeangle{joint_limits(*(rightarm::GetFreeParameters()), 0)}; freeangle <= joint_limits(*(rightarm::GetFreeParameters()), 1); freeangle += (joint_limits(*(rightarm::GetFreeParameters()), 1) - joint_limits(*(rightarm::GetFreeParameters()), 0)) / static_cast<double>(resolution))
            {
                vfree[0] = freeangle;

                bool bSuccess = rightarm::ComputeIk(eetrans_des, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

                std::vector<IkReal> solvalues(rightarm::GetNumJoints());
                for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) 
                {
                    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
                    std::vector<IkReal> vsolfree(sol.GetFree().size());
                    sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

                    // - JOINT LIMIT CHECK!!
                    Vector7d Solution;
                    Solution << solvalues[0], solvalues[1], solvalues[2], solvalues[3], solvalues[4], solvalues[5], solvalues[6];

                    if(CheckJointLimit(Solution))
                    {
                        ValidSolutions.push_back(Solution);
                    }
                }
            }
        }
        return ValidSolutions;
    }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node =rclcpp::Node::make_shared("analyze", node_options);
  const auto& LOGGER = node->get_logger();


  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));


  const moveit::core::JointModelGroup* leftarm_group = kinematic_model->getJointModelGroup("leftarm");
  const moveit::core::JointModelGroup* rightarm_group = kinematic_model->getJointModelGroup("rightarm");
  const moveit::core::JointModelGroup* all_group = kinematic_model->getJointModelGroup("all");

  const std::vector<std::string>& leftarm_joint_names = leftarm_group->getVariableNames();
  const std::vector<std::string>& rightarm_joint_names = rightarm_group->getVariableNames();
  const std::vector<std::string>& all_joint_names = all_group->getVariableNames();


  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_detection::CollisionResult collision_result;


  // //print current joint state
  // const moveit::core::RobotState& current_state = planning_scene.getCurrentState();
  // std::vector<double> joint_values;
  // current_state.copyJointGroupPositions(all_group, joint_values);
  // RCLCPP_INFO(LOGGER, "Current Joint Values: ");
  // for (std::size_t i = 0; i < all_joint_names.size(); ++i)
  // {
  //   RCLCPP_INFO(LOGGER, "  %s: %f", all_joint_names[i].c_str(), joint_values[i]);
  // }

  double x_min = -0.2;
  double x_max = 0.7;
  double y_min = -1.0;
  double y_max = 1.0;
  double z_min = -0.2;
  double z_max = 1.5;

  double increment = 0.1;

  int sphere_pts = 50;
  int orientation_num = 20;

  int num = 50;

  vector<Matrix3d> OrientationCandidate = GetOrientationCandidate_Superfibonacci(num);
  
  //define package directory not shared
  std::string package_directory = ament_index_cpp::get_package_prefix("gen_humanik");
  RCLCPP_INFO(LOGGER, "Package Directory: %s", package_directory.c_str());

  //creat folder to save data
  std::string data_directory = package_directory + "/data";
  std::string command = "mkdir -p " + data_directory;
  system(command.c_str());

  std::ofstream leftarm_file, rightarm_file;
  leftarm_file.open(package_directory + "/data/leftarm.csv");
  rightarm_file.open(package_directory + "/data/rightarm.csv");

  leftarm_file << "x_min, x_max, y_min, y_max, z_min, z_max, position_increment, sphere_pts, orientation_num, \n";
  leftarm_file << x_min <<"," << x_max <<"," << y_min <<"," << y_max <<"," << z_min <<"," << z_max <<"," << increment <<"," << sphere_pts <<"," << orientation_num <<",\n"; 
  leftarm_file << "x, y, z, qx, qy, qz, qw, j1, j2, j3, j4, j5, j6, j7, \n";
  rightarm_file << "x_min, x_max, y_min, y_max, z_min, z_max, position_increment, sphere_pts, orientation_num, \n";
  rightarm_file << x_min <<"," << x_max <<"," << y_min <<"," << y_max <<"," << z_min <<"," << z_max <<"," << increment <<"," << sphere_pts <<"," << orientation_num <<",\n"; 
  rightarm_file << "x, y, z, qx, qy, qz, qw, j1, j2, j3, j4, j5, j6, j7, \n";

  Clock::time_point t1 = Clock::now();

  LeftArm leftarm;
  RightArm rightarm;

  for (double i = x_min; i < x_max; i += increment)
  {
    for (double j = y_min; j < y_max; j+= increment)
    {
      for (double k = z_min; k < z_max; k += increment)
      {
        Eigen::Isometry3d desired_position(Eigen::Translation3d(Eigen::Vector3d(i, j, k)));

        for (int t = 0; t < OrientationCandidate.size(); ++t)
        {
          Eigen::Isometry3d desired_orientation(OrientationCandidate[t]);
          Eigen::Isometry3d desired_pose = desired_position * desired_orientation;

          const Eigen::Isometry3d& leftarm_base = kinematic_state->getGlobalLinkTransform("LeftShoulder");
          const Eigen::Isometry3d& rightarm_base = kinematic_state->getGlobalLinkTransform("RightShoulder");

          Eigen::Isometry3d leftarm_basetoEE = leftarm_base.inverse() * desired_pose;
          Eigen::Isometry3d rightarm_basetoEE = rightarm_base.inverse() * desired_pose;

          vector<Vector7d> res_LeftArm, res_RightArm, res_LeftArmWithoutCollision, res_RightArmWithoutCollision;

          res_LeftArm = leftarm.getAllValidSolutions(leftarm_basetoEE.translation(), leftarm_basetoEE.rotation());
          res_RightArm = rightarm.getAllValidSolutions(rightarm_basetoEE.translation(), rightarm_basetoEE.rotation());

          int left_max_manipulability_idx{0}, right_max_manipulability_idx{0};
          double left_max_manipulability{0.0}, right_max_manipulability{0.0};

          if (res_LeftArm.size() > 0) // IF LEFT HAS SOLUTION
          {
            // CHECK COLLISION
            for (int p = 0; p < res_LeftArm.size(); ++p)
            {
              kinematic_state->setToDefaultValues();
              kinematic_state->setJointGroupPositions(leftarm_group, res_LeftArm[p]);

              collision_result.clear();
              planning_scene.checkSelfCollision(collision_request, collision_result, *kinematic_state);

              if (!collision_result.collision)
              {
                res_LeftArmWithoutCollision.push_back(res_LeftArm[p]);
                
                MatrixXd leftarm_Jacob = kinematic_state->getJacobian(leftarm_group);
                double Manipulability = sqrt((leftarm_Jacob * leftarm_Jacob.transpose()).determinant());
                if (left_max_manipulability < Manipulability)
                {
                  left_max_manipulability = Manipulability;
                  left_max_manipulability_idx = res_LeftArmWithoutCollision.size()-1;
                }
              }
            }
          }

          if (res_RightArm.size() > 0) // IF RIGHT HAS SOLUTION
          {
            // CHECK COLLISION
            for (int p = 0; p < res_RightArm.size(); ++p)
            {
              kinematic_state->setToDefaultValues();
              kinematic_state->setJointGroupPositions(rightarm_group, res_RightArm[p]);

              collision_result.clear();
              planning_scene.checkSelfCollision(collision_request, collision_result, *kinematic_state);

              if (!collision_result.collision)
              {
                res_RightArmWithoutCollision.push_back(res_RightArm[p]);

                MatrixXd rightarm_Jacob = kinematic_state->getJacobian(rightarm_group);
                double Manipulability = sqrt((rightarm_Jacob * rightarm_Jacob.transpose()).determinant());
                if (right_max_manipulability < Manipulability)
                {
                  right_max_manipulability = Manipulability;
                  right_max_manipulability_idx = res_RightArmWithoutCollision.size()-1;
                }
              }
            }
          }
          
          if (res_LeftArmWithoutCollision.size() > 0)
          {
            Quaterniond desired_quat(desired_pose.linear());
            leftarm_file << desired_pose.translation()(0) << "," << desired_pose.translation()(1) << "," << desired_pose.translation()(2) << ","
                  << desired_quat.x() << "," << desired_quat.y() << "," << desired_quat.z() << "," << desired_quat.w() << ","
                  << res_LeftArmWithoutCollision[left_max_manipulability_idx](0) << "," << res_LeftArmWithoutCollision[left_max_manipulability_idx](1) << "," << res_LeftArmWithoutCollision[left_max_manipulability_idx](2) << "," << res_LeftArmWithoutCollision[left_max_manipulability_idx](3) << "," << res_LeftArmWithoutCollision[left_max_manipulability_idx](4) << "," << res_LeftArmWithoutCollision[left_max_manipulability_idx](5) << "," << res_LeftArmWithoutCollision[left_max_manipulability_idx](6) << ",\n";
                  // << res_LeftArmWithoutCollision[res_LeftArmWithoutCollision.size()/2](0) << "," << res_LeftArmWithoutCollision[res_LeftArmWithoutCollision.size()/2](1) << "," << res_LeftArmWithoutCollision[res_LeftArmWithoutCollision.size()/2](2) << "," << res_LeftArmWithoutCollision[res_LeftArmWithoutCollision.size()/2](3) << "," << res_LeftArmWithoutCollision[res_LeftArmWithoutCollision.size()/2](4) << "," << res_LeftArmWithoutCollision[res_LeftArmWithoutCollision.size()/2](5) << "," << res_LeftArmWithoutCollision[res_LeftArmWithoutCollision.size()/2](6) << ",\n";
          }
          if (res_RightArmWithoutCollision.size() > 0)
          {
            Quaterniond desired_quat(desired_pose.linear());
            rightarm_file << desired_pose.translation()(0) << "," << desired_pose.translation()(1) << "," << desired_pose.translation()(2) << ","
                  << desired_quat.x() << "," << desired_quat.y() << "," << desired_quat.z() << "," << desired_quat.w() << ","
                  << res_RightArmWithoutCollision[right_max_manipulability_idx](0) << "," << res_RightArmWithoutCollision[right_max_manipulability_idx](1) << "," << res_RightArmWithoutCollision[right_max_manipulability_idx](2) << "," << res_RightArmWithoutCollision[right_max_manipulability_idx](3) << "," << res_RightArmWithoutCollision[right_max_manipulability_idx](4) << "," << res_RightArmWithoutCollision[right_max_manipulability_idx](5) << "," << res_RightArmWithoutCollision[right_max_manipulability_idx](6) << ",\n";
                  // << res_RightArmWithoutCollision[res_RightArmWithoutCollision.size()/2](0) << "," << res_RightArmWithoutCollision[res_RightArmWithoutCollision.size()/2](1) << "," << res_RightArmWithoutCollision[res_RightArmWithoutCollision.size()/2](2) << "," << res_RightArmWithoutCollision[res_RightArmWithoutCollision.size()/2](3) << "," << res_RightArmWithoutCollision[res_RightArmWithoutCollision.size()/2](4) << "," << res_RightArmWithoutCollision[res_RightArmWithoutCollision.size()/2](5) << "," << res_RightArmWithoutCollision[res_RightArmWithoutCollision.size()/2](6) << ",\n";
          }



        }

        cerr <<i <<"," << j <<"," << k << endl;
        
        }
      }
    }
  Clock::time_point t2 = Clock::now();
  cerr<<"Overall Time: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()<<endl;

  leftarm_file.close();
  rightarm_file.close();
  return 0;
}