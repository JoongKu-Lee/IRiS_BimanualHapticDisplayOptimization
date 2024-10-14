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
#include "human_tworobot_leftpanda_ikfast_solver.cpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>


#include <boost/scoped_ptr.hpp>



typedef std::chrono::high_resolution_clock Clock;

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double,7,1> Vector7d; 


class Panda
{
public:
	MatrixXd joint_limits;

	Panda()
	{
		MatrixXd joint_limit(7,2); // JOINT LIMIT OF PANDA
		joint_limit.col(0) << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
		joint_limit.col(1) << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
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
		std::vector<IkReal> vfree(panda::GetNumFreeParameters());
		std::vector<Vector7d> ValidSolutions;

		if (panda::GetNumFreeParameters() == 1) // FOR THE CASE OF PANDA
		{
			// for (double freeangle{joint_limits(6,0)}; freeangle <= joint_limits(6,1); freeangle += 0.01) // CONSIDER FREE JOINT 7 WITH CORRESPONDING JOINT LIMITS
			int resolution = 100;
			for (double freeangle{joint_limits(*(panda::GetFreeParameters()), 0)}; freeangle <= joint_limits(*(panda::GetFreeParameters()), 1); freeangle += (joint_limits(*(panda::GetFreeParameters()), 1) - joint_limits(*(panda::GetFreeParameters()), 0)) / static_cast<double>(resolution))
			{
				vfree[0] = freeangle;

				bool bSuccess = panda::ComputeIk(eetrans_des, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

				std::vector<IkReal> solvalues(panda::GetNumJoints());

				for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) 
				{
					const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
					std::vector<IkReal> vsolfree(sol.GetFree().size());
					sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("calculate_score");
    const auto& LOGGER = node->get_logger();
    // RCLCPP_INFO(LOGGER, "calculate_score node started");

    node -> declare_parameter("robot_namespace", rclcpp::PARAMETER_STRING);
    rclcpp::Parameter str_param= node->get_parameter("robot_namespace");

    std::string robot_namespace = str_param.as_string();
    // RCLCPP_INFO(LOGGER, "robot_namespace: %s", robot_namespace.c_str());

	auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>(robot_namespace+"/joint_states", 10);

    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    // RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));



    const moveit::core::JointModelGroup* leftarm_group = kinematic_model->getJointModelGroup("leftarm");
    const moveit::core::JointModelGroup* rightarm_group = kinematic_model->getJointModelGroup("rightarm");
    const moveit::core::JointModelGroup* leftpanda_group = kinematic_model->getJointModelGroup("leftpanda");
    const moveit::core::JointModelGroup* rightpanda_group = kinematic_model->getJointModelGroup("rightpanda");
    const moveit::core::JointModelGroup* all_group = kinematic_model->getJointModelGroup("all");


    const std::vector<std::string>& joint_names = all_group->getVariableNames();

    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_detection::CollisionResult collision_result;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("master_optimization");

    // RCLCPP_INFO(LOGGER, "package_share_directory: %s", package_share_directory.c_str());

    ifstream leftarm_file(package_share_directory  + "/data/leftarm.csv");
    string leftarm_line;

    vector<double> grid_parameters;
    vector<double> leftarm_data;

    int linenum{0};

    while(getline(leftarm_file, leftarm_line))
    {
        string data;
        char linechar[200];
        strcpy(linechar, leftarm_line.c_str());
        char *tok = strtok(linechar, ", ");
        if (linenum == 1)
        {
        while (tok != NULL)
        {
            data = tok;
            tok = strtok(NULL, ", ");
            grid_parameters.push_back(stod(data));
        }
        }
        else if(linenum > 2)
        {
        while (tok != NULL)
        {
            data = tok;
            tok = strtok(NULL, ", ");
            leftarm_data.push_back(stod(data));
        }
        }
        ++linenum;
        // cout<<"linenum: "<<linenum<<endl;
    }

    double x_min = grid_parameters[0];
    double x_max = grid_parameters[1];
    double y_min = grid_parameters[2];
    double y_max = grid_parameters[3];
    double z_min = grid_parameters[4];
    double z_max = grid_parameters[5];
    double increment = grid_parameters[6];
	int sphere_pts = static_cast<int>(grid_parameters[7]);
	int orientation_num = static_cast<int>(grid_parameters[8]);   

    vector<Vector7d> leftarm_poses;
    vector<Vector7d> leftarm_joints;

    for (size_t i{0}; i < (leftarm_data.size() / 14); ++i)
    {
        Vector7d pose, joint;
        pose << leftarm_data[i*14], leftarm_data[i*14+1], leftarm_data[i*14+2], leftarm_data[i*14+3], leftarm_data[i*14+4], leftarm_data[i*14+5], leftarm_data[i*14+6];
        joint << leftarm_data[i*14+7], leftarm_data[i*14+8], leftarm_data[i*14+9], leftarm_data[i*14+10], leftarm_data[i*14+11], leftarm_data[i*14+12], leftarm_data[i*14+13];
        leftarm_poses.push_back(pose);
        leftarm_joints.push_back(joint);

        // RCLCPP_INFO(LOGGER, "left i: %d", i);
    }

    ifstream rightarm_file(package_share_directory  + "/data/rightarm.csv");
    string rightarm_line;

    vector<double> rightarm_data;

    linenum = 0;

    while(getline(rightarm_file, rightarm_line))
    {
        string data;
        char linechar[200];
        strcpy(linechar, rightarm_line.c_str());
        char *tok = strtok(linechar, ", ");
        if(linenum > 2)
        {
            while (tok != NULL)
            {
                data = tok;
                tok = strtok(NULL, ", ");
                rightarm_data.push_back(stod(data));
            }
        }
        ++linenum;
    }

    vector<Vector7d> rightarm_poses;
    vector<Vector7d> rightarm_joints;

    for (size_t i{0}; i < (rightarm_data.size() / 14); ++i)
    {
        Vector7d pose, joint;
        pose << rightarm_data[i*14], rightarm_data[i*14+1], rightarm_data[i*14+2], rightarm_data[i*14+3], rightarm_data[i*14+4], rightarm_data[i*14+5], rightarm_data[i*14+6];
        joint << rightarm_data[i*14+7], rightarm_data[i*14+8], rightarm_data[i*14+9], rightarm_data[i*14+10], rightarm_data[i*14+11], rightarm_data[i*14+12], rightarm_data[i*14+13];
        rightarm_poses.push_back(pose);
        rightarm_joints.push_back(joint);

        // RCLCPP_INFO(LOGGER, "right i: %d", i);

    }

    // cerr << "TEST START !!"<<"LEFT : "<<leftarm_poses.size()<<"RIGHT : "<<rightarm_poses.size()<<endl;

    // RCLCPP_INFO(LOGGER, "LEFT : %d, RIGHT : %d", leftarm_poses.size(), rightarm_poses.size());

    // DATA LOAD FINISHED !!

    
    // PANDA CLASS
    Panda LeftPanda;
    Panda RightPanda;

    const Eigen::Isometry3d& leftpanda_base = kinematic_state->getGlobalLinkTransform("leftpanda_link0");
    const Eigen::Isometry3d& rightpanda_base = kinematic_state->getGlobalLinkTransform("rightpanda_link0");

    //KDL ROBOT SETTING 
    KDL::Tree kdl_tree;

	if (!kdl_parser::treeFromUrdfModel(*kinematic_model->getURDF(), kdl_tree))
    {
        RCLCPP_INFO(LOGGER, "Failed to construct kdl tree");
        return false;
    }

	KDL::Chain leftpanda_kdl_chain, rightpanda_kdl_chain;
	if(!kdl_tree.getChain("world", "leftpanda_link8", leftpanda_kdl_chain))
	{
		return false;
	}	
	if(!kdl_tree.getChain("world", "rightpanda_link8", rightpanda_kdl_chain))
	{
		return false;
	}	

	KDL::Vector gravity = KDL::Vector::Zero();
	gravity(2) = -9.81;

	boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	
	KDL::JntArray G(7), q(7);

    Clock::time_point t1 = Clock::now();

    std::ofstream leftpanda_file, rightpanda_file;
    leftpanda_file.open(package_share_directory + "/data/leftpanda_data"+robot_namespace+".csv");
    rightpanda_file.open(package_share_directory + "/data/rightpanda_data"+robot_namespace+".csv");

	leftpanda_file << "Number of IK, Number of IK Considering Collision, Robot Minimum Force Coverage, Robot Minimum Torque Coverage, Robot Average Force Coverage, Robot Average Torque Coverage, \n";
	rightpanda_file << "Number of IK, Number of IK Considering Collision, Robot Minimum Force Coverage, Robot Minimum Torque Coverage, Robot Average Force Coverage, Robot Average Torque Coverage, \n";

	for (size_t i{0}; i < leftarm_poses.size(); ++i)
	{
		Eigen::Isometry3d desired_position(Eigen::Translation3d(Eigen::Vector3d(leftarm_poses[i](0), leftarm_poses[i](1), leftarm_poses[i](2))));
		Quaterniond quaternion(leftarm_poses[i](6), leftarm_poses[i](3), leftarm_poses[i](4), leftarm_poses[i](5));
		Eigen::Isometry3d desired_orientation(quaternion.toRotationMatrix());

		Eigen::Isometry3d leftpanda_EE = kinematic_state->getGlobalLinkTransform("leftpanda_link8");
		Eigen::Isometry3d leftpanda_TCP = kinematic_state->getGlobalLinkTransform("leftpanda_TCP");
		Eigen::Isometry3d leftpanda_EETOTCP = leftpanda_EE.inverse() * leftpanda_TCP;

		Eigen::Isometry3d leftpanda_basetoEE = leftpanda_base.inverse() * desired_position * desired_orientation * leftpanda_EETOTCP.inverse();

		vector<Vector7d> res_LeftPanda;
		res_LeftPanda = LeftPanda.getAllValidSolutions(leftpanda_basetoEE.translation(), leftpanda_basetoEE.rotation());

		// Collision check
		vector<Vector7d> res_LeftPandaWithoutCollision;
		vector<double> force_coverage, torque_coverage;

		for (size_t j{0}; j < res_LeftPanda.size(); ++j)
		{
			kinematic_state->setToDefaultValues();
			// SET LEFTARM
			kinematic_state->setJointGroupPositions(leftarm_group, leftarm_joints[i]);
			// SET LEFTPANDA
			kinematic_state->setJointGroupPositions(leftpanda_group, res_LeftPanda[j]);
			// CHECK COLLISION
			collision_result.clear();
			planning_scene.checkSelfCollision(collision_request, collision_result, *kinematic_state);

			if (!collision_result.collision)
			{
				res_LeftPandaWithoutCollision.push_back(res_LeftPanda[j]);

				MatrixXd left_Jacob = kinematic_state->getJacobian(leftpanda_group);

				id_solver_.reset( new KDL::ChainDynParam(leftpanda_kdl_chain, gravity) );

				// compute gravity torque
				q(0) = res_LeftPanda[j](0);
				q(1) = res_LeftPanda[j](1);
				q(2) = res_LeftPanda[j](2);
				q(3) = res_LeftPanda[j](3);
				q(4) = res_LeftPanda[j](4);
				q(5) = res_LeftPanda[j](5);
				q(6) = res_LeftPanda[j](6);
				id_solver_->JntToGravity(q, G);

				MatrixXd J_v = left_Jacob.block(0, 0, 3, left_Jacob.cols());
				MatrixXd J_w = left_Jacob.block(3, 0, 3, left_Jacob.cols());

				Vector7d tau_armweight = J_v.transpose() * Vector3d(0, 0, 20);

				JacobiSVD<MatrixXd> SVD_Jv, SVD_Jw;
				SVD_Jv.compute(J_v, Eigen::ComputeThinV | Eigen::ComputeThinU);
				SVD_Jw.compute(J_w, Eigen::ComputeThinV | Eigen::ComputeThinU);

				Vector3d V_v = SVD_Jv.matrixU().col(0);
				Vector3d V_w = SVD_Jw.matrixU().col(0);

				Vector7d robot_torque_lower, robot_torque_upper;
				robot_torque_lower << -87, -87, -87, -87, -12, -12, -12;
				robot_torque_upper << 87, 87, 87, 87, 12, 12, 12;

				robot_torque_lower -= (G.data + tau_armweight);
				robot_torque_upper -= (G.data + tau_armweight);

				force_coverage.push_back((robot_torque_lower.cwiseAbs().cwiseQuotient((J_v.transpose()*V_v).cwiseAbs()).minCoeff(), robot_torque_upper.cwiseAbs().cwiseQuotient((J_v.transpose()*V_v).cwiseAbs()).minCoeff()));
				// RCLCPP_INFO(LOGGER, "force_coverage: %f, %f", force_coverage.back());
				torque_coverage.push_back(min(robot_torque_lower.cwiseAbs().cwiseQuotient((J_w.transpose()*V_w).cwiseAbs()).minCoeff(), robot_torque_upper.cwiseAbs().cwiseQuotient((J_w.transpose()*V_w).cwiseAbs()).minCoeff()));
				// RCLCPP_INFO(LOGGER, "torque_coverage: %f, %f", torque_coverage.back());
			}
		}
		
		if (res_LeftPandaWithoutCollision.size() > 0)
		{
			sensor_msgs::msg::JointState current_joint_state;
			current_joint_state.header.frame_id = "world";
			kinematic_state->setJointGroupPositions(leftarm_group, leftarm_joints[i]);
			kinematic_state->setJointGroupPositions(leftpanda_group, res_LeftPandaWithoutCollision[res_LeftPandaWithoutCollision.size()/2]);
			std::vector<double> joint_values;
			kinematic_state->copyJointGroupPositions(all_group, joint_values);
			for(unsigned int i=0; i<joint_values.size();i++)
			{
				current_joint_state.name.push_back(joint_names[i].c_str());
				current_joint_state.position.push_back(joint_values[i]);
			}

			rclcpp::Time current_time = node->get_clock()->now();
			current_joint_state.header.stamp = current_time;
			joint_state_publisher->publish(current_joint_state);
		}


		if (force_coverage.size() > 0)
		{
			leftpanda_file << res_LeftPanda.size() << "," << res_LeftPandaWithoutCollision.size() << "," << *min_element(force_coverage.begin(), force_coverage.end()) << "," << *min_element(torque_coverage.begin(), torque_coverage.end()) << "," << accumulate(force_coverage.begin(), force_coverage.end(), 0) / static_cast<double>(force_coverage.size()) << "," << accumulate(torque_coverage.begin(), torque_coverage.end(), 0) / static_cast<double>(torque_coverage.size()) <<",\n";
			// RCLCPP_INFO(LOGGER, "left max force coverage: %f, max torque coverage: %f", *max_element(force_coverage.begin(), force_coverage.end()), *max_element(torque_coverage.begin(), torque_coverage.end()));

		}
		else
		{
			leftpanda_file << res_LeftPanda.size() << "," << res_LeftPandaWithoutCollision.size() << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 <<",\n";
		}
	}

	for (size_t i{0}; i < rightarm_poses.size(); ++i)
	{
		Eigen::Isometry3d desired_position(Eigen::Translation3d(Eigen::Vector3d(rightarm_poses[i](0), rightarm_poses[i](1), rightarm_poses[i](2))));
		Quaterniond quaternion(rightarm_poses[i](6), rightarm_poses[i](3), rightarm_poses[i](4), rightarm_poses[i](5));
		Eigen::Isometry3d desired_orientation(quaternion.toRotationMatrix());

		Eigen::Isometry3d rightpanda_EE = kinematic_state->getGlobalLinkTransform("rightpanda_link8");
		Eigen::Isometry3d rightpanda_TCP = kinematic_state->getGlobalLinkTransform("rightpanda_TCP");
		Eigen::Isometry3d rightpanda_EETOTCP = rightpanda_EE.inverse() * rightpanda_TCP;

		Eigen::Isometry3d rightpanda_basetoEE = rightpanda_base.inverse() * desired_position * desired_orientation * rightpanda_EETOTCP.inverse();


		vector<Vector7d> res_RightPanda;
		res_RightPanda = RightPanda.getAllValidSolutions(rightpanda_basetoEE.translation(), rightpanda_basetoEE.rotation());

		// Collision check
		vector<Vector7d> res_RightPandaWithoutCollision;
		vector<double> force_coverage, torque_coverage;

		for (size_t j{0}; j < res_RightPanda.size(); ++j)
		{
			kinematic_state->setToDefaultValues();
			// SET LEFTARM
			kinematic_state->setJointGroupPositions(rightarm_group, rightarm_joints[i]);
			// SET LEFTPANDA
			kinematic_state->setJointGroupPositions(rightpanda_group, res_RightPanda[j]);
			// CHECK COLLISION
			collision_result.clear();
			planning_scene.checkSelfCollision(collision_request, collision_result, *kinematic_state);

			if (!collision_result.collision)
			{
				res_RightPandaWithoutCollision.push_back(res_RightPanda[j]);

				MatrixXd right_Jacob = kinematic_state->getJacobian(rightpanda_group);

				id_solver_.reset( new KDL::ChainDynParam(rightpanda_kdl_chain, gravity) );

				// compute gravity torque
				q(0) = res_RightPanda[j](0);
				q(1) = res_RightPanda[j](1);
				q(2) = res_RightPanda[j](2);
				q(3) = res_RightPanda[j](3);
				q(4) = res_RightPanda[j](4);
				q(5) = res_RightPanda[j](5);
				q(6) = res_RightPanda[j](6);
				id_solver_->JntToGravity(q, G);

				MatrixXd J_v = right_Jacob.block(0, 0, 3, right_Jacob.cols());
				MatrixXd J_w = right_Jacob.block(3, 0, 3, right_Jacob.cols());

				Vector7d tau_armweight = J_v.transpose() * Vector3d(0, 0, 20);

				JacobiSVD<MatrixXd> SVD_Jv, SVD_Jw;
				SVD_Jv.compute(J_v, Eigen::ComputeThinV | Eigen::ComputeThinU);
				SVD_Jw.compute(J_w, Eigen::ComputeThinV | Eigen::ComputeThinU);

				Vector3d V_v = SVD_Jv.matrixU().col(0);
				Vector3d V_w = SVD_Jw.matrixU().col(0);

				Vector7d robot_torque_lower, robot_torque_upper;
				robot_torque_lower << -87, -87, -87, -87, -12, -12, -12;
				robot_torque_upper << 87, 87, 87, 87, 12, 12, 12;

				robot_torque_lower -= (G.data + tau_armweight);
				robot_torque_upper -= (G.data + tau_armweight);

				// RCLCPP_INFO(LOGGER, "robot_torque_lower: %f, %f, %f, %f, %f, %f, %f", robot_torque_lower(0), robot_torque_lower(1), robot_torque_lower(2), robot_torque_lower(3), robot_torque_lower(4), robot_torque_lower(5), robot_torque_lower(6));
				// RCLCPP_INFO(LOGGER, "robot_torque_upper: %f, %f, %f, %f, %f, %f, %f", robot_torque_upper(0), robot_torque_upper(1), robot_torque_upper(2), robot_torque_upper(3), robot_torque_upper(4), robot_torque_upper(5), robot_torque_upper(6));


				// RCLCPP_INFO(LOGGER, "G: %f, %f, %f, %f, %f, %f, %f", G(0), G(1), G(2), G(3), G(4), G(5), G(6));
				// RCLCPP_INFO(LOGGER, "tau_armweight: %f, %f, %f, %f, %f, %f, %f", tau_armweight(0), tau_armweight(1), tau_armweight(2), tau_armweight(3), tau_armweight(4), tau_armweight(5), tau_armweight(6));

				force_coverage.push_back((robot_torque_lower.cwiseAbs().cwiseQuotient((J_v.transpose()*V_v).cwiseAbs()).minCoeff(), robot_torque_upper.cwiseAbs().cwiseQuotient((J_v.transpose()*V_v).cwiseAbs()).minCoeff()));
				torque_coverage.push_back(min(robot_torque_lower.cwiseAbs().cwiseQuotient((J_w.transpose()*V_w).cwiseAbs()).minCoeff(), robot_torque_upper.cwiseAbs().cwiseQuotient((J_w.transpose()*V_w).cwiseAbs()).minCoeff()));
				// RCLCPP_INFO(LOGGER, "force_coverage: %f, %f", force_coverage.back(), torque_coverage.back());
			}
		}


		if (res_RightPandaWithoutCollision.size() > 0)
		{
			sensor_msgs::msg::JointState current_joint_state;
			current_joint_state.header.frame_id = "world";
			kinematic_state->setJointGroupPositions(rightarm_group, rightarm_joints[i]);
			kinematic_state->setJointGroupPositions(rightpanda_group, res_RightPandaWithoutCollision[res_RightPandaWithoutCollision.size()/2]);
			std::vector<double> joint_values;
			kinematic_state->copyJointGroupPositions(all_group, joint_values);
			for(unsigned int i=0; i<joint_values.size();i++)
			{
				current_joint_state.name.push_back(joint_names[i].c_str());
				current_joint_state.position.push_back(joint_values[i]);
			}
			
			rclcpp::Time current_time = node->get_clock()->now();
			current_joint_state.header.stamp = current_time;
			joint_state_publisher->publish(current_joint_state);
		}


		if (force_coverage.size() > 0)
		{
			rightpanda_file << res_RightPanda.size() << "," << res_RightPandaWithoutCollision.size() << "," << *min_element(force_coverage.begin(), force_coverage.end()) << "," << *min_element(torque_coverage.begin(), torque_coverage.end()) << "," << accumulate(force_coverage.begin(), force_coverage.end(), 0) / static_cast<double>(force_coverage.size()) << "," << accumulate(torque_coverage.begin(), torque_coverage.end(), 0) / static_cast<double>(torque_coverage.size()) <<",\n";
			//RCLCPP_INFO(LOGGER, "right max force coverage: %f, max torque coverage: %f", *max_element(force_coverage.begin(), force_coverage.end()), *max_element(torque_coverage.begin(), torque_coverage.end()));
		}
		else
		{
			rightpanda_file << res_RightPanda.size() << "," << res_RightPandaWithoutCollision.size() << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 <<",\n";
		}
		// cerr<<res_RightPandaWithoutCollision.size()<<"/"<<res_RightPanda.size()<<endl;
	}

	Clock::time_point t2 = Clock::now();
	// cerr<< "Overall Calculation took "<< std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()<< "(s)"<< endl;


	leftpanda_file.close();
	rightpanda_file.close();




    //shut down the node
    rclcpp::shutdown();
}