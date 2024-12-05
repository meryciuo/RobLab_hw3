// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            declare_parameter("cmd", "positioning");    
            get_parameter("cmd", cmd);
            RCLCPP_INFO(get_logger(),"Current cmd is: '%s'", cmd.c_str());


            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
            }

            if (!(cmd == "positioning" || cmd == "look_at_point"))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd is not valid! Use 'positioning', 'look_at_point'"); return;
            }

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 
            camera_available_ = false;

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_.data.setZero();

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));
            
            ArucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

           // Wait for the joint_state topic
            while(!camera_available_){
                RCLCPP_INFO(this->get_logger(), "No camera received yet! ...");
                rclcpp::spin_some(node_handle_);
            }


            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();

            init_position_ = robot_->getJntValues();

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0));
            Eigen::Vector3d end_position;
           
            KDL::Frame Camera_pose (Aruco.M,KDL::Vector(Aruco.p.data[0]+0.7,Aruco.p.data[1],Aruco.p.data[2])); //da mettere offset
            KDL::Frame Aruco_pose_(Aruco.M,KDL::Vector(Aruco.p.data[0],Aruco.p.data[1],Aruco.p.data[2])); 
            Aruco_p=Aruco_pose_;

            std::cout<<Camera_pose<<std::endl;

            end_position = toEigen(Camera_pose.p);
    
            // Plan trajectory
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
            
            // Retrieve the first trajectory point
            trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }
            } 

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; 
            int trajectory_len = 150; 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;


            if (cmd=="positioning"){

                if (t_ < total_time){

                    // Retrieve the trajectory point
                    trajectory_point p = planner_.compute_trajectory(t_); 

                    // Compute EE frame
                    KDL::Frame cartpos = robot_->getEEFrame();           

                    // Compute desired Frame
                    KDL::Frame desFrame; desFrame.M = Aruco.M; desFrame.p = toKDL(p.pos); 

                    // compute errors
                    Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));

                    Eigen::Matrix3d R_z_180;

                    R_z_180 << -1,  0,  0, 
                                0,  1,  0, 
                                0,  0,  -1;

                    Eigen::Matrix3d Aruco_R = toEigen(Aruco.M);

                    Eigen::Matrix3d Aruco_R_rotated = R_z_180 * Aruco_R;


                    Eigen::Vector3d o_error = computeOrientationError(Aruco_R_rotated, toEigen(cartpos.M));
                    //std::cout << "The error norm is : " << error.norm() << std::endl;
                    std::cout << "The o_error norm is : " << o_error.norm() << std::endl;

                    if(cmd_interface_ == "position"){
                        // Next Frame
                        KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                        // Compute IK
                        joint_positions_cmd_ = joint_positions_;
                        robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                    }
                    else if(cmd_interface_ == "velocity"){

                            // Compute differential IK
                        Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                       
                    }
                    
                    else if(cmd_interface_ == "effort"){
                        joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                    }

                    // Update KDLrobot structure
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                    if(cmd_interface_ == "position"){
                        // Set joint position commands
                        for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                            desired_commands_[i] = joint_positions_cmd_(i);
                        }
                    }
                    else if(cmd_interface_ == "velocity"){
                        // Set joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_velocities_cmd_(i);
                        }
                    }
                    else if(cmd_interface_ == "effort"){
                        // Set joint effort commands
                        for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                            desired_commands_[i] = joint_efforts_cmd_(i);
                        }
                    } 

                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                }


                else{
                    RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

            
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                    
                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                }

            }

            else if (cmd=="look_at_point"){

                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_); 

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = Aruco.M; desFrame.p = toKDL(p.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));

                Eigen::Matrix3d R_z_180;

                R_z_180 << -1,  0,  0, 
                            0,  1,  0,
                            0,  0, -1;

                Eigen::Matrix3d Aruco_R = toEigen(Aruco.M);

                Eigen::Matrix3d Aruco_R_rotated = R_z_180 * Aruco_R;


                Eigen::Vector3d o_error = computeOrientationError(Aruco_R_rotated, toEigen(cartpos.M));
                //std::cout << "The error norm is : " << error.norm() << std::endl;
                //std::cout << "The o_error norm is : " << o_error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){

                    Eigen::VectorXd dq = robot_->getJntValues();

                    Eigen::VectorXd q0 = init_position_ - dq;

                    KDL::Jacobian J_cam = robot_->getEEJacobian();

                    Eigen::Vector3d s = toEigen(Aruco.p/Aruco.p.Norm());

                    Eigen::Matrix3d L1 = -1/Aruco.p.Norm()*(Eigen::Matrix3d::Identity()-s*s.transpose());

                    Eigen::Matrix3d S = skew(s);

                    Eigen::Matrix < double, 6 ,6 > R = Eigen::Matrix <double , 6, 6 >::Zero();

                    KDL::Rotation rotation = Aruco.M;

                    Eigen::Matrix3d rotation_matrix = Eigen::Map<Eigen::Matrix3d>(rotation.data);

                    R.block(0,0,3,3) = rotation_matrix;

                    R.block(3,3,3,3) = rotation_matrix;

                    Eigen::Matrix < double,3,6> L2 = Eigen::Matrix <double , 3, 6 >::Zero();

                    L2.block(0,0,3,3) = L1;

                    L2.block(0,3,3,3) = S;

                    Eigen::Matrix < double,3,6> L = L2*R.transpose();

                    Eigen::Vector3d s_d (0,0,1);

                    Eigen::Matrix < double,3,7> L_J = L*J_cam.data;

                    Eigen::Matrix < double,7,3> pseudoLJ = pseudoinverse(L_J);

                    Eigen::Matrix < double,7,7> N = Eigen::Matrix<double,7,7> :: Identity() - pseudoLJ*L_J;

                    joint_velocities_cmd_.data = 2*pseudoLJ*s_d+8*N*q0; 
                
                }
                    
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }


        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        void aruco_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

            camera_available_ = true;

            double x_aruco = msg->pose.position.x;
            double y_aruco = msg->pose.position.y;
            double z_aruco = msg->pose.position.z;

            double qx_aruco = msg->pose.orientation.x;
            double qy_aruco = msg->pose.orientation.y;
            double qz_aruco = msg->pose.orientation.z;
            double qw_aruco = msg->pose.orientation.w;

            Aruco.p = KDL::Vector(x_aruco, y_aruco, z_aruco);
            Aruco.M = KDL::Rotation::Quaternion(qx_aruco, qy_aruco, qz_aruco, qw_aruco); 

        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ArucoSubscriber_;

        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_efforts_;
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        KDL::Frame Aruco_p;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        Eigen::VectorXd init_position_;

        int iteration_;
        bool joint_state_available_;
        bool camera_available_;
        double t_;
        double q0;
        double K=2000;
        std::string cmd_interface_;
        std::string cmd;
        KDL::Frame init_cart_pose_;

        KDL::Frame Aruco;


};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}