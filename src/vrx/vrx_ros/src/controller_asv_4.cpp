#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "ros_gz_interfaces/msg/param_vec.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Scalar.h>
#include <cmath>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std;

class USVMovementNode : public rclcpp::Node {
public:
    USVMovementNode() : Node("usv_simple_movement") {
        // Constant Variables
        velocity_ASV_set = 0.0;
        omega_ASV_set = 0.0;
        Kp_velo = 500;
        Kp_omega = 500;
        Ki_velo = 0;
        Kd_velo = 100;
        K_Bearing = 15;
        k_alpha = 10;
        k_beta = -10;
        last_time_ = 0.0;
        current_x = 0.0;
        current_y = 0.0;
        current_yaw = 0.0;
        current_velo = 0.0;
        current_omega = 0.0;  
        prev_error_velocity = 0.0;
        prev_error_omega = 0.0;
        integral_velo = 0.0;
        integral_omega = 0.0;
        // Initialize goals vector with values
        goals.push_back({4700, 4700});
        goals.push_back({4700, 4000});
        goals.push_back({4700, 3500});
        goals.push_back({4700, 3000});
        goals.push_back({4500, 3000});
        goal_number = 0;
        latest_bearing_ = 0.0;
        latest_range_ = 0.0;
        search_number = 1;
        in_range = false;
        search_change_x = true;

        // Initialize publishers
        w1_left_pub_1 = create_publisher<std_msgs::msg::Float64>("/wamv_m_4/thrusters/left/thrust", 1);
        w2_right_pub_1 = create_publisher<std_msgs::msg::Float64>("/wamv_m_4/thrusters/right/thrust", 1);
        angle_r1_pub_1 = create_publisher<std_msgs::msg::Float64>("/wamv_m_4/thrusters/left/pos", 1);
        angle_r2_pub_1 = create_publisher<std_msgs::msg::Float64>("/wamv_m_4/thrusters/right/pos", 1);

        // Initialize subscribers
        range_boat_subscriber = this->create_subscription<ros_gz_interfaces::msg::ParamVec>(
            "/wamv_m_4/sensors/acoustics/receiver/range_bearing", 1,
            std::bind(&USVMovementNode::paramVecCallback, this, std::placeholders::_1));

        // Initialize subscribers
        pose_boat_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wamv_m_4/sensors/position/ground_truth_odometry", 1,
            std::bind(&USVMovementNode::poseBoatCallback, this, std::placeholders::_1));

        // // // Initialize sub for target positon
        target_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target/particle_filter_estimated_location", 5,
            std::bind(&USVMovementNode::targetPoseCallback, this, std::placeholders::_1));
    }

private:
    // Member variables to store constants
    double velocity_ASV_set;
    double omega_ASV_set;
    double Kp_velo;
    double Kp_omega;
    double Ki_velo;
    double Kd_velo;
    double K_Bearing;
    double k_alpha;
    double k_beta;
    double last_time_;
    double current_x;
    double current_y;
    double current_yaw;
    double current_velo;
    double current_omega;
    double prev_error_velocity;
    double prev_error_omega;
    double integral_velo;
    double integral_omega;
    double latest_bearing_;
    double latest_range_;
    std::vector<std::vector<double>> goals;
    int goal_number;
    int search_number;
    bool in_range;
    bool search_change_x;
    bool new_goal = true;
    double velo_max = 4.5;
    

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr w1_left_pub_1;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr w2_right_pub_1;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_r1_pub_1;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_r2_pub_1;

    // Subscriber
    rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr range_boat_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_boat_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber;

    // // Timer for control loop
    // rclcpp::TimerBase::SharedPtr timer_;

    void paramVecCallback(const ros_gz_interfaces::msg::ParamVec::SharedPtr msg)
    {
        // Process the received message
        double range_value = msg->params[2].value.double_value; 
        double bearing_value = msg->params[1].value.double_value;   
        double sim_time_s = msg->header.stamp.sec + (msg->header.stamp.nanosec)/10^9;
        
        // //save beaning and range to variables for controller to use for logic calcs.
        // latest_range_ = range_value;
        // latest_bearing_ = bearing_value;
    }


    void poseBoatCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
            // Process the received message
        double current_time = msg->header.stamp.sec + (msg->header.stamp.nanosec) / std::pow(10.0, 9);
        
        //Subcribe to pose info.
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

        cout<<"Current X: "<< current_x << endl;
        cout<<"Current Y: "<< current_y << endl;

        //Subcribe to Quaternion data
        geometry_msgs::msg::Quaternion quaternion_msg = msg->pose.pose.orientation;

        // Transform the quaternion to YPR
        tf2::Quaternion tf_quaternion;
        tf2::fromMsg(quaternion_msg, tf_quaternion);
        tf2Scalar yaw, pitch, roll;
        tf2::Matrix3x3(tf_quaternion).getEulerYPR(yaw, pitch, roll);

        current_yaw = static_cast<double>(yaw);

        current_velo = msg->twist.twist.linear.x;
        current_omega = msg->twist.twist.angular.z; 

        //Send Pose to Velocity Controller.
        setpointValues();

        
        // Do something with the calculated velocities
        thursterController();
        
        // startSearch();     
    }

    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Process the received message
        double x_goal = msg->pose.position.x;
        double y_goal = msg->pose.position.y;
        double z_goal = msg->pose.position.z;

        double euclidean_distance = 100000 + std::hypot(x_goal - current_x, y_goal - current_y);
        
        if (euclidean_distance < 10000)
        {
            in_range = true;
            goals[goal_number][0] = x_goal;
            goals[goal_number][1] = y_goal;
        }
        else if (euclidean_distance > 10000)
        {
            in_range = false;
        }

    }

    void startSearch() {
        // Implement the expanding square spiral search algorithm here
        // Generate setpoints and set them continuously until a detection is made
        // Update goals vector with the generated setpoints
        // Example:
        // Generate and set new setpoints
        int check_number = goal_number + 1;
        cout<<"Check Number: "<< goal_number << endl;
        if (new_goal) {
            generateNextSetpoint();
            cout << "New setpoint: " << goals[goal_number][0] << ", " << goals[goal_number][1] << endl;
        };
        new_goal = false;

        cout<<"Current X Goal: "<< goals[goal_number][0] << endl;
        cout<<"Current Y Goal: "<< goals[goal_number][1] << endl;


        setpointValues(); // Update setpoints
        thursterController(); // Move towards the setpoints
        
        // Switch to tracking mode when detection is made
        if (in_range){
            startTracking();
        }

    }

    void startTracking() {
        // Implement the tracking logic here
        // Move towards the detected target while tracking it

        setpointValues(); // Update setpoints
        thursterController(); // Move towards the setpoints
        rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Adjust sleep duration as needed
    }

    void generateNextSetpoint() {
        // Implement the expanding square spiral search pattern to generate next setpoint
        // Update goals vector with the generated setpoints

        int leg_length = 1;

        if (search_number % 2 == 0) {
            leg_length = search_number;
        } else {
            leg_length = -search_number;;
        }
    
        if (search_change_x) {
            goals[goal_number][0] = current_x + 50*leg_length;
            goals[goal_number][1] = current_y;
            search_change_x = false;
            cout<<"X: "<< goals[goal_number][0] << endl;
            cout<<"Y: "<< goals[goal_number][1] << endl;
        } else {
            goals[goal_number][1] = current_y + 50*leg_length;
            goals[goal_number][0] = current_x;
            search_change_x = true;
            search_number += 1;
        }

    }

    

    double circle_minus(double angle_1) {
        // Calculates the number of rotations past the E[-pi, pi].
        double num = std::floor(std::abs(angle_1 / (2 * M_PI)));

        // Check to see difference is the angles is positive or negative.
        double angle;
        if (angle_1 > 0) {
            angle = angle_1 - (num * 2 * M_PI);
        } else {
            angle = angle_1 + (num * 2 * M_PI);
        }

        // Checks the angle to ensure the smallest rotation is presented.

        if (angle > M_PI) {
            angle = (angle - 2 * M_PI);
        } else if (angle < -M_PI) {
            angle = (angle + 2 * M_PI);
        }
        return angle;
    }

    void setpointValues() {
        // Calculate distances

        double d_x_boat = goals[goal_number][0] - current_x;
        double d_y_boat = goals[goal_number][1] - current_y;
        double rho = sqrt(pow(d_y_boat, 2) + pow(d_x_boat, 2));

        cout<<"range to goal is " << rho << endl;

        // Control Law to Calc Velocity
        double velocity_hold = K_Bearing * rho;

        if (velocity_hold > velo_max){
            velocity_hold = velo_max;
        }else if (velocity_hold < 1)
        {
            velocity_hold = 1;
        }
        
        double alpha_hold = -current_yaw + atan2(d_y_boat, d_x_boat);
        alpha_hold = circle_minus(alpha_hold);

        // double beta_hold = -bearing - alpha_hold;
        double beta_hold = 0;
        beta_hold = circle_minus(beta_hold);

        double omega_hold = k_alpha * alpha_hold + k_beta * beta_hold;

        cout<<"Bearning Error: "<< alpha_hold << endl;
        cout<<"atan2 Error: "<< atan2(d_y_boat, d_x_boat) << endl;


        // if (omega_hold > 1){
        //     omega_hold = 1;
        // }else if (omega_hold < -1)
        // {
        //     omega_hold < -1;
        // }
        
        // Assign the calculated values
        velocity_ASV_set = velocity_hold;
        omega_ASV_set = omega_hold;
        latest_range_ = rho;
    }


    void thursterController()
    {
        // if (latest_range_ < 10) {
        //     velocity_ASV_set = 0;
        //     omega_ASV_set = 0;
        //     new_goal = true;
        // }

        if (latest_range_ < 20) {
            velocity_ASV_set = 0;
            goal_number += 1;
        }

        

        double error_velocity_x =  velocity_ASV_set - current_velo;
        integral_velo += error_velocity_x;
        double derivative_velo = error_velocity_x - prev_error_velocity;

        cout<<"Velocity Error"<< error_velocity_x << endl;

 
        double thurst_velo = error_velocity_x*Kp_velo + Ki_velo * integral_velo + Kd_velo * derivative_velo;

        double error_omega_ =  omega_ASV_set - current_omega;
        integral_omega += error_omega_;
        double derivative_omega = error_omega_ - prev_error_omega;

        cout<<"omega Error"<< error_omega_ << endl;

 
        double turing_velo_ = error_omega_*Kp_omega + Kd_velo * derivative_omega;
        // double turing_velo_;

        
        // if (thurst_velo < 100){
        //     double turing_velo_ = 0;
        // } else{
        //     double turing_velo_ = omega_ASV_set*K_Bearing;
        // }
        

        double thurst_velo_left_ = thurst_velo - turing_velo_;
        double thurst_velo_right_ = thurst_velo + turing_velo_;

        std_msgs::msg::Float64 thurst_velo_msg_left_;
        std_msgs::msg::Float64 thurst_velo_msg_right_;

        thurst_velo_msg_left_.data = thurst_velo_left_;
        thurst_velo_msg_right_.data = thurst_velo_right_;
        // std_msgs::msg::Float32 angle_r1_msg;
        // std_msgs::msg::Float32 angle_r2_msg;

        // Populate messages with data...

        w1_left_pub_1->publish(thurst_velo_msg_left_);
        w2_right_pub_1->publish(thurst_velo_msg_right_);


        // angle_r1_pub->publish(angle_r1_msg);
        // angle_r2_pub->publish(angle_r2_msg);

        prev_error_velocity = error_velocity_x;
        prev_error_omega = error_omega_;
    }


//==================================================================================================
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<USVMovementNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
