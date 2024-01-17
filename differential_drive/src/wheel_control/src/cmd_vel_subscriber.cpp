#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
using namespace std;

// Constants
const double max_pwm = 100.0;  
const double wheel_radius = 0.1;  // R
const double wheel_distance = 0.4;  // L

// Struct to hold PWM values
struct PWMValues {
    double right_rear;
    double left_rear;
    double right_front;
    double left_front;
};

ros::Publisher right_rear_pub, right_front_pub, left_rear_pub, left_front_pub;


// Transformation to PWM
PWMValues transform_to_pwm(const geometry_msgs::Twist::ConstPtr& msg) {
    
    // Kinematics equations
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    // Calculate wheel velocities
    double v_right_rear = (2.0 / wheel_radius) * (linear_vel - 0.5 * wheel_distance * angular_vel);
    double v_left_rear = (2.0 / wheel_radius) * (linear_vel + 0.5 * wheel_distance * angular_vel);
    double v_right_front = (2.0 / wheel_radius) * (linear_vel + 0.5 * wheel_distance * angular_vel);
    double v_left_front = (2.0 / wheel_radius) * (linear_vel - 0.5 * wheel_distance * angular_vel);

    // ROS_INFO("\nWheel velocities: v_right_rear = %f, v_left_rear = %f, v_right_front = %f, v_left_front = %f",
    // v_right_rear, v_left_rear, v_right_front, v_left_front);

    // Scale velocities to the range [-1, 1]
    double v_max = std::max({std::abs(v_right_rear), std::abs(v_left_rear), std::abs(v_right_front), std::abs(v_left_front), 1.0});
    v_right_rear /= v_max;
    v_left_rear /= v_max;
    v_right_front /= v_max;
    v_left_front /= v_max;

    // Convert velocities to PWM signals (assuming PWM range is [0, max_pwm])
    PWMValues pwm_values;
    pwm_values.right_rear = v_right_rear * max_pwm;
    pwm_values.left_rear = v_left_rear * max_pwm;
    pwm_values.right_front = v_right_front * max_pwm;
    pwm_values.left_front = v_left_front * max_pwm;

    ROS_INFO("\nWheel velocities: v_right_rear = %f, v_left_rear = %f, v_right_front = %f, v_left_front = %f",
     pwm_values.right_rear, pwm_values.left_rear, pwm_values.right_front, pwm_values.left_front);

    return pwm_values;

}

void publish_signals(const PWMValues& pwm_values) {

    std_msgs::Float64 right_rear_pwm, right_front_pwm, left_rear_pwm, left_front_pwm;
    right_rear_pwm.data = pwm_values.right_rear;
    right_front_pwm.data = pwm_values.right_front;
    left_rear_pwm.data = pwm_values.left_rear;
    left_front_pwm.data = pwm_values.left_front;

    right_rear_pub.publish(right_rear_pwm);
    right_front_pub.publish(right_front_pwm);
    left_rear_pub.publish(left_rear_pwm);
    left_front_pub.publish(left_front_pwm);
}



// Callback function for cmd_vel messages
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received cmd_vel: linear.x = %f, angular.z = %f", msg->linear.x, msg->angular.z);

    // Transformation from cmd_vel to PWM values
    PWMValues pwm_values = transform_to_pwm(msg);

    publish_signals(pwm_values);

}



int main(int argc, char** argv) {
    
    ros::init(argc, argv, "cmd_vel_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 10, cmdVelCallback);

    right_rear_pub = nh.advertise<std_msgs::Float64>("/right_rear_pwm", 10);
    right_front_pub = nh.advertise<std_msgs::Float64>("/right_front_pwm", 10);
    left_rear_pub = nh.advertise<std_msgs::Float64>("/left_rear_pwm", 10);
    left_front_pub = nh.advertise<std_msgs::Float64>("/left_front_pwm", 10);

    ros::spin();

    return 0;
}