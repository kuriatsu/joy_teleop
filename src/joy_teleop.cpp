#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

#include <cmath>

#include <dynamic_reconfigure/server.h>
#include <joy_teleop/joy_teleopConfig.h>
#include <g29_force_feedback/ForceFeedback.h>

class Teleop
{
private:
    ros::Subscriber sub_joy;
    ros::Subscriber sub_twist;
    ros::Subscriber sub_odom;
    ros::Publisher pub_twist;
    ros::Publisher pub_vehicle_control;
    ros::Publisher pub_ff;

    float pub_rate;


    geometry_msgs::Twist m_autoware_twist;
    // geometry_msgs::Twist joy_twist;
    geometry_msgs::Twist m_current_twist;
    carla_msgs::CarlaEgoVehicleControl m_vehicle_cmd;
    float m_throttle;
    float m_brake;
    float m_manual_omega;
    // float max_radius;
    ros::Timer timer;

    // dynamic_reconfigure params
    float m_max_vel;
    float m_max_wheel_angle;
    float m_max_accel;
    float m_max_decel;
    float m_natural_decel;
    int m_device_type;
    int m_control_type;
    bool m_autonomous_mode;
    bool m_rosbag_flag;
    bool m_back;
    bool m_enable_ff;

    dynamic_reconfigure::Server<joy_teleop::joy_teleopConfig> server;
    dynamic_reconfigure::Server<joy_teleop::joy_teleopConfig>::CallbackType server_callback;
public:
    Teleop();

private:
    void joyCallback(const sensor_msgs::Joy &in_joy);
    void twistCallback(const geometry_msgs::TwistStamped &in_twist);
    void odomCallback(const nav_msgs::Odometry &in_odom);
    void timerCallback(const ros::TimerEvent&);
    void callbackDynamicReconfigure(joy_teleop::joy_teleopConfig &config, uint32_t lebel);
    void dynamicReconfigureUpdate();
    float calcVelChange();
    float calcOmega(float angular_vel);
};


Teleop::Teleop(): m_throttle(0.0), m_brake(0.0), m_back(false), pub_rate(0.1)
{
    ros::NodeHandle n;

    server_callback = boost::bind(&Teleop::callbackDynamicReconfigure, this, _1, _2);
    server.setCallback(server_callback);

    sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &Teleop::twistCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 1, &Teleop::odomCallback, this);
    pub_ff = n.advertise<g29_force_feedback::ForceFeedback>("/ff_target", 1);
    pub_twist = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist_cmd", 1);
    pub_vehicle_control = n.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);
    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &Teleop::timerCallback, this);
}


void Teleop::callbackDynamicReconfigure(joy_teleop::joy_teleopConfig &config, uint32_t lebel)
{
    m_max_vel = config.max_speed / 3.6;
    m_max_wheel_angle = config.max_steer;
    m_max_accel = config.acceleration_coefficient * 9.8 * pub_rate * 3.6;
    m_max_decel = config.deceleration_coefficient * 9.8 * pub_rate  * 3.6;
    m_natural_decel = config.natural_deceleration_coefficient * 9.8 * pub_rate  * 3.6;
    m_autonomous_mode = config.autonomous_mode?true:false;
    m_rosbag_flag = config.rosbag_flag?true:false;
    m_device_type = config.controller;
    m_control_type = config.control_type;
    m_enable_ff = config.enable_ff?true:false;

    if (m_enable_ff && m_autonomous_mode)
    {
        ros::param::set("/mode", 0);
    }
    else if (m_enable_ff && !m_autonomous_mode)
    {
        ros::param::set("/mode", 1);
    }
}


void Teleop::dynamicReconfigureUpdate()
{
    joy_teleop::joy_teleopConfig config;
    config.max_speed = m_max_vel * 3.6;
    config.max_steer = m_max_wheel_angle;
    config.acceleration_coefficient = m_max_accel / (9.8 * pub_rate * 3.6);
    config.deceleration_coefficient = m_max_decel / (9.8 * pub_rate  * 3.6);
    config.natural_deceleration_coefficient = m_natural_decel / (9.8 * pub_rate  * 3.6);
    config.autonomous_mode = m_autonomous_mode;
    config.rosbag_flag = m_rosbag_flag;
    config.controller = m_device_type;
    config.control_type = m_control_type;
    config.enable_ff = m_enable_ff;

    server.updateConfig(config);
}


void Teleop::joyCallback(const sensor_msgs::Joy &in_joy)
{
    if (m_device_type == 0)
    {
        m_throttle = (1.0 - in_joy.axes[5]) * 0.5;
        m_brake = (1.0 - in_joy.axes[2]) * 0.5;
        m_manual_omega = -m_current_twist.linear.x * tan(m_max_wheel_angle * in_joy.axes[0] * M_PI / 180) / 3.0;
        // m_manual_omega = -m_current_twist.linear.x * tan((90.0 - m_max_wheel_angle) * in_joy.axes[0] * M_PI / 180) / 3.0;

        if(in_joy.buttons[1] && fabs(m_current_twist.linear.x) < 2.0)
        {
            m_back = !m_back;
            m_autonomous_mode = false;
        }

        if(in_joy.buttons[2])
        {
            m_autonomous_mode = !m_autonomous_mode;
            dynamicReconfigureUpdate();
        }

        // START [7]
        if (in_joy.buttons[7])
        {
            m_rosbag_flag = !m_rosbag_flag;
            dynamicReconfigureUpdate();
            if(!m_rosbag_flag)
            {
                ROS_INFO("bag_record_on");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
            }else
            {
                ROS_INFO("bag_record_off");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
            }
        }
    }
    else if(m_device_type == 1)
    {
        m_throttle = (1.0 + in_joy.axes[2]) * 0.5;
        m_brake = (1.0 + in_joy.axes[3]) * 0.5;
        m_manual_omega = -m_current_twist.linear.x * tan(m_max_wheel_angle * in_joy.axes[0] * M_PI / 180) / 3.0;
        // m_manual_omega = -m_current_twist.linear.x * tan((90.0 - m_max_wheel_angle) * in_joy.axes[0] * M_PI / 180) / 3.0;

        if(in_joy.buttons[3])
        {
            m_autonomous_mode = !m_autonomous_mode;
            dynamicReconfigureUpdate();
        }

        if(in_joy.buttons[2] && !m_autonomous_mode)
        {
            m_back = !m_back;
            m_autonomous_mode = false;
        }
        // START [7]
        if (in_joy.buttons[1])
        {
            m_rosbag_flag = !m_rosbag_flag;
            dynamicReconfigureUpdate();
            if(!m_rosbag_flag)
            {
                ROS_INFO("bag_record_on");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
            }else
            {
                ROS_INFO("bag_record_off");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
            }
        }
    }
    m_vehicle_cmd.header.stamp = ros::Time::now();
    m_vehicle_cmd.throttle = m_throttle;
    m_vehicle_cmd.steer = -in_joy.axes[0];
    m_vehicle_cmd.brake = m_brake;
    m_vehicle_cmd.reverse = m_back;
    // m_vehicle_cmd.gear = 4;
    // m_vehicle_cmd.gear = int(m_vehicle_cmd.manual_gear_shift + in_joy.buttons[4] - in_joy.buttons[5]) % 4 + 1;
}


void Teleop::twistCallback(const geometry_msgs::TwistStamped &in_twist)
{
    if (m_autonomous_mode)
    {
        m_autoware_twist = in_twist.twist;
        m_autoware_twist.angular.z = -in_twist.twist.angular.z;
    }
}


void Teleop::odomCallback(const nav_msgs::Odometry &in_odom)
{
    m_current_twist = in_odom.twist.twist;
}


void Teleop::timerCallback(const ros::TimerEvent&)
{
    geometry_msgs::Twist out_twist;

    if (m_autonomous_mode || m_control_type == 0)
    {
        float vel_change = calcVelChange();

        if (m_back)
            vel_change *= -1;

        out_twist.linear.x = m_current_twist.linear.x + vel_change;

        if ((m_back && out_twist.linear.x > 0.0) || (!m_back && out_twist.linear.x < 0.0))
        {
            std::cout << "sign changed unexpectedly" << m_back << m_control_type << out_twist.linear.x << std::endl;
            out_twist.linear.x = 0.0;
        }

        if ((m_enable_ff && m_autonomous_mode) || !m_autonomous_mode)
        {
            out_twist.angular.z = calcOmega(m_manual_omega);
        }
        else
        {
            out_twist.angular.z = calcOmega(m_autoware_twist.angular.z);
        }

        pub_twist.publish(out_twist);
        // m_autoware_twist.linear.x = 0.0;
    }
    else
    {
        pub_vehicle_control.publish(m_vehicle_cmd);
        // pub_twist.publish(m_autoware_twist);
    }

    if (m_enable_ff)
    {
        g29_force_feedback::ForceFeedback ff;

        if (m_autonomous_mode)
        {
            ff.angle = -out_twist.angular.z;
            ff.force = 0.0;
            ff.pid_mode = true;
        }
        else
        {
            ff.angle = 0.0;
            ff.force = 0.25;
            ff.pid_mode = false;
        }

        pub_ff.publish(ff);
    }
    else
    {
        g29_force_feedback::ForceFeedback ff;
        // ff.angle = -out_twist.angular.z;
        ff.force = 0.0;
        pub_ff.publish(ff);
    }
}


float Teleop::calcVelChange()
{
    float vel_change;

    // when no pedal control
    if (m_brake == 0.0 && m_throttle == 0.0 && !m_autonomous_mode)
    {
        return -m_natural_decel;
    }
    else if (m_brake == 0.0 && m_throttle == 0.0 && m_autonomous_mode)
    {
        vel_change = m_autoware_twist.linear.x - m_current_twist.linear.x;
    }
    // when manual override
    else
    {
        vel_change = m_throttle * m_max_accel - m_brake * m_max_decel;
    }
    // cut velocity change with threshold
    if (vel_change < -m_max_decel || m_max_accel < vel_change)
    {
        std::cout << "velchange over max" << std::endl;
        return (vel_change > 0.0) ? m_max_accel : -m_max_decel;
    }
    if (fabs(m_current_twist.linear.x) > m_max_vel)
    {
        std::cout << "vel is over max" << std::endl;
        return 0.0;
    }
    return vel_change;
}


float Teleop::calcOmega(const float angular_vel)
{
    float max_angular = fabs(m_current_twist.linear.x) * tan(m_max_wheel_angle * M_PI / 180) / 3.0;
    std::cout <<  m_current_twist.linear.x << " : " << angular_vel << " : " << max_angular << std::endl;
    return (fabs(angular_vel) < max_angular) ?  angular_vel : ( angular_vel < 0.0) ? -max_angular : max_angular;
    // return autonomous_omega;
    // if (m_autonomous_mode)
    //     return (fabs(autonomous_omega) < max_angular) ? autonomous_omega : (autonomous_omega < 0.0) ? max_angular : -max_angular;
    // else
    //     return (fabs(manual_omega) < max_angular) ? manual_omega : (manual_omega < 0.0) ? -max_angular : max_angular;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_teleop_node");
    Teleop teleop;
    ros::spin();
    return(0);
}
