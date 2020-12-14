#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/TwistStamped.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <autoware_msgs/VehicleCmd.h>

#include <cmath>

#include <dynamic_reconfigure/server.h>
#include <joy_teleop/joy_teleopConfig.h>
#include <g29_force_feedback/ForceFeedback.h>

class Teleop
{
private:
    ros::Subscriber sub_joy;
    ros::Subscriber sub_fake_control;
    ros::Subscriber sub_autoware_cmd;
    ros::Subscriber sub_current_vel;
    ros::Publisher pub_vehicle_control_cmd;
    ros::Publisher pub_ff;

    float m_pub_rate;
    ros::Timer timer;

    // carla_msgs::CarlaEgoVehicleControl m_fake_control;
    carla_msgs::CarlaEgoVehicleControl m_autoware_cmd;
    // geometry_msgs::Twist joy_twist;
    geometry_msgs::Twist m_current_twist;
    carla_msgs::CarlaEgoVehicleControl m_joy_cmd;

    bool m_gear_up;
    bool m_gear_down;
    // float max_radius;
    double m_accel_Kp = 0.01;
    double m_accel_Ki = 0.01;
    double m_accel_Kd = 0.03;
    double m_brake_Kp = 0.01;
    double m_brake_Ki = 0.0;
    double m_brake_Kd = 0.01;
    double m_wheel_base = 2.7;

    // dynamic_reconfigure params
    dynamic_reconfigure::Server<joy_teleop::joy_teleopConfig> server;
    dynamic_reconfigure::Server<joy_teleop::joy_teleopConfig>::CallbackType server_callback;
    int m_device_type;
    int m_input;
    bool m_autonomous_mode;
    bool m_rosbag_flag;
    bool m_enable_ff;

public:
    Teleop();

private:
    void joyCallback(const sensor_msgs::Joy &in_joy);
    void fakeControlCb(const carla_msgs::CarlaEgoVehicleControl &in_cmd);
    void autowareCmdCb(const autoware_msgs::VehicleCmd &in_twist);
    void currentVelCb(const geometry_msgs::TwistStamped &in_twist);
    void timerCallback(const ros::TimerEvent&);
    void callbackDynamicReconfigure(joy_teleop::joy_teleopConfig &config, uint32_t lebel);
    void dynamicReconfigureUpdate();
    // float calcVelChange();
    // float calcOmega(float angular_vel);
};


Teleop::Teleop(): m_pub_rate(0.1)
{
    ros::NodeHandle n;

    server_callback = boost::bind(&Teleop::callbackDynamicReconfigure, this, _1, _2);
    server.setCallback(server_callback);

    sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
    sub_fake_control = n.subscribe("/fake_control_cmd", 1, &Teleop::fakeControlCb, this);
    sub_current_vel = n.subscribe("/current_velocity", 1, &Teleop::currentVelCb, this);
    sub_autoware_cmd = n.subscribe("/vehicle_cmd", 1, &Teleop::autowareCmdCb, this);
    pub_ff = n.advertise<g29_force_feedback::ForceFeedback>("/ff_target", 1);
    pub_vehicle_control_cmd = n.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);
    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(m_pub_rate), &Teleop::timerCallback, this);
}


void Teleop::callbackDynamicReconfigure(joy_teleop::joy_teleopConfig &config, uint32_t lebel)
{
    m_autonomous_mode = config.autonomous_mode?true:false;
    m_rosbag_flag = config.rosbag_flag?true:false;
    m_device_type = config.controller;
    m_input = config.input;
    m_enable_ff = config.enable_ff?true:false;
    m_joy_cmd.reverse = config.reverse?true:false;
}


void Teleop::dynamicReconfigureUpdate()
{
    joy_teleop::joy_teleopConfig config;
    config.autonomous_mode = m_autonomous_mode;
    config.rosbag_flag = m_rosbag_flag;
    config.controller = m_device_type;
    config.input = m_input;
    config.enable_ff = m_enable_ff;
    config.reverse = m_joy_cmd.reverse;

    server.updateConfig(config);
}


void Teleop::joyCallback(const sensor_msgs::Joy &in_joy)
{
    static int last_reverse_button = 0, last_bug_button = 0, last_autonomous_button = 0;
    m_joy_cmd.header.stamp = ros::Time::now();

    if (m_device_type == 0)
    {
        m_joy_cmd.steer = -in_joy.axes[0];
        m_joy_cmd.throttle = (1.0 - in_joy.axes[5]) * 0.5;
        m_joy_cmd.brake = (1.0 - in_joy.axes[2]) * 0.5;

        if(in_joy.buttons[1])
        {
            m_joy_cmd.reverse = !m_joy_cmd.reverse;
            m_autonomous_mode = false;
            dynamicReconfigureUpdate();
        }

        if(in_joy.buttons[2])
        {
            m_autonomous_mode = !m_autonomous_mode;
            m_joy_cmd.reverse = false;
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
        m_joy_cmd.steer = -in_joy.axes[0];
        m_joy_cmd.throttle = (1.0 + in_joy.axes[2]) * 0.5;
        m_joy_cmd.brake = (1.0 + in_joy.axes[3]) * 0.5;

        if(in_joy.buttons[4] && !m_gear_up)
        {
            m_joy_cmd.gear = std::min(m_joy_cmd.gear + 1, 4);
            m_gear_up = in_joy.buttons[4];
            std::cout << m_joy_cmd.gear << std::endl;
        }

        if(in_joy.buttons[5] && !m_gear_down)
        {
            m_joy_cmd.gear = std::max(m_joy_cmd.gear - 1, 0);
            m_gear_down = in_joy.buttons[5];
            std::cout << m_joy_cmd.gear << std::endl;
        }

        if(in_joy.buttons[2] && !last_autonomous_button)
        {
            m_joy_cmd.reverse = !m_joy_cmd.reverse;
            m_autonomous_mode = false;
            dynamicReconfigureUpdate();
        }
        last_reverse_button == in_joy.buttons[2];

        if(in_joy.buttons[3] && !last_reverse_button)
        {
            m_autonomous_mode = !m_autonomous_mode;
            m_joy_cmd.reverse = false;
            dynamicReconfigureUpdate();
        }
        last_autonomous_button == in_joy.buttons[3];

        // START [7]
        if (in_joy.buttons[24] && !last_bug_button)
        {
            m_rosbag_flag = !m_rosbag_flag;
            dynamicReconfigureUpdate();
            if(!m_rosbag_flag)
            {
                ROS_INFO("bag_record_on");
                system("bash /home/kuriatsu/Source/catkin_ws/src/joy-teleop/src/bag_recorder.sh &");
            }else
            {
                ROS_INFO("bag_record_off");
                system("bash /home/kuriatsu/Source/catkin_ws/src/joy-teleop/src/bag_stopper.sh &");
            }
        }
        last_bug_button = in_joy.buttons[24];
    }
}


void Teleop::currentVelCb(const geometry_msgs::TwistStamped &in_twist)
{
    if (m_input == 0)
    {
        m_current_twist = in_twist.twist;
    }
}


void Teleop::fakeControlCb(const carla_msgs::CarlaEgoVehicleControl &in_cmd)
{
    m_autoware_cmd = in_cmd;
}


void Teleop::autowareCmdCb(const autoware_msgs::VehicleCmd &in_cmd)
{
    if (m_input == 1) return;

    static double integral=0.0, proportional = 0.0, differential = 0.0;
    double buf_proportional, buf_differential, result, radius;
    // carla_msgs::CarlaEgoVehicleControl calcurated_cmd;

    buf_proportional = proportional;
    buf_differential = differential;
    proportional = in_cmd.twist_cmd.twist.linear.x - m_current_twist.linear.x;
    differential =  proportional - buf_proportional;
    std::cout <<m_current_twist.linear.x << ", " << proportional << ", " << differential << ", " << integral << std::endl;
    if((buf_proportional > 0.0 && proportional < 0.0) || (buf_proportional < 0.0 && proportional >0.0))
    {
        integral = 0.0;
    }
    else
    {
        integral += proportional;
    }

    if (proportional > 0.0)
    {
        m_autoware_cmd.throttle = fabs(m_accel_Kp * proportional + m_accel_Ki * integral + m_accel_Kd * differential);
        m_autoware_cmd.brake = 0.0;
    }
    else
    {
        m_autoware_cmd.brake = fabs(m_brake_Kp * proportional + m_brake_Ki * integral + m_brake_Kd * differential);
        m_autoware_cmd.throttle = 0.0;
    }

    radius = in_cmd.twist_cmd.twist.linear.x / in_cmd.twist_cmd.twist.angular.z;
    // radius = m_current_twist.linear.x / in_cmd.twist_cmd.twist.angular.z;
    m_autoware_cmd.steer = -std::atan(m_wheel_base / radius);
    // if (in_cmd.twist_cmd.twist.angular.z == 0.0 || m_current_twist.linear.x == 0.0)
    // {
    //     m_autoware_cmd.steer = 0.0;
    //     std::cout << "twist or angular is 0" << std::endl;
    // }
    // else
    // {
    //     radius = m_current_twist.linear.x / in_cmd.twist_cmd.twist.angular.z;
    //     m_autoware_cmd.steer = -std::atan(m_wheel_base / radius);
    // }
}


void Teleop::timerCallback(const ros::TimerEvent&)
{
    carla_msgs::CarlaEgoVehicleControl final_cmd;

    if (m_enable_ff)
    {
        g29_force_feedback::ForceFeedback ff;

        if (m_autonomous_mode)
        {
            ff.angle = m_autoware_cmd.steer;
            ff.force = 0.0;
            ff.pid_mode = true;
        }
        else
        {
            ff.angle = 0.0;
            ff.force = 0.2;
            ff.pid_mode = false;
        }

        pub_ff.publish(ff);
    }
    else
    {
        g29_force_feedback::ForceFeedback ff;
        ff.force = 0.0;
        pub_ff.publish(ff);
    }

    if (m_autonomous_mode)
    {
        final_cmd = m_autoware_cmd;

        if (m_enable_ff)
        {
            final_cmd.steer = m_joy_cmd.steer;
        }

        final_cmd.throttle = (m_joy_cmd.throttle != 0.0) ? m_joy_cmd.throttle : m_autoware_cmd.throttle;
        // final_cmd.throttle = std::max(m_joy_cmd.throttle, m_autoware_cmd.throttle);
        final_cmd.brake = std::max(m_joy_cmd.brake, m_autoware_cmd.brake);

    }
    else
    {
        final_cmd = m_joy_cmd;
    }

    pub_vehicle_control_cmd.publish(final_cmd);
}

//
// float Teleop::calcVelChange()
// {
//     float vel_change;
//
//     // when no pedal control
//     if (m_joy_cmd.brake == 0.0 && m_joy_cmd.throttle == 0.0 && !m_autonomous_mode)
//     {
//         return -m_natural_decel;
//     }
//     else if (m_joy_cmd.brake == 0.0 && m_joy_cmd.throttle == 0.0 && m_autonomous_mode)
//     {
//         vel_change = m_autoware_cmd.linear.x - m_current_twist.linear.x;
//     }
//     // when manual override
//     else
//     {
//         vel_change = m_joy_cmd.throttle * m_max_accel - m_joy_cmd.brake * m_max_decel;
//     }
//     // cut velocity change with threshold
//     if (vel_change < -m_max_decel || m_max_accel < vel_change)
//     {
//         std::cout << "velchange over max" << std::endl;
//         return (vel_change > 0.0) ? m_max_accel : -m_max_decel;
//     }
//     if (fabs(m_current_twist.linear.x) > m_max_vel)
//     {
//         std::cout << "vel is over max" << std::endl;
//         return 0.0;
//     }
//     return vel_change;
// }

//
// float Teleop::calcOmega(const float angular_vel)
// {
//     float max_angular = fabs(m_current_twist.linear.x) * tan(m_max_wheel_angle * M_PI / 180) / 3.0;
//     // std::cout <<  m_current_twist.linear.x << " : " << angular_vel << " : " << max_angular << std::endl;
//     return (fabs(angular_vel) < max_angular) ?  angular_vel : ( angular_vel < 0.0) ? -max_angular : max_angular;
//     // return autonomous_omega;
//     // if (m_autonomous_mode)
//     //     return (fabs(autonomous_omega) < max_angular) ? autonomous_omega : (autonomous_omega < 0.0) ? max_angular : -max_angular;
//     // else
//     //     return (fabs(manual_omega) < max_angular) ? manual_omega : (manual_omega < 0.0) ? -max_angular : max_angular;
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_teleop_node");
    Teleop teleop;
    ros::spin();
    return(0);
}
