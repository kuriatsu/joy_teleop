#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

#include <cmath>

#include <dynamic_reconfigure/server.h>
#include <joy_teleop/joy_teleopConfig.h>
#include <g29_force_feedback/ForceFeedback.h>

class Teleop
{
private:
    ros::Subscriber sub_joy;
    ros::Subscriber sub_vehicle_control_cmd;
    // ros::Subscriber sub_odom;
    ros::Publisher pub_vehicle_control_cmd;
    ros::Publisher pub_ff;

    float pub_rate;


    carla_msgs::CarlaEgoVehicleControl m_autoware_cmd;
    // geometry_msgs::Twist joy_twist;
    // geometry_msgs::Twist m_current_twist;
    carla_msgs::CarlaEgoVehicleControl m_joy_cmd;

    bool m_gear_up;
    bool m_gear_down;
    // float max_radius;
    ros::Timer timer;

    // dynamic_reconfigure params
    int m_device_type;
    bool m_autonomous_mode;
    bool m_rosbag_flag;
    // bool m_joy_cmd.reverse;
    bool m_enable_ff;

    dynamic_reconfigure::Server<joy_teleop::joy_teleopConfig> server;
    dynamic_reconfigure::Server<joy_teleop::joy_teleopConfig>::CallbackType server_callback;
public:
    Teleop();

private:
    void joyCallback(const sensor_msgs::Joy &in_joy);
    void vehicleControlCmdCb(const carla_msgs::CarlaEgoVehicleControl &in_cmd);
    // void odomCallback(const nav_msgs::Odometry &in_odom);
    void timerCallback(const ros::TimerEvent&);
    void callbackDynamicReconfigure(joy_teleop::joy_teleopConfig &config, uint32_t lebel);
    void dynamicReconfigureUpdate();
    // float calcVelChange();
    // float calcOmega(float angular_vel);
};


Teleop::Teleop(): pub_rate(0.1)
{
    ros::NodeHandle n;

    server_callback = boost::bind(&Teleop::callbackDynamicReconfigure, this, _1, _2);
    server.setCallback(server_callback);

    sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
    sub_vehicle_control_cmd = n.subscribe("/fake_control_cmd", 1, &Teleop::vehicleControlCmdCb, this);
    // sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 1, &Teleop::odomCallback, this);
    pub_ff = n.advertise<g29_force_feedback::ForceFeedback>("/ff_target", 1);
    pub_vehicle_control_cmd = n.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);
    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &Teleop::timerCallback, this);
}


void Teleop::callbackDynamicReconfigure(joy_teleop::joy_teleopConfig &config, uint32_t lebel)
{
    m_autonomous_mode = config.autonomous_mode?true:false;
    m_rosbag_flag = config.rosbag_flag?true:false;
    m_device_type = config.controller;
    m_enable_ff = config.enable_ff?true:false;
    m_joy_cmd.reverse = config.reverse?true:false;
}


void Teleop::dynamicReconfigureUpdate()
{
    joy_teleop::joy_teleopConfig config;
    config.autonomous_mode = m_autonomous_mode;
    config.rosbag_flag = m_rosbag_flag;
    config.controller = m_device_type;
    config.enable_ff = m_enable_ff;
    config.reverse = m_joy_cmd.reverse;

    server.updateConfig(config);
}


void Teleop::joyCallback(const sensor_msgs::Joy &in_joy)
{
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

        if(in_joy.buttons[2])
        {
            m_joy_cmd.reverse = !m_joy_cmd.reverse;
            m_autonomous_mode = false;
            dynamicReconfigureUpdate();
        }

        if(in_joy.buttons[3])
        {
            m_autonomous_mode = !m_autonomous_mode;
            m_joy_cmd.reverse = false;
            dynamicReconfigureUpdate();
        }

        // START [7]
        if (in_joy.buttons[24])
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
    }
}


void Teleop::vehicleControlCmdCb(const carla_msgs::CarlaEgoVehicleControl &in_cmd)
{
    m_autoware_cmd = in_cmd;
}


// void Teleop::odomCallback(const nav_msgs::Odometry &in_odom)
// {
//     m_current_twist = in_odom.twist.twist;
// }


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

        final_cmd.throttle = std::max(m_joy_cmd.throttle, m_autoware_cmd.throttle);
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
