#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sound_play/sound_play.h>
#include "swipe_obstacles/closest_obstacle.h"
#include <dynamic_reconfigure/server.h>
#include <teleop_study/teleop_studyConfig.h>

#include <cmath>



class YpTeleopStudy
{
    private :
    ros::Subscriber sub_joy;
    ros::Subscriber sub_twist;
    ros::Publisher pub_yp_cmd;

    geometry_msgs::Twist in_twist;
    geometry_msgs::Twist twist;
    int triger_frag;

    float max_twist_speed;
    float accel;
    float accel_limit;
    float brake;
    float brake_limit;
    float pub_rate;
    float dash = 0;
    float base_speed = 0.5;


    ros::Timer timer;
    bool rosbag_flag ;
    bool mode;

    // bool scenario_runner;
    swipe_obstacles::closest_obstacle closest_obstacle;
    ros::Time stoped_time;
    bool closest_obstacle_is_new;

    dynamic_reconfigure::Server<teleop_study::teleop_studyConfig> server;
    dynamic_reconfigure::Server<teleop_study::teleop_studyConfig>::CallbackType server_callback;

    public :
    YpTeleopStudy();

    private :
    void joyCallback(const sensor_msgs::Joy &in_msg);
    void twistCallback(const geometry_msgs::TwistStamped &in_msg);
    void timerCallback(const ros::TimerEvent&);
    void dynamicCfgCallback(teleop_study::teleop_studyConfig &config, uint32_t level);
};


YpTeleopStudy::YpTeleopStudy(): mode(false), triger_frag(0), /*scenario_runner(false),*/closest_obstacle_is_new(false), rosbag_flag(0), accel(0.0), brake(1.0)
{
    ros::NodeHandle n;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;

    sub_joy = n.subscribe("/joy", 1, &YpTeleopStudy::joyCallback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &YpTeleopStudy::twistCallback, this);
    pub_yp_cmd = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist_cmd", 1);

    server_callback = boost::bind(&YpTeleopStudy::dynamicCfgCallback, this, _1, _2);
    server.setCallback(server_callback);

    n.getParam("autonomous_mode", mode);

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &YpTeleopStudy::timerCallback, this);
}


void YpTeleopStudy::dynamicCfgCallback(teleop_study::teleop_studyConfig &config, uint32_t level)
{
    base_speed = config.base_speed;
    dash = config.dash;
    max_twist_speed = config.max_twist_speed;
    accel_limit = config.acceleration_limit;
    brake_limit = config.deceleration_limit;
    pub_rate = config.pub_rate;
}


void YpTeleopStudy::timerCallback(const ros::TimerEvent&)
{
    float speed_change;
    float current_twist_speed, aim_twist_speed;

    // std::cout << "current_mode = " << mode << std::endl;
    // std::cout << "current_vel = " << twist.linear.x << std::endl;
    // std::cout << "current_vel = " << twist.linear.x << std::endl;
    // std::cout << "accel = " << accel << "brake = " << brake << std::endl;

    current_twist_speed = twist.linear.x;

    // if(accel != 0 && brake != 0)
    // {
    //     aim_twist_speed = in_twist.linear.x;
    //     std::cout << "aim_vel = " << aim_twist_speed << std::endl;
    //
    // }
    // else
    // {
    //     aim_twist_speed = current_twist_speed
    //     + accel * 0.25 * pub_rate * ((max_twist_speed - current_twist_speed) / max_twist_speed)
    //     - brake * 0.7 * pub_rate * (0.1 + (max_twist_speed - current_twist_speed) / max_twist_speed);
    //
    //     aim_twist_speed = (aim_twist_speed < max_twist_speed) ? aim_twist_speed : max_twist_speed;
    //     aim_twist_speed = (aim_twist_speed > 0.0) ? aim_twist_speed : 0.0;
    // }
    ROS_INFO("in_twist %f", in_twist.linear.x);


    if (in_twist.linear.x >= 0.0)
    {
        aim_twist_speed = (in_twist.linear.x + accel) * brake;
    }else
    {
        aim_twist_speed = (in_twist.linear.x - accel) * brake;
    }

    aim_twist_speed = (aim_twist_speed < max_twist_speed) ? aim_twist_speed : max_twist_speed;
    aim_twist_speed = (aim_twist_speed > -max_twist_speed) ? aim_twist_speed : -max_twist_speed;

    ROS_INFO("aim_twist1 %f", aim_twist_speed);

    // automode
    if(mode)
    {
        speed_change = aim_twist_speed - current_twist_speed;
        if(speed_change > accel_limit * pub_rate)
        {
            aim_twist_speed = current_twist_speed + accel_limit * pub_rate * ((max_twist_speed - current_twist_speed) / max_twist_speed);
        }
        // if(speed_change < -brake_limit * pub_rate)
        // {
            // aim_twist_speed = current_twist_speed - brake_limit * pub_rate * (0.1 + (max_twist_speed - current_twist_speed) / max_twist_speed);
        // }

        aim_twist_speed = (aim_twist_speed < max_twist_speed) ? aim_twist_speed : max_twist_speed;
        aim_twist_speed = (aim_twist_speed > 0.0) ? aim_twist_speed : 0.0;
    }

    // aim_twist_speed = (aim_twist_speed < max_twist_speed) ? aim_twist_speed : max_twist_speed;
    // aim_twist_speed = (aim_twist_speed > 0.0) ? aim_twist_speed : 0.0;
    ROS_INFO("aim_twist2 %f", aim_twist_speed);

    twist.linear.x = aim_twist_speed;
    twist.angular.z = in_twist.angular.z;
    // std::cout << "aim_twist=" << aim_twist_speed << std::endl;
    // std::cout << "current_twist=" << current_twist_speed << std::endl;
    // std::cout << "twist=" << twist.linear.x << std::endl;
    pub_yp_cmd.publish(twist);
}


void YpTeleopStudy::closestObstacleCallback(const swipe_obstacles::closest_obstacle &in_msg)
{
    // if closest_obstacle id is updated
    if(closest_obstacle.id != in_msg.id)
    {
        closest_obstacle_is_new = true;
        std::cout << "new obstacle comming" << std::endl;
    }
    closest_obstacle = in_msg;
}


void YpTeleopStudy::twistCallback(const geometry_msgs::TwistStamped &in_msg)
{
    if(mode)
    {
        in_twist.linear.x = in_msg.twist.linear.x;
        in_twist.angular.z = in_msg.twist.angular.z;

        // if(scenario_runner){}
        if(closest_obstacle.brief_stop && closest_obstacle.distance < closest_obstacle.stop_distance && closest_obstacle_is_new)
        {
            stoped_time = ros::Time::now();
            std::cout << "timer start: " << stoped_time << std::endl;
            closest_obstacle_is_new = false;
        }

        if(ros::Time::now() - stoped_time < ros::Duration(closest_obstacle.stop_time))
        {
            std::cout << "stopping time: " << ros::Time::now() - stoped_time << std::endl;
            in_twist.linear.x = 0.0;
        }
        else
        {
            std::cout << stoped_time << std::endl;
            std::cout << "stop_time is over time is:" << ros::Time::now() - stoped_time << std::endl;
        }
    }
}


void YpTeleopStudy::joyCallback(const sensor_msgs::Joy &in_msg)
{
    double sec_interval;

    if(in_msg.axes[5] != -0.0) triger_frag = 1;
    if(in_msg.axes[2] != -0.0) triger_frag = 2;

    if(triger_frag < 1)
    {
        accel = 0.0;
    }else
    {
        accel = (1.0 - in_msg.axes[5]) * 0.5;
    }

    if(triger_frag < 2)
    {
        brake = 0.0;
    }else
    {
        brake = (1.0 + in_msg.axes[2]) * 0.5;
    }

    ROS_INFO("accel %f", accel);
    ROS_INFO("brake %f", brake);
    // if(in_msg.axes[2] == 1.0)
    // {
    //     brake = 0.0;
    // }
    // else
    // {
    //     brake = (1.0 + in_msg.axes[2]) * 0.5;
    // }

    //A button [0]
    if (in_msg.buttons[0])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/jump.wav");
        // if(scenario_runner)
        // {
        //     sc.playWave("/usr/share/sounds/robot_sounds/mini_jump.wav");
        // }
        // else
        // {
        // sc.playWave("/usr/share/sounds/robot_sounds/jump.wav");
        // }
    }
    // B button [1]
    // if (in_msg.buttons[1])
    // {
    //     if(scenario_runner)
    //     {
    //         scenario_runner = false;
    //     }
    //     else if(!scenario_runner)
    //     {
    //         scenario_runner = true;
    //         ROS_INFO("scenario mode\n");
    //     }
    // }
    // X button [2]
    if (in_msg.buttons[2])
    {
        if(mode)
        {
            mode = false;
            sc.playWave("/usr/share/sounds/robot_sounds/pipe.wav");
            ROS_INFO("manual mode\n");
        }else if(!mode)
        {
            mode = true;
            sc.playWave("/usr/share/sounds/robot_sounds/powerup.wav");
            ROS_INFO("autonomous mode\n");
        }
    }

    // Y button [3]
    if (in_msg.buttons[3])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/coin.wav");
    }

    if(!mode)
    {
        in_twist.linear.x = base_speed * in_msg.axes[4];
        in_twist.angular.z = 0.5 * in_msg.axes[0] * in_msg.axes[4];
        // current_twist_speed = twist.linear.x;
    }

    // LB [4]
    if (in_msg.buttons[4])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
        in_twist.linear.x += (in_twist.linear.x > 0.0) ? dash : -dash;
    }
    // RB [5]
    if (in_msg.buttons[5])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
        in_twist.linear.x += (in_twist.linear.x > 0.0) ? dash : -dash;
    }

    // START [7]
    if (in_msg.buttons[7])
    {
        if(!rosbag_flag)
        {
            ROS_INFO("bag_record_on");
            sc.playWave("/usr/share/sounds/robot_sounds/new_world.wav");
            system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
            rosbag_flag = true;
        }else if(rosbag_flag)
        {
            ROS_INFO("bag_record_off");
            sc.playWave("/usr/share/sounds/robot_sounds/break_brick_block.wav");
            system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
            rosbag_flag = false;
        }
    }
    // BACK [6]
    // Logicoool [8]
    // left joy click [9]
    // right joy click [10]

    ROS_INFO("joy_in_twist %f", in_twist.linear.x);

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "yp_teleop_study0");




    YpTeleopStudy yp_teleop;


    ros::spin();
    return (0);
}
