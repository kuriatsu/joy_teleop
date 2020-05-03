#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

class Teleop
{
private:
    // ros::Subscriber sub_twist;
    ros::Subscriber sub_joy;
    // ros::Publisher pub_twist;

    ros::Timer timer;
    int device_handle;
    int axis_code = ABS_X;
    int axis_min;
    int axis_max;
    struct ff_effect effect;
    std::string device_name;
    float steer;

public:
    Teleop();

private:
    // void twistCallback(const sensor_msgs::TwistStamped &in_twist);
    void joyCallback(const sensor_msgs::Joy &in_joy);
    void timerCallback(const ros::TimerEvent&);
    int testBit(int bit, unsigned char *array);
    void initFfDevice();
    void updateFfDevice();
};

Teleop::Teleop() : device_name("/dev/input/event19")
{
    ros::NodeHandle n;

    sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
    n.getParam("device_name", device_name);

    // sub_twist
    initFfDevice();

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(0.05), &Teleop::timerCallback, this);
}
//
void Teleop::timerCallback(const ros::TimerEvent&)
{
    updateFfDevice();
}


void Teleop::updateFfDevice()
{
    struct input_event event;
    double force;
    // std::cout << fabs(steer) << std::endl;
    if (fabs(steer) < 0.0001) force = 0.0;
    else force = (steer > 0.0) ? 0.8 : -0.8;
    effect.u.constant.level = (short)(force * 32767.0);
    effect.direction = 0xC000;
    effect.u.constant.envelope.attack_level = (short)(force * 32767.0);
    effect.u.constant.envelope.fade_level = (short)(force * 32767.0);

    if (ioctl(device_handle, EVIOCSFF, &effect) < 0)
    {
        std::cout << "failed to upload effect" << std::endl;
    }

}


void Teleop::joyCallback(const sensor_msgs::Joy &in_joy)
{
    steer = in_joy.axes[0];
}


void Teleop::initFfDevice()
{
    // setup device
    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];

    struct input_event event;
    struct input_absinfo abs_info;

    device_handle = open(device_name.c_str(), O_RDWR|O_NONBLOCK);
    if (device_handle < 0)
    {
        std::cout << "ERROR: cannot open device : "<< device_name << std::endl;
        exit(1);
    }else{std::cout << "device opened" << std::endl;}

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0)
    {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        exit(1);
    }

    // get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0)
    {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
        exit(1);
    }

    // get axis value range
    if (ioctl(device_handle, EVIOCGABS(axis_code), &abs_info) < 0)
    {
        std::cout << "ERROR: cannot get axis range" << std::endl;
        exit(1);
    }
    axis_max = abs_info.maximum;
    axis_min = abs_info.minimum;
    if (axis_min >= axis_max)
    {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        exit(1);
    }

    // check force feedback is supported?
    if(!testBit(FF_CONSTANT, ff_bits))
    {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        exit(1);
    }else{std::cout << "force feedback supported" << std::endl;}

    // auto centering off
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;
    if (write(device_handle, &event, sizeof(event)) != sizeof(event))
    {
        std::cout << "failed to disable auto centering" << std::endl;
        exit(1);
    }

    // initialize constant foce effect
    memset(&effect, 0, sizeof(effect));
    effect.type = FF_CONSTANT;
    effect.id = -1;
    effect.trigger.button = 0;
    effect.trigger.interval = 0;
    effect.replay.length = 0xffff;
    effect.replay.delay = 0;
    effect.u.constant.level = 0;
    effect.direction = 0xC000;
    effect.u.constant.envelope.attack_length = 0;
    effect.u.constant.envelope.attack_level = 0;
    effect.u.constant.envelope.fade_length = 0;
    effect.u.constant.envelope.fade_level = 0;

    if (ioctl(device_handle, EVIOCSFF, &effect) < 0)
    {
        std::cout << "failed to upload effect" << std::endl;
        exit(1);
    }

    // start effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = effect.id;
    event.value = 1;
    if (write(device_handle, &event, sizeof(event)) != sizeof(event))
    {
        std::cout << "failed to start event" << std::endl;
        exit(1);
    }
}

int Teleop::testBit(int bit, unsigned char *array)
{
    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_carla_ff_node");
    Teleop teleop;
    ros::spin();
    return(0);
}
