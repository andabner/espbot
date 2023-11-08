#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>


#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_W_CAP 0x57
#define KEYCODE_A_CAP 0x41
#define KEYCODE_S_CAP 0x53
#define KEYCODE_D_CAP 0x44
#define KEYCODE_SPACE 0x20



class EspbotKeyboardTeleopNode
{
    private:
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        EspbotKeyboardTeleopNode()
        {
            pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);
            ros::NodeHandle n_private("~");
        }

        ~EspbotKeyboardTeleopNode() { }
        void keyboardLoop();

        void stopRobot()
        {
            cmdvel_.linear.x = 0.0;
            cmdvel_.angular.z = 0.0;
            pub_.publish(cmdvel_);
        }
};

EspbotKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;
float vel;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"espbot_teleope_keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    EspbotKeyboardTeleopNode tbk;
     
    cout << "Velocity: ";
    cin >> vel;

    boost::thread t = boost::thread(boost::bind(&EspbotKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void EspbotKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double maxVel = 0.0;
    double maxTurn = 0.0;
    bool dirty = false;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }

            continue;
        }

        switch(c)
        {
            case KEYCODE_W:
                maxVel = vel;//0.8;
                maxTurn = 0;
                dirty = false;
                break;
            case KEYCODE_S:
                maxVel = -vel; //0.8;
                maxTurn = 0;
                dirty = false;
                break;
            case KEYCODE_A:
                maxVel = 0;
                maxTurn = vel*15; //8.0;
                dirty = false;
                break;
            case KEYCODE_D:
                maxVel = 0;
                maxTurn = -vel*15;//8.0;
                dirty = false;
                break;

            case KEYCODE_W_CAP:
                maxVel = vel; //0.8;
                maxTurn = 0;
                dirty = false;
                break;
            case KEYCODE_S_CAP:
                maxVel = -vel; //0.8;
                maxTurn = 0;
                dirty = false;
                break;
            case KEYCODE_A_CAP:
                maxVel = 0;
                maxTurn = vel*15; //8.0;
                dirty = false;
                break;
            case KEYCODE_D_CAP:
                maxVel = 0;
                maxTurn = -vel*15;//8.0;
                dirty = false;
                break;

            case KEYCODE_SPACE:
                maxVel = 0.0;
                maxTurn = 0.0;
                dirty = false;
                break;

            default:
                maxVel = 0.0;
                maxTurn = 0.0;
                dirty = false;
                break;
        }

        cmdvel_.linear.x = maxVel;
        cmdvel_.angular.z = maxTurn;
        ROS_INFO("Send control command [ %f, %f]", cmdvel_.linear.x, cmdvel_.angular.z);
        pub_.publish(cmdvel_);
    }
}

