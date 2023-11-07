#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
extern "C" {
#include "user_interface.h"
}

// Init constants and global variables

#define DEBUG 0
#define TRIGGER D8  // ultrasonic trigger pin
#define ECHO    D0  // ultrasonic echo pin  (!!Note: use 5v Vcc on ultrasonic board & a 2k,1k divider for ECHO GPIO protection)

// Differential drive working variables and constants

double lpwm=0, rpwm=0, lv=0, rv=0, lvt=0, rvt=0;                       // motor pwm output and calculated velocities (lv as tics/sec, lvt as tic/(time interval))
int lmc=0, rmc=0, lmc0=0, rmc0=0, ldir=1, rdir=1, llevel=1, rlevel=1;  // l=left, r=right motors encoder params
                  
double WheelSeparation= 0.135;                              // wheel separation in meters
double whedia= 0.07;                               // wheel diameter in meters
int CPR = 20;                                      // Encoder Count per Revolutions (can be duplicated changing RAISING interrupt by CHANGE)

// Timing using timer for PID and time interval between encoders ticks

int rcurrenttime, rlasttime, lcurrenttime, llasttime;
os_timer_t myTimer;

// WiFi configuration. Replace '***' with your data

const char* ssid = "***";
const char* password = "***";
IPAddress server(192,168,1,***);      // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port


// Functions definitions //

void setupWiFi() {                    // connect to ROS server as as a client
  if(DEBUG){
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  if(DEBUG){
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

static inline int8_t sgn(int val) {   // Better sgn function definition
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

void motion(double lpwm, double rpwm) {  // move motor at pwm power and change directions flags only when motor cross stop
  if(abs(lIn)<1){
    if(lOut>=0){
      ldir=1; 
      llevel=HIGH; 
    } else {
      ldir=-1;
      llevel=LOW; 
    }
  }
  if(abs(rIn)<1){
    if(rOut>=0){
      rdir=1; 
      rlevel=HIGH; 
    } else {
      rdir=-1;
      rlevel=LOW; 
    }
  }
  analogWrite(D1, abs(lpwm));
  analogWrite(D2, abs(rpwm));
  digitalWrite(D3, llevel);
  digitalWrite(D4, rlevel);
}
void lencode() {          // GPIO ISR Interrupt service routines for encoder changes
  lcurrenttime = millis();
  lvt = ldir*1000./(lcurrenttime - llasttime);
  llasttime = lcurrenttime;
  lmc=lmc+ldir;
}
void rencode(){ 
  rcurrenttime = millis();
  rvt = rdir*1000./(rcurrenttime - rlasttime);
  rlasttime = rcurrenttime;
  rmc=rmc+rdir;
}

void tic(void *pArg) {    // timerCallback, repeat every "period"
  lv=(lmc-lmc0);     
  lmc0=lmc;
  rv=(rmc-rmc0); 
  rmc0=rmc;
  lIn = lv*kt;
  rIn = rv*kt;
  if(abs(lv)>=1) lIn=abs(lvt)*sgn(lv); // use timing to calculate velocity only if ticks are greater than one
  if(abs(rv)>=1) rIn=abs(rvt)*sgn(rv); // to avoid problems with sign and inertia for a simple encoder (no quadrature)
  lPID.Compute();
  rPID.Compute();
  motion(lOut,rOut); 
}

void cmd_velCallback( const geometry_msgs::Twist& CVel){
  //geometry_msgs::Twist twist = twist_msg;   
    double vel_x = CVel.linear.x;
    double vel_th = CVel.angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;

    // turning
    if(vel_x == 0){  
        right_vel = vel_th * WheelSeparation / 2.0;
        left_vel = (-1) * right_vel;
    }
    // forward / backward
    else if(vel_th == 0){ 
        left_vel = right_vel = vel_x;
    }
    // moving doing arcs
    else{ 
        left_vel = vel_x - vel_th * WheelSeparation / 2.0;
        right_vel = vel_x + vel_th * WheelSeparation / 2.0;
    }
    //write new command speeds to global vars 
    lSet = left_vel;
    rSet = right_vel;
}

int srange(){                // calculate distance from ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIGGER, LOW);  
  delayMicroseconds(2);   
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10); 
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1;
  sstep();
  return (int) distance;
}

// ROS nodes //
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;    // transformation frame for base 
geometry_msgs::TransformStamped t2;   // transformation frame for ultrasonic to base
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom;              // Odometry message
geometry_msgs::Twist odom_msg;        // 

//tf::TransformBroadcaster odom_broadcaster;
sensor_msgs::Range range_msg;         // Ultrasonic Range message

// ROS topics object definitions PUBLISHERS
std_msgs::String str_msg;
std_msgs::Int16 int_msg;

ros::Publisher pub_range("/car/ultrasound", &range_msg);
ros::Publisher odom_pub("/car/odom", &odom); 
ros::Publisher Pub ("/car/ard_odom", &odom_msg);

// ROS SUBSCRIBERS
ros::Subscriber<geometry_msgs::Twist> Sub("/car/cmd_vel", &cmd_velCallback );

// ros variables
double x = 1.0;
double y = 0.0;
double th = 0;
char base_link[] = "/base_link";
char odomid[] = "/odom";
char ultrafrid[] = "/ultrasound";

void setup() {
  if(DEBUG) Serial.begin(115200);
  setupWiFi();
  delay(2000);
// Ros objects constructors   
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(pub_range);
  nh.advertise(odom_pub);
  nh.subscribe(Sub);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  ultrafrid;   // ultrasound frame id
  range_msg.field_of_view = 0.1;
  range_msg.min_range = 0.0;
  range_msg.max_range = 20;
  
// configure GPIO's
  pinMode(D0, OUTPUT); // Ultrasonic Trigger
  pinMode(D1, OUTPUT); // 1,2EN aka D1 pwm left
  pinMode(D2, OUTPUT); // 3,4EN aka D2 pwm right
  pinMode(D3, OUTPUT); // 1A,2A aka D3
  pinMode(D4, OUTPUT); // 3A,4A aka D4
  pinMode(D5, INPUT); //  Left encoder
  pinMode(D6, INPUT); //  Right encoder

  pinMode(ECHO, INPUT); //  Ultrasonic Echo. D0 with 1k,2k voltage divisor
  pinMode(TRIGGER, OUTPUT); // Ultrasonic Trigger. D8 . Power Ultrasonic board with 5v.
// configure interrupts to their ISR's    
  attachInterrupt(D5, lencode, RISING); // Setup Interrupt 
  attachInterrupt(D6, rencode, RISING); // Setup Interrupt 
  sei();                                // Enable interrupts  
// configure timer
  os_timer_setfn(&myTimer, tic, NULL); 
  os_timer_arm(&myTimer, period, true);   // timer in ms
// Configure and start PID's
  lPID.SetSampleTime(period);
  rPID.SetSampleTime(period); 
  lPID.SetOutputLimits(-1023, 1023);  
  rPID.SetOutputLimits(-1023, 1023);  
  lPID.SetMode(AUTOMATIC);
  rPID.SetMode(AUTOMATIC);
}

// odometry configuration
ros::Time current_time = nh.now();
ros::Time last_time = current_time;
double DistancePerCount = (TWO_PI * 0.035) / 20;   // 2*PI*R/CPR  WHEEL ENCODER 20 CPR
double lengthBetweenTwoWheels = 0.13;
int last_lmc = lmc;
int last_rmc = rmc;
int current_lmc = lmc;
int current_rmc = rmc;

void loop() {
  if (nh.connected()) {
    current_time = nh.now();
    current_lmc = lmc;
    current_rmc = rmc;
    double dt = current_time.toSec() - last_time.toSec();  
    double ld = (current_lmc-last_lmc)*DistancePerCount;   // left wheel linear distance
    double rd = (current_rmc-last_rmc)*DistancePerCount;  // right wheel linear distance
    double vlm = ld / dt;                                // left wheel linear velocity
    double vrm = rd / dt;                               // right wheel linear velocity
    double vx = (ld + rd) / 2.;                        // base center forward velocity 
    double vy = 0;
    double th = (current_rmc - current_lmc) / 45. * PI;
    double vth = th / dt; 
    last_lmc = current_lmc;
    last_rmc = current_rmc;
    last_time = current_time;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    x += delta_x;
    y += delta_y;
    t.header.frame_id = odomid;
    t.child_frame_id = base_link;
    t.transform.translation.x = x; 
    t.transform.translation.y = y; 
    t.transform.rotation = tf::createQuaternionFromYaw(th);
    t.header.stamp = current_time;
    broadcaster.sendTransform(t);
    range_msg.range = srange()/100.;
    range_msg.header.stamp = current_time;
    pub_range.publish(&range_msg);
    t2.header.frame_id = base_link;
    t2.child_frame_id = ultrafrid;
    t2.transform.translation.x = 0.05; 
    t2.transform.translation.y = 0.0; 
    t2.transform.translation.z = 0.1;
    t2.transform.rotation = tf::createQuaternionFromYaw(radian(sa));
    t2.header.stamp = current_time;
    broadcaster.sendTransform(t2);
    // odometry
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
    odom.header.stamp = current_time;
    odom.header.frame_id = odomid;

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(&odom);

  } else {
    if(DEBUG) Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop aprox. every  
  delay(200);  // milliseconds
}
