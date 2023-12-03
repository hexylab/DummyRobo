#define USE_PCA9685_SERVO_EXPANDER
#include <M5Stack.h>
#include <Arduino.h>
#include <ServoEasing.hpp>
#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Variables                                                                      *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//-------- Variables for servo data -------------
float last_servoAngle_q1 = 90;
float last_servoAngle_q2 = 90;
float last_servoAngle_q3 = 90;
float last_servoAngle_EE = 90;
float servoAngle_q1 = 90;
float servoAngle_q2 = 90;
float servoAngle_q3 = 90;
float servoAngle_EE = 90;
int servoTime_q1 = 1000;
int servoTime_q2 = 1000;
int servoTime_q3 = 1000;
int servoTime_EE = 1000;
int check = 0;

//-------- Variables for WiFi connection -------------
const char ssid[] = "Coke-24g";
const char pass[] = "gatyapin";
WiFiClient client;
IPAddress server(100, 64, 1, 25);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Instatiate clasess for libraries                                               *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo2(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo3(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo0(PCA9685_DEFAULT_ADDRESS, &Wire);

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);   
    }
    int read() {
      return client.read();      
    }
    void write(uint8_t* data, int length) {
      for (int i = 0; i < length; i++)
        client.write(data[i]);
    }
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

ros::NodeHandle_<WiFiHardware> nh;

void servo_cb(const sensor_msgs::JointState& msg){
  float sie = (msg.position[4]*180)/3.14;
  float si1 = (msg.position[0]*180)/3.14;
  float si2 = (msg.position[1]*180)/3.14;
  float si3 = -(msg.position[2]*180)/3.14;
  servoAngle_EE = 90 + sie;
  servoAngle_q1 = 90 + si1;
  servoAngle_q2 = 90 + si2;
  servoAngle_q3 = 90 + (si3 - si2);
  servo_move();
  if(check == 1){
    servo_display();
    check = 0;
  }
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void servo_move(){
  if(last_servoAngle_q1 != servoAngle_q1){
    Servo1.startEaseToD(servoAngle_q1, servoTime_q1);
    last_servoAngle_q1 = servoAngle_q1;
    check = 1;
  }
  if(last_servoAngle_q2 != servoAngle_q2){
    Servo2.startEaseToD(servoAngle_q2+12, servoTime_q2);
    last_servoAngle_q2 = servoAngle_q2;
    check = 1;
  }
  if(last_servoAngle_q3 != servoAngle_q3){
    Servo3.startEaseToD(servoAngle_q3+8, servoTime_q3);
    last_servoAngle_q3 = servoAngle_q3;
    check = 1;
  }
  if(last_servoAngle_EE != servoAngle_EE){
    Servo0.startEaseToD(servoAngle_EE, servoTime_EE);
    last_servoAngle_EE = servoAngle_EE;
    check = 1;
  }
}

void servo_init(){
  Servo0.write(last_servoAngle_EE);
  Servo1.write(last_servoAngle_q1);
  Servo2.write(last_servoAngle_q2+12);
  Servo3.write(last_servoAngle_q3+8);
}

void servo_display(){
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Servo:EE Position:%lf\n", servoAngle_EE);
  M5.Lcd.printf("Servo:01 Position:%lf\n", servoAngle_q1);
  M5.Lcd.printf("Servo:02 Position:%lf\n", servoAngle_q2);
  M5.Lcd.printf("Servo:03 Position:%lf\n", servoAngle_q3);

}

void wifi_start(){
  WiFi.begin(ssid, pass);
  while( WiFi.status() != WL_CONNECTED) {
    delay(500); 
    M5.Lcd.print("."); 
  }  
  M5.Lcd.println("WiFi connected");
  M5.Lcd.print("IP address = ");
  M5.Lcd.println(WiFi.localIP());
}

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.setTextSize(1);
  //WiFi Setting
  wifi_start();
  //ServoSetting
  Servo0.attach(0);
  Servo1.attach(1);
  Servo2.attach(2);
  Servo3.attach(3);
  Servo0.setEasingType(EASE_CUBIC_IN_OUT);
  Servo1.setEasingType(EASE_CUBIC_IN_OUT);
  Servo2.setEasingType(EASE_CUBIC_IN_OUT);
  Servo3.setEasingType(EASE_CUBIC_IN_OUT);
  servo_init();
  //ROS node Setting
  nh.initNode();
  nh.subscribe(sub);

  M5.Lcd.printf("Robot Ready\n");
}

void loop() {
  nh.spinOnce();
  delay(10);
}
