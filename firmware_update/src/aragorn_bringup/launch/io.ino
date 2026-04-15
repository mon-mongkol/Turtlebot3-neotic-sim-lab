
#define SSR_CHARGER_COPPER_BAR_PIN 1


#define ULTRASONIC_LEFT_PIN 39
#define ULTRASONIC_LEFT_FRONT_PIN 38
#define ULTRASONIC_MIDDLE_FRONT_PIN 37
#define ULTRASONIC_RIGHT_FRONT_PIN 36
#define ULTRASONIC_RIGHT_PIN 28


#define CLIFF_FRONT_PIN A22
#define CLIFF_LEFT_PIN A21
#define CLIFF_RIGHT_PIN A3

#define DHTPIN 24

#define IR_RECV_PIN 6

#define S_EMERGENCY_PIN 10
#define S_MASTER_ON_PIN 9
#define S_UVC_INTERLOCK_PIN 8
#define S_MASTER_ON_DONE_PIN 7
#define S_UVC_LAMP_STATE 3

#define SS_PIR_MOTION_1_PIN 31
#define SS_PIR_MOTION_2_PIN 14
#define SS_PIR_MOTION_3_PIN 15
#define SS_PIR_MOTION_4_PIN 16

#define SS_PIR_MOTION_5_PIN 27
#define SS_PIR_MOTION_6_PIN 21
#define SS_PIR_MOTION_7_PIN 22
#define SS_PIR_MOTION_8_PIN 23

#define IR_TRACK 5

#define RELAY_SYSTEM_READY_PIN 11
#define RELAY_UVC_MODE_ON_PIN 12
#define RELAY_UVC_MODE_OFF_PIN 29
#define RELAY_INTERLOCK_LAMP_PIN 33
#define RELAY_ROUTER_PIN 32
#define RELAY_TABLET_COMMU_LOSS 25
#define LED_STRIP_PIN 20


byte pir_motion_adds[] = {
                          SS_PIR_MOTION_1_PIN, 
                          SS_PIR_MOTION_2_PIN,
                          SS_PIR_MOTION_3_PIN,
                          SS_PIR_MOTION_4_PIN,
                          SS_PIR_MOTION_5_PIN,
                          SS_PIR_MOTION_6_PIN,
                          SS_PIR_MOTION_7_PIN,
                          SS_PIR_MOTION_8_PIN
                          };

byte scan_store[]={};
byte init_scan_store[]={};



IntervalTimer INTERLOCK_BLINK;
IntervalTimer LED_BLINK;


#include <ros.h>
#include <aragorn_msg/ServiceCmd.h>
#include <aragorn_msg/IRcommu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
//#include <aragorn_msg/CUbar.h>
#include <aragorn_msg/UVCcmd.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;



#include <DHT.h>
// #define DHTPIN 24
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);





#include <WS2812Serial.h>

const int LED_STRIP_NUMLED = 30;
// const int LED_STRIP_PIN = 26;
int led_color_code;
int led_mode  = 0;
int led_blink = 1;
int led_set   = 2;
int led_off   = 3;

byte drawingMemory[LED_STRIP_NUMLED*3];         //  3 bytes per LED
DMAMEM byte displayMemory[LED_STRIP_NUMLED*12]; // 12 bytes per LED

WS2812Serial leds(LED_STRIP_NUMLED, displayMemory, drawingMemory, LED_STRIP_PIN, WS2812_GRB);

#define red    0xFF0000
#define green  0x00FF00
#define blue   0xCCFFFF
#define yellow 0xFFFF00
#define pink   0xFF1088
#define orange 0xE05800
#define white  0xFFFFFF
#define black  0x000000








#include "Ultrasonic.h"


// #define ULTRASONIC_LEFT_PIN 30
// #define ULTRASONIC_LEFT_FRONT_PIN 35
// #define ULTRASONIC_MIDDLE_FRONT_PIN 36
// #define ULTRASONIC_RIGHT_FRONT_PIN 37
// #define ULTRASONIC_RIGHT_PIN 38

char frameid_ultrasonic_left[] = "ultrasonic_left_link";
char frameid_ultrasonic_left_front[] = "ultrasonic_left_front_link";
char frameid_ultrasonic_middle_front[] = "ultrasonic_middle_front_link";
char frameid_ultrasonic_right_front[] = "ultrasonic_right_front_link";
char frameid_ultrasonic_right[] = "ultrasonic_right_link";

char *ultrasonic_frame[] = {  frameid_ultrasonic_left, 
                              frameid_ultrasonic_left_front, 
                              frameid_ultrasonic_middle_front, 
                              frameid_ultrasonic_right_front, 
                              frameid_ultrasonic_right
                           }; 
                          
byte ultrasonic_address[] = { ULTRASONIC_LEFT_PIN, 
                              ULTRASONIC_LEFT_FRONT_PIN,
                              ULTRASONIC_MIDDLE_FRONT_PIN, 
                              ULTRASONIC_RIGHT_FRONT_PIN, 
                              ULTRASONIC_RIGHT_PIN
                            };


// #define CLIFF_FRONT_PIN A21
// #define CLIFF_LEFT_PIN A22
// #define CLIFF_RIGHT_PIN A3

char frameid_cliff_front[] = "cliff_front_link";
char frameid_cliff_left[] = "cliff_left_link";
char frameid_cliff_right[] = "cliff_right_link";

char *cliff_frame[] = {   frameid_cliff_front,
                          frameid_cliff_left,
                          frameid_cliff_right,
                      };

byte cliff_address[] = {  CLIFF_FRONT_PIN,
                          CLIFF_LEFT_PIN,
                          CLIFF_RIGHT_PIN
                       };




                            
#include <IRremote.h>
// #define IR_RECV_PIN 3
IRrecv irrecv(IR_RECV_PIN);
IRsend irsend;
decode_results results;


enum copperbar_mode{  DISABLE_COPPER_BAR,
                      ENABLE_COPPER_BAR
                   };


enum uvc_state{ OFF_UVC,
                ON_UVC
              };

enum robot_mode{  INITIAL,
                  IDLE,
                  START_MOTOR,
                  SHUTDOWN_MOTOR,
                  SHUTDOWN_ROBOT,
                  EMERGENCY,
                  };

uvc_state uvc_set = OFF_UVC;

int counter_sys_ready = 0;
int counter_emer = 0;
int counter_reset_router = 0;
int counter_master_on_done = 0;
bool system_ready = false;
bool temp_status = false;
bool _set_emer = false;
bool _set_ready =false;
bool _set_motor = false;
bool enable_counter_reset_router = false;
bool led_on = false;
bool docking_mode = false;
unsigned long start_counter_reset_router;
unsigned long goal_counter_reset_router;
unsigned long time, tablet_time_stamp_, tablet_time_stamp_old_;



void IrRemote_callback(const aragorn_msg::IRcommu::Request & req, aragorn_msg::IRcommu::Response & res)
{
    
    IR_transmit(req.cmd);

    res.res = req.cmd;

//    nh.loginfo("IR remote send step %ld to docking");
  
}

void IR_transmit(int cmd)
{
    for(int i; i<50; i++)
    {
        irsend.sendSony(cmd, 20);
        delay(40);
        digitalToggle(LED_BUILTIN);
    }
}


void LedStatus_callback(const aragorn_msg::ServiceCmd::Request & req, aragorn_msg::ServiceCmd::Response & res)
{
  String cmd_color = String(req.color);
  String cmd_mode = String(req.mode);
                        
  if(cmd_color == "green")
  {
    led_color_code = green;
  }
  else if(cmd_color == "red")
        {
          led_color_code = red;
        }
        else if(cmd_color == "blue")
              {
                led_color_code = blue;
              }
              else if(cmd_color == "yellow")
                    {
                      led_color_code = yellow;
                    }
                    else if(cmd_color == "pink")
                          {
                            led_color_code = pink;
                          }
                          else if(cmd_color == "orange")
                                {
                                  led_color_code = orange;
                                }
                                else if(cmd_color == "white")
                                      {
                                        led_color_code = white;
                                      }
                                      else if(cmd_color == "black")
                                            {
                                              led_color_code = black;
                                            }
                                            else
                                            {
                                              res.status = "No color";
                                            }

  if(cmd_mode == "blink")
  {
    led_mode = led_blink;
  }
  else if(cmd_mode == "set")
       {
         led_mode = led_set;
       }
       else if(cmd_mode == "off")
            {
              led_mode = led_off;
            }
  
  res.status = "led on";
}



void ChargerCuBar_callback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data == true)
  {
    digitalWrite(SSR_CHARGER_COPPER_BAR_PIN, LOW);
    if(digitalRead(SSR_CHARGER_COPPER_BAR_PIN) == LOW)
    {
      res.success = true;
      res.message = "connect charger";
    }else ;
  }else
  {
    digitalWrite(SSR_CHARGER_COPPER_BAR_PIN, HIGH);
    if(digitalRead(SSR_CHARGER_COPPER_BAR_PIN) == HIGH)
    {
      res.success = true;
      res.message = "disconnect charger";
    }
  }
}

void uvc_callback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data == true)
  { 
      // uvc_set = ON_UVC;
      if((digitalRead(S_MASTER_ON_DONE_PIN) == HIGH) and (digitalRead(S_UVC_INTERLOCK_PIN) == HIGH))
      {
        uvc_set = ON_UVC;
      }else 
      {
        uvc_set = OFF_UVC;
      }

      uvc_update();

      for(int i=0;i<14000;i++) //wait all uvc lamp on
      {
        nh.spinOnce();
        delay(1);
      }

      if((digitalRead(S_UVC_LAMP_STATE) == HIGH) && (uvc_set = ON_UVC))
      {
          res.success = true;
          res.message = "UVC lamp on";
          uvc_set = ON_UVC;
          goto end_state;
      }
      else
      {
          res.success = false;
          res.message = "UVC lamp off";
          uvc_set = OFF_UVC;
      }
  }
  else 
  {
      uvc_set = OFF_UVC;
      res.success = false;
      res.message = "UVC lamp off";
  }

  uvc_update();

  end_state:

  Serial.print("");
  
} 


///ros::NodeHandle  nh;
void DockingMode_callback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data == true)
  {
    docking_mode = true;
    res.success = true;
    res.message = "set Dockinging mode";
  }
  else
  {
    docking_mode = false;
    res.success = false;
    res.message = "turn off docking mode";
  }
}

void scan_motion(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  int scan_timeout = 3000;
  int couter_timeout=0;
  bool scan_result = false;

  if(req.data == true)
  {
    for(int i = 0; i<sizeof(pir_motion_adds) ; i++) //store and init all pir motion state
    {
      init_scan_store[i] = digitalRead(pir_motion_adds[i]);
    }
    
    
    while(1)
    {

      nh.spinOnce();

      for(int i=0; i<8 ; i++)
      {
          if(init_scan_store[i] != digitalRead(pir_motion_adds[i]))
          {
            scan_result = false;
            i=1000;
          }
          else
          {
            scan_result = true;
          }
      }
      
      if((couter_timeout >= 5000) || (scan_result == false)) //motion scan timeout 3s 
      {
          break;
      }
      else
      {
          couter_timeout++;
          delay(1);
      }
    }
      
  }
  else
  {
      scan_result = false;
  }

  if(scan_result == true)
  {
      res.success = scan_result;
      res.message = "Scan results : Safty";
  }
  else
  {
      res.success = scan_result;
      res.message = "Scan results : something still moving";
  }
  
  
}

void cbSystemReady(const std_msgs::Bool &system_msgs)
{
  if(system_msgs.data == true)
  {
    system_ready = true;
  }
  else
  {
    system_ready = false;
  }
} 

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cbTablet_commu(const geometry_msgs::Vector3& acc_msg)
{
  tablet_time_stamp_ = millis();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////



ros::ServiceServer<aragorn_msg::ServiceCmd::Request, aragorn_msg::ServiceCmd::Response> server_led("/aragorn_io/led_status", &LedStatus_callback);
ros::ServiceServer<aragorn_msg::IRcommu::Request, aragorn_msg::IRcommu::Response> server_IRremote("/aragorn_io/ir_remote", &IrRemote_callback);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> server_charger_cu_bar("/aragorn_io/charger_cu_bar", &ChargerCuBar_callback);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> server_uvc("/aragorn_io/uvc", &uvc_callback);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> server_scan_motion("/aragorn_io/scan_motion", &scan_motion);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> server_docking("/aragorn_io/docking_mode", &DockingMode_callback);

ros::Subscriber<std_msgs::Bool> sub_system_ready("/aragorn/diagnostics/system_ready", cbSystemReady);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ros::Subscriber<geometry_msgs::Vector3> sub_tab_acc("/accelerometer", cbTablet_commu); //check tablet communicate ros 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// sensor_msgs::Temperature temp_msg;
// ros::Publisher pub_temp("/aragorn_io/temperature", &temp_msg);

std_msgs::Int8 ir_track_msg;
ros::Publisher pub_ir_track("/aragorn_io/ir_track", &ir_track_msg);


std_msgs::Bool teensy_msg;
ros::Publisher pub_diag("/aragorn_io/diagnostics", &teensy_msg);

std_msgs::Int8 uvc_msg;
ros::Publisher pub_uvc_state("/aragorn_io/uvc_state", &uvc_msg);

sensor_msgs::Range ultrasonic_msg;
ros::Publisher pub_ultrasonic_left( "/aragorn_io/ultrasonic/left", &ultrasonic_msg);
ros::Publisher pub_ultrasonic_left_front( "/aragorn_io/ultrasonic/left_front", &ultrasonic_msg);
ros::Publisher pub_ultrasonic_middle_front( "/aragorn_io/ultrasonic/middle_front", &ultrasonic_msg);
ros::Publisher pub_ultrasonic_right_front( "/aragorn_io/ultrasonic/right_front", &ultrasonic_msg);
ros::Publisher pub_ultrasonic_right( "/aragorn_io/ultrasonic/right", &ultrasonic_msg);

std_msgs::Int8 cliff_msg;
ros::Publisher pub_cliff_front("/aragorn_io/cliff/front", &cliff_msg);
ros::Publisher pub_cliff_left("/aragorn_io/cliff/left", &cliff_msg);
ros::Publisher pub_cliff_right("/aragorn_io/cliff/right", &cliff_msg);


std_msgs::Int64 ir_recv_msg;
ros::Publisher pub_ir_recv( "/aragorn_io/ir_recv", &ir_recv_msg);

std_msgs::Int8 emer_msg;
ros::Publisher pub_emer("/aragorn_io/emergency", &emer_msg);

std_msgs::Int8 mode_msg;
ros::Publisher pub_robot_mode("/aragorn/robot_mode", &mode_msg);


void setup() 
{
  
  Serial.begin(115200);

  
  dht.begin();

  
  leds.begin();
  led_color_set(black);

  pinMode(SSR_CHARGER_COPPER_BAR_PIN, OUTPUT);
  digitalWrite(SSR_CHARGER_COPPER_BAR_PIN, HIGH);

  pinMode(ULTRASONIC_LEFT_PIN, INPUT);
  pinMode(ULTRASONIC_LEFT_FRONT_PIN, INPUT);
  pinMode(ULTRASONIC_MIDDLE_FRONT_PIN, INPUT);
  pinMode(ULTRASONIC_RIGHT_FRONT_PIN, INPUT);
  pinMode(ULTRASONIC_RIGHT_PIN, INPUT);
  

  pinMode(CLIFF_FRONT_PIN, INPUT);
  pinMode(CLIFF_LEFT_PIN, INPUT);
  pinMode(CLIFF_RIGHT_PIN, INPUT);

  pinMode(S_EMERGENCY_PIN, INPUT);
  pinMode(S_MASTER_ON_PIN, INPUT);
  pinMode(S_UVC_INTERLOCK_PIN, INPUT);
  pinMode(S_MASTER_ON_DONE_PIN, INPUT);
  pinMode(S_UVC_LAMP_STATE, INPUT);

  pinMode(RELAY_SYSTEM_READY_PIN, OUTPUT);
  pinMode(RELAY_UVC_MODE_ON_PIN, OUTPUT);
  pinMode(RELAY_UVC_MODE_OFF_PIN, OUTPUT);
  pinMode(RELAY_INTERLOCK_LAMP_PIN, OUTPUT);
  pinMode(RELAY_ROUTER_PIN, OUTPUT);
  pinMode(RELAY_TABLET_COMMU_LOSS, OUTPUT);

  digitalWrite(RELAY_SYSTEM_READY_PIN, LOW);
  digitalWrite(RELAY_INTERLOCK_LAMP_PIN, LOW);
  digitalWrite(RELAY_ROUTER_PIN, HIGH); // turn on router
  digitalWrite(RELAY_UVC_MODE_ON_PIN, LOW);
  digitalWrite(RELAY_UVC_MODE_OFF_PIN, LOW);
  digitalWrite(RELAY_TABLET_COMMU_LOSS, LOW);

  pinMode(SS_PIR_MOTION_1_PIN, INPUT_PULLUP);
  pinMode(SS_PIR_MOTION_2_PIN, INPUT_PULLUP);
  pinMode(SS_PIR_MOTION_3_PIN, INPUT_PULLUP);
  pinMode(SS_PIR_MOTION_4_PIN, INPUT_PULLUP);
  pinMode(SS_PIR_MOTION_5_PIN, INPUT_PULLUP);
  pinMode(SS_PIR_MOTION_6_PIN, INPUT_PULLUP);
  pinMode(SS_PIR_MOTION_7_PIN, INPUT_PULLUP);
  pinMode(SS_PIR_MOTION_8_PIN, INPUT_PULLUP);
  
  pinMode(IR_TRACK, INPUT_PULLUP);

  pinMode(LED_BUILTIN,OUTPUT);    // LED
  digitalWrite(LED_BUILTIN,LOW);  // LED off   

  INTERLOCK_BLINK.begin(blinkInterlockPB, 500000);
  LED_BLINK.begin(blinkLed, 400000);
  
  nh.initNode();
  
  nh.advertiseService(server_led);
  nh.advertiseService(server_IRremote);
  nh.advertiseService(server_charger_cu_bar);
  nh.advertiseService(server_uvc);
  nh.advertiseService(server_scan_motion);
  nh.advertiseService(server_docking);
  
  nh.advertise(pub_ultrasonic_left);
  nh.advertise(pub_ultrasonic_left_front);
  nh.advertise(pub_ultrasonic_middle_front);
  nh.advertise(pub_ultrasonic_right_front);
  nh.advertise(pub_ultrasonic_right);
  
  nh.advertise(pub_cliff_front);
  nh.advertise(pub_cliff_left);
  nh.advertise(pub_cliff_right);

  nh.advertise(pub_emer);
  nh.advertise(pub_ir_recv);
  nh.advertise(pub_ir_track);
  nh.advertise(pub_diag);

  nh.advertise(pub_robot_mode); 

  nh.advertise(pub_uvc_state);

  nh.subscribe(sub_system_ready);
  nh.subscribe(sub_tab_acc);

  irrecv.enableIRIn();

  while(!nh.connected())
  {
    nh.spinOnce();
  }
  delay(100);
  nh.loginfo("aragorn I/O connected");
  



}

void blinkInterlockPB()
{
    if(nh.connected())
    {
        if(digitalRead(S_MASTER_ON_DONE_PIN) == HIGH)   ////////////////////////////////////// Interlock check
        {
      
              if(digitalRead(S_UVC_INTERLOCK_PIN) == LOW)
              {
                digitalWrite(RELAY_INTERLOCK_LAMP_PIN, HIGH-digitalRead(RELAY_INTERLOCK_LAMP_PIN));
                uvc_set = OFF_UVC;
                counter_master_on_done=0;
              }
              else 
              {
                digitalWrite(RELAY_INTERLOCK_LAMP_PIN, HIGH);
                
              }
        }
    }
    else
    {
      digitalWrite(RELAY_INTERLOCK_LAMP_PIN, LOW);
    }

}


void blinkLed()
{
  if(nh.connected()) 
  {
          if(led_mode == led_blink)
          {
              if(led_on == false)
              {
                  for (int i=0; i < leds.numPixels(); i++) 
                  {
                    leds.setPixel(i, led_color_code);
                    leds.show();
                  }
                  led_on = true;
              }
              else
              {
                  for (int i=0; i < leds.numPixels(); i++) 
                  {
                    leds.setPixel(i, black);
                    leds.show();
                  }
                  led_on = false;
              }
          }
          else if(led_mode == led_set)
          {
            for (int i=0; i < leds.numPixels(); i++) 
            {
              leds.setPixel(i, led_color_code);
              leds.show();
            }
          }
          else if(led_mode == led_off)
          {
            for (int i=0; i < leds.numPixels(); i++) 
            {
              leds.setPixel(i, black);
              leds.show();
            }
          }
          else
          {
            for (int i=0; i < leds.numPixels(); i++) 
            {
              leds.setPixel(i, black);
              leds.show();
            }
          }

  }
}


int time_period;

void loop() 
{

  
  
  if(nh.connected())
  {

  

    if(docking_mode == true)
    {
      long time_now = millis();
      if(time_now >= time_period+150.0)
      {
        IR_receive_decode();
        time_period = time_now;
      }
      
      IR_Track();
    }
    else
    {
  
      ultrasonic_update();

      // cliff_update();

      test_cliff();
      // test_ultrasonic();

      // uvc_update();

    }

    botton_check();

    system_check();

    update_uvc_state();

    temperature_update();
    


   


    
    nh.spinOnce();
    delay(10);
  }
  else
  {
    nh.loginfo("aragorn I/O disconnected");
    Serial.print("aragorn I/O disconnected");
    digitalWrite(LED_BUILTIN, LOW);
    led_color_set(black);

    digitalWrite(RELAY_SYSTEM_READY_PIN, LOW);

    digitalWrite(RELAY_INTERLOCK_LAMP_PIN, LOW);

    digitalWrite(RELAY_RESET_ROUTER_PIN, LOW);

    digitalWrite(SSR_CHARGER_COPPER_BAR_PIN, HIGH);

    // digitalWrite(RELAY_UVC_MODE_ON_PIN, LOW);
    // digitalWrite(RELAY_UVC_MODE_OFF_PIN, HIGH);
    
    uvc_set = OFF_UVC;
    uvc_off();

    led_mode = led_off;
    led_color_code = black;
    nh.spinOnce();
    delay(10);
  }


}

void update_uvc_state()
{
  if((digitalRead(S_UVC_LAMP_STATE) == HIGH) && (uvc_set == ON_UVC))
  {
    uvc_set = ON_UVC;
    uvc_msg.data = 1;
    pub_uvc_state.publish(&uvc_msg);
  }
  else
  {
    uvc_set = OFF_UVC;
    uvc_msg.data = 0;
    pub_uvc_state.publish(&uvc_msg);
  }
  
}
int uvc_set_old;

void uvc_update()
{
  // if(uvc_set != uvc_set_old)
  // {
  if((uvc_set == ON_UVC) && (digitalRead(S_UVC_INTERLOCK_PIN) == HIGH))
  {
    uvc_on();
  }
  else
  { 
    uvc_off();
    uvc_set == OFF_UVC;
  }
  // }
  // else;

  uvc_set_old = uvc_set;
}

void uvc_on()
{
    digitalWrite(RELAY_UVC_MODE_OFF_PIN, LOW);
    delay(1000);
    digitalWrite(RELAY_UVC_MODE_ON_PIN, HIGH);
    // delay(1000);
    // digitalWrite(RELAY_UVC_MODE_ON_PIN, LOW);
}

void uvc_off()
{
    digitalWrite(RELAY_UVC_MODE_ON_PIN, LOW);  
    delay(1000);
    digitalWrite(RELAY_UVC_MODE_OFF_PIN, HIGH);
    
    // digitalWrite(RELAY_UVC_MODE_OFF_PIN, LOW);
    
}


void botton_check()
{

    if((digitalRead(S_MASTER_ON_PIN) == HIGH) && (digitalRead(S_EMERGENCY_PIN) == LOW) && (system_ready == true) && (digitalRead(S_MASTER_ON_DONE_PIN) == LOW)) ////////////////////// System Ready
    {
      counter_sys_ready++;
      delay(1);
      if(counter_sys_ready >= 10)
      {
        digitalWrite(RELAY_SYSTEM_READY_PIN, HIGH);
        // S_MASTER_ON_DONE_PIN == HIGH;
        mode_msg.data = START_MOTOR;
        pub_robot_mode.publish(&mode_msg);
      }
    }else
    {
      counter_sys_ready = 0;
    }



    
    if((digitalRead(S_EMERGENCY_PIN) == HIGH))     ////////////////////////////////////// Emergency Pub
    {
     counter_emer++;
     delay(1);
     if(counter_emer >= 50)
     {

      digitalWrite(RELAY_SYSTEM_READY_PIN, LOW);
      digitalWrite(RELAY_UVC_MODE_ON_PIN, LOW);
      digitalWrite(RELAY_UVC_MODE_OFF_PIN, LOW);
      digitalWrite(RELAY_INTERLOCK_LAMP_PIN, LOW);
      digitalWrite(RELAY_RESET_ROUTER_PIN, LOW);
      digitalWrite(SSR_CHARGER_COPPER_BAR_PIN, HIGH);
      uvc_set = OFF_UVC;
//      uvc_off();
      counter_sys_ready = 0;
      emer_msg.data = digitalRead(S_EMERGENCY_PIN);
      pub_emer.publish(&emer_msg);
      mode_msg.data = SHUTDOWN_MOTOR;
      pub_robot_mode.publish(&mode_msg);

     }
    }
    else
    {
      counter_emer = 0;
      emer_msg.data = digitalRead(S_EMERGENCY_PIN);
      pub_emer.publish(&emer_msg);

    }



    if(digitalRead(S_MASTER_ON_PIN) == HIGH)   ////////////////////////////////////// Router reset
    {
      if(enable_counter_reset_router == false)
      {
        goal_counter_reset_router = millis() + 4700;
        enable_counter_reset_router = true;
      }else ; 
      time = millis();
      if(time >= goal_counter_reset_router)
      {
        digitalWrite(RELAY_RESET_ROUTER_PIN, HIGH);
        delay(200);
        digitalWrite(RELAY_RESET_ROUTER_PIN, LOW);
        enable_counter_reset_router = false;
      }
      else
      {
        digitalWrite(RELAY_RESET_ROUTER_PIN, LOW);
      }
    }
    else
    {
      counter_reset_router = 0;
      enable_counter_reset_router = false;
    }
}



void emergency_mode()
{
    digitalWrite(RELAY_SYSTEM_READY_PIN, LOW);
    digitalWrite(RELAY_INTERLOCK_LAMP_PIN, LOW);
    digitalWrite(RELAY_RESET_ROUTER_PIN, LOW);
    digitalWrite(SSR_CHARGER_COPPER_BAR_PIN, HIGH);

    digitalWrite(RELAY_UVC_MODE_OFF_PIN, HIGH);
    digitalWrite(RELAY_UVC_MODE_ON_PIN, LOW);

    
}

int ir_old, ir_new, ir_counter, ir_pub;

void IR_Track()
{
    if (digitalRead(IR_TRACK) == 0) 
    {
        ir_new = 1;
        //irrecv.resume(); // Receive the next value
    }
    else
    {
      ir_new = 0;
    }

    if(ir_new == ir_old)
    {
      ir_counter++;
      if(ir_counter >= 20)
      {
        ir_pub = 0;
      }
    }
    else
    {
      ir_pub = 1;
      ir_counter=0;
    }

    ir_old = ir_new;
    // ir_recv_msg.data = ir_pub;
    ir_track_msg.data = ir_pub;
    Serial.println(ir_pub);
    pub_ir_track.publish(&ir_track_msg);

    delay(15);
}

enum IR_CMD{  INIT,
              FOLLOWTRACK,
              STOP,
              NOT_FOUND,
              NO_MEMBER,
              FULLCHARGE
           };
int ir_decode_pub;

void IR_receive_decode()
{
  if(irrecv.decode(&results))
  {
    if(results.value == IR_CMD::FOLLOWTRACK)
    {
      ir_decode_pub = IR_CMD::FOLLOWTRACK;
    }
    else if(results.value == IR_CMD::STOP)
    {
      ir_decode_pub = IR_CMD::STOP;
    }
    else if(results.value == IR_CMD::FULLCHARGE)
    {
      ir_decode_pub = IR_CMD::FULLCHARGE;
    }else
    {
      ir_decode_pub = IR_CMD::NO_MEMBER;
    }
    irrecv.resume();

  }
  else
  {
    ir_decode_pub = IR_CMD::NOT_FOUND;
  }

  ir_recv_msg.data = ir_decode_pub;
  pub_ir_recv.publish(&ir_recv_msg);
  delay(15);

}
// void IR_receive_decode()
// {
//     if (irrecv.decode(&results)) 
//     {
//         ir_new = 1;
//         irrecv.resume(); // Receive the next value
//     }
//     else
//     {
//       ir_new = 0;
//     }

//     if(ir_new == ir_old)
//     {
//       ir_counter++;
//       if(ir_counter >= 20)
//       {
//         ir_pub = 0;
//       }
//     }
//     else
//     {
//       ir_pub = 1;
//       ir_counter=0;
//     }

//     ir_old = ir_new;
//     ir_recv_msg.data = ir_pub;
//     Serial.println(ir_pub);
//     pub_ir_recv.publish(&ir_recv_msg);

//     delay(15);
// }




 void temperature_update()
 {
//  float t= dht.readTemperature();
//  if(t <= 50)
//  {
    temp_status = true;
//  }
//  else
//  {
//    temp_status = false;
//  }
//  temp_msg.header.stamp = nh.now();
//  temp_msg.temperature = t;
//  temp_msg.variance = 0;
//  pub_temp.publish(&temp_msg);
 }


void system_check()
{
  if(temp_status == false)
  {
    teensy_msg.data = false;
//    emergency_mode();
  }
  else
  {
   teensy_msg.data = true;
   
  }
  pub_diag.publish(&teensy_msg);
  
}





void led_color_set(int color)
{
  for (int j=leds.numPixels(); j >= 0; j--) 
  {
    leds.setPixel(j, color);
    leds.show();
  }
}




void ultrasonic_update()
{
  for (int i = 0; i < sizeof(ultrasonic_address); i++)
  {
      ultrasonic_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
      ultrasonic_msg.header.frame_id =  ultrasonic_frame[i];
      ultrasonic_msg.field_of_view = 0.267;
      ultrasonic_msg.min_range = 0.001;
      ultrasonic_msg.max_range = 1.5000;

      /////////////////////////////////////////////////////////////

      pinMode(ultrasonic_address[i], OUTPUT);
      digitalWrite(ultrasonic_address[i], LOW);
      delayMicroseconds(2);
      digitalWrite(ultrasonic_address[i], HIGH);
      delayMicroseconds(5);
      digitalWrite(ultrasonic_address[i], LOW);
      pinMode(ultrasonic_address[i], INPUT);
      long duration;
      duration = pulseIn(ultrasonic_address[i], HIGH, 7000); //timeout 1500us for max_range 0.2 m
      float RangeInCentimeters;
      
      if(duration == 0.000000)
      {
        RangeInCentimeters = 150.0;
      }
      else
      {
        RangeInCentimeters = duration / 29 / 2 ;
      }

      if(RangeInCentimeters >= 150.0)
      {
        RangeInCentimeters = 150.0;
      }
      else;

      ////////////////////////////////////////////////////////////

      ultrasonic_msg.range = RangeInCentimeters /100;
      ultrasonic_msg.header.stamp = nh.now();
      
      switch(i)
      {
        case 0:
          pub_ultrasonic_left.publish(&ultrasonic_msg);
          break;
          
        case 4:
          pub_ultrasonic_right.publish(&ultrasonic_msg); 
          break;

        case 1:
          pub_ultrasonic_left_front.publish(&ultrasonic_msg); 
          break;

        case 2:
          pub_ultrasonic_middle_front.publish(&ultrasonic_msg); 
          break;

        case 3:
          pub_ultrasonic_right_front.publish(&ultrasonic_msg); 
          break;
      }
  }
}



void test_cliff()
{
    float volts = analogRead(CLIFF_FRONT_PIN)*0.00322265625;
    int distance = 13*pow(volts,-1);
    cliff_msg.data = distance;
    pub_cliff_front.publish(&cliff_msg);

    volts = analogRead(CLIFF_LEFT_PIN)*0.00322265625;
    distance = 13*pow(volts,-1);
    cliff_msg.data = distance;
    pub_cliff_left.publish(&cliff_msg);

    volts = analogRead(CLIFF_RIGHT_PIN)*0.00322265625;
    distance = 13*pow(volts,-1);
    cliff_msg.data = distance;
    pub_cliff_right.publish(&cliff_msg);
}