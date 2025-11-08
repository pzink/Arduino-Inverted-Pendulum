//Derived from https://github.com/krz4tw/Inverted-Pendulum-PID-control/blob/master/Cascading_PID_s_lin_kd_denoised.ino

#include "Arduino.h"
#include <PID_v1.h>
#include <Encoder.h> // this one: https://github.com/PaulStoffregen/Encoder
#include <AFMotor.h>
AF_DCMotor motor(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm

//Mechanical Sytem Properties
  //max linear travel ~8200 Encoder counts
  //full pendulum rotation =1024 Encoder counts

//Variable Definitions
  
//variables for end stops
  int valF = 0;
  int valR = 0;
  int inPinF = 9;
  int inPinR = 10;

  //Pin Definitions
    byte pin_lin_enc1=2;  //Need to use InterruptPin
    byte pin_lin_enc2=3;  //Need to use InterruptPin
    byte pin_rot_enc1=19;  //Need to use InterruptPin
    byte pin_rot_enc2=18;  //Need to use InterruptPin
    int potPin = A5;  //pin for potentiometer rot_offset adjustment

  //Speeds
    byte setup_drive_speed=155; //speed[pwm] used to drive on travel limit runs 
    byte setup_zero_speed=60;  //speed[pwm] at which the cart completely stops moving, used for mapping the PWM Output

  //Linear and angle Limits for Controller
  
    int limit_lin_min=-13000;    //min Limit (in direction torwards limit switch--> right)
    int limit_lin_max=13000;   //max Limit (left)
    double limit_angle=400; //limit at which it becomes impossible to catch the pendulum again if exeeded, active control of pendulum is 
    //stopped and cart is stopped if exeeded. 
    //Limit works in both direction (+ and -) at the equilibrium point
   
    
  //Miscellaneous Variables
    boolean button_current=LOW;
    boolean button_last=LOW;
    boolean poti_enable=LOW;
    int rot_offset=0;   //Offset from Vertical, System inherent error. Use if System always tends to run in one direction
    double timer;
    double lin_speed;
    double rot_speed;
    //lin_speed and angle_speed calculations
//a single controller cycle is not sufficient to calculate the speeds due to the "low" resolution of the encoders. 
//In one cycle the linear position reading might not even change even though its running at full speed (depending von the sample time).
//The speeds therefore are calculated by averaging over array_lenght controller cycles
    const int array_length=400; //this has a huge effect on performance!
    double lin_position_array[array_length];
    double angle_array[array_length];
    double timer_array[array_length];
    double angle = 0;
    double lin_position;

  //Buffer Arrays
   double timer_data[8000];
   double rot_Setpoint_data[8000];
   double lin_position_data[8000];
   double angle_data[8000];
   double Output_data[8000];

//Encoder Setups
  Encoder linEnc(pin_lin_enc1,pin_lin_enc2);                
  Encoder rotEnc(pin_rot_enc1,pin_rot_enc2);  
  
//PID Setup
    double Output, rot_Setpoint=0, lin_Setpoint=0;            //starting track in center will makde 0 approx. middle of track
///*
  //PID Control Variables
    double rot_kp=12, rot_ki=100, rot_kd=0.225;               //rot_kp=12, rot_ki=100, rot_kd=0.225; 
    double lin_kp=0.0025, lin_ki=0.000001, lin_kd=0.0015;    //lin_kp=0.0025, lin_ki=0.0000005, lin_kd=0.0015;  
//*/
/*
  //PI Control Variables
    double rot_kp=12, rot_ki=100, rot_kd=0;                 //Kp, Ki Only
    double lin_kp=0.0025, lin_ki=0.000001, lin_kd=0.0015;   // PID linear control 
*/
/*
  //Pd Control Variables
    double rot_kp=12, rot_ki=0, rot_kd=0.225;               //Kp, Kd Only
    double lin_kp=0.0025, lin_ki=0.000001, lin_kd=0.0015;   // PID linear control 
*/
/*
  //Tuning Control Variables
    double rot_kp=10, rot_ki=100, rot_kd=.2;                   //zero Ki and Kd for tuning, finding Kcr
    double lin_kp=0.0025, lin_ki=0.000001, lin_kd=0.0015;   // PID linear control
*/

//Controller initialisation
    PID rot_Controller(&angle,&Output,&rot_Setpoint,rot_kp,rot_ki,rot_kd,DIRECT);
    PID lin_Controller(&lin_position,&rot_Setpoint,&lin_Setpoint,lin_kp,lin_ki,lin_kd,REVERSE); 


void setup() {

       Serial.begin(57600); // >57600 will not work with Serial Plotter

//Calibration time!       
       //Initialize Encoder counts based on startup position
       rotEnc.write(0);                 //set when pendulum is hanging straight up
       linEnc.write(0);                 //Zero linear Encoder     
       delay(500);

//PID Settings
     rot_Controller.SetMode(AUTOMATIC);
     rot_Controller.SetOutputLimits(-255,255); 
     rot_Controller.SetSampleTime(2); //2
     lin_Controller.SetMode(AUTOMATIC);
     lin_Controller.SetOutputLimits(-255,255); 
     lin_Controller.SetSampleTime(2);
     
  //End Stops pin code

  pinMode(inPinF, INPUT);           // set pin to input
  digitalWrite(inPinF, LOW);       // turn on pulldown resistor
  pinMode(inPinR, INPUT);           // set pin to input
  digitalWrite(inPinR, LOW);       // turn on pulldown resistor
     
delay(300);
}

void loop() {
  //code for tuning rot_offset
   rot_offset=map(analogRead(potPin),1,695,-100,100); //to correct for non-vertical position
   //lin_kp=double(analogRead(potPin))/1000.000;
   //rot_kd=double(map(analogRead(potPin),1,695,0,500));

  //code for hard stops

  valF = digitalRead(inPinF);     // if the input pin on the forward direction crashes
  valR = digitalRead(inPinR);     // if the input pin on the reverse direction crashes

  while(valF || valR){               //you crashed, stop
    motor.run(RELEASE);
    valF = digitalRead(inPinF);     // if the input pin on the forward direction crashes
    valR = digitalRead(inPinR);     // if the input pin on the reverse direction crashes
  }
  
  //Pushing the linear postion values one step down in the array to make place at element [0] for the present linear position  
    for (int x=array_length; x>1; x--){
      lin_position_array[x-1]=lin_position_array[x-2];
    }
  //Pushing the angle values one step down in the array to make place at element [0] for the present angle    
    for (int x=array_length; x>1; x--){
      angle_array[x-1]=angle_array[x-2];
    }
 
  //Pushing the timer values one step down in the array to make place at element [0] for the present time in milliseconds    
    for (int x=array_length; x>1; x--){
      timer_array[x-1]=timer_array[x-2];
    }
    
  //reading the present values for timer, angle and linear postion and save them to element 0 in their arrays  
    timer=millis();
    timer_array[0]=timer;
    angle=rotEnc.read(); 
    angle_array[0]=angle;
    lin_position=linEnc.read();
    lin_position_array[0]=lin_position;
  //calculate lin_speed and rot_speed as averages over array_length controller cycles  
    lin_speed=(lin_position_array[0]-lin_position_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);
    rot_speed=(angle_array[0]-angle_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);

//Main Controller Part
  
    if (lin_position>limit_lin_min &&lin_position<limit_lin_max){       //perform as long carriage is within the linear limits
                if (abs(angle)<limit_angle){    //perform as long as pendulum is within the angular limits  
                                //Execute linear PID controller
                                  //lin_Controller.SetTunings(lin_kp,lin_ki,lin_kd); 
                                  lin_Controller.Compute();;
                                //Execute angular PID controller
                                  rot_Setpoint=rot_Setpoint+rot_offset;
                                  //rot_Controller.SetTunings(rot_kp,rot_ki,rot_kd); 
                                  rot_Controller.Compute();
                                    
                                  if(Output>0){
                                    Output=map(Output,0,255,setup_zero_speed,255);
                                    go_right(Output);
                                    }
                                    
                                  if(Output<0){
                                    Output=map(abs(Output),0,255,setup_zero_speed,255);
                                    go_left(Output);
                                    }
                              
                                  if(Output==0){
                                    go_stop();
                                    Output=0; //why is this here?
                                    }
                Serial.print(-5000);  // To freeze the lower limit
                Serial.print(" ");
                Serial.print(5000);  // To freeze the upper limit
                Serial.print(" ");
                Serial.print(lin_position);
                Serial.print(" ");
                Serial.println(lin_Setpoint);
                                    
                }
                //stop carrigage if pendulum has tipped out of the angular limits
                  else{   
                    go_stop();    
                    Output=0;
                  }   
            
    }
    //drive back into linear limits after they were exeeded
      else{ 
             go_stop();
          }
}
//End Loop

// slide position positive away from motor
//Procedure Declarations
  //move cart to the left (farther from motor) with 'velocity'
    void go_left(int velocity){       
       motor.setSpeed(velocity);
       motor.run(BACKWARD);  
      }
      
  //move cart to the right (closer to motor) with 'velocity'
    void go_right(int velocity){ 
       motor.setSpeed(velocity);
       motor.run(FORWARD); 
    }
    
  //stop the cart
    void go_stop(){    
      //motor.setSpeed(0);        
      motor.run(RELEASE); 
    }


