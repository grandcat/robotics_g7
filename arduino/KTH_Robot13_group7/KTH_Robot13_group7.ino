/*
 * Main embedded project for KTH DD2425
 *   No control
 *   Silly application of what you ask
 *
 * Send back the encoders change when receive a post on "/motion/Speed"
 *
 * Subscribes on topics :
 *      | "/motion/Speed"        to receive speed instructions
 *      | "/actuator/Servo"      to receive servo position
 *
 * Publishes on topics :
 *      | "/motion/Encoders"      send Encoders change
 *      | "/sensors/ADC"          send ADC values, including battery voltage
 */


#include <ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/AnalogC.h>
#include <differential_drive/Servomotors.h>

#include <math.h>

#include <Servo.h> 

#include <music.h> 


/* PinOut definition */
#define ChA_Dir 12
#define ChA_Pwm 3
#define ChA_Brk 9
#define ChA_CFb A0

#define ChB_Dir 13
#define ChB_Pwm 11
#define ChB_Brk 8
#define ChB_CFb A1

#define ChA_Encoder1 20
#define ChA_Encoder2 21

#define ChB_Encoder1 18
#define ChB_Encoder2 19


/* Battery monitoring const */
#define seuil_cell 3.6
#define seuil_batt 10.8
int led_pin[6] = {38,40,42,44,46,48};


/* Servomotors definition */
Servo servo[8];
char servo_pin[] = {22,24,26,28,30,32,34,36};


/* Lot of global variables */
int encoder1, encoder1_old ;
int encoder1_loc,encoder2_loc ;
int encoder2, encoder2_old ;

unsigned long time,time_old;
unsigned long t_enc, t_enc_old;
unsigned long wdtime ;

int cpt = 0;


/* ROS Use */
ros::NodeHandle  nh;

differential_drive::Encoders imlost ;
differential_drive::AnalogC Amsg ;

ros::Publisher p("/motion/Encoders", &imlost);  // Create a publisher to "/motion/Encoders" topic
ros::Publisher sensor("/sensors/ADC", &Amsg);  // Create a publisher to "/sensors/ADC" topic


/* Subscriber Callback */
void messagePWM( const differential_drive::PWM &cmd_msg){
  /* get the speed from message and apply it */
  MotorA.Set_speed(cmd_msg.PWM1);
  MotorB.Set_speed(cmd_msg.PWM2);
}


void messageServo(const differential_drive::Servomotors& params)  {
  for(int i=0;i<8;i++) {
           servo[i].write(params.servoangle[i]);
  }
}


/* Create Subscriber to "/motion/PWM" topic. Callback function is messageSpeed */
ros::Subscriber<differential_drive::PWM> subPWM("/motion/PWM", &messagePWM);


/* Create Subscriber to "/actuator/Servo" topic. Callback function is messageServo */
ros::Subscriber<differential_drive::Servomotors> subServo("/actuator/Servo", &messageServo);


void hello_world()  {
  for(int m=0;m<6;m++)  {
      digitalWrite(led_pin[m],HIGH);
      delay(50);
      digitalWrite(led_pin[m],LOW);
  }
  for(int m=4;m>=0;m--)  {
      digitalWrite(led_pin[m],HIGH);
      delay(50);
      digitalWrite(led_pin[m],LOW);
  }
  tone(7,700,50);
  //play_starwars();
  //play_tetris();
}


void setup()  {  
         /* Define interruptions, on changing edge */
         attachInterrupt(3,interrupt1,CHANGE);  // A
         attachInterrupt(2,interrupt2,CHANGE);  // A
         attachInterrupt(5,interrupt4,CHANGE);  // B
         attachInterrupt(4,interrupt3,CHANGE);  // B
         
         /* define the outputs pins */
         for(int i=0;i<6;i++) {
           pinMode(led_pin[i],OUTPUT);
         }
         pinMode(7,OUTPUT);
         
         
         /* Configure servomotors pins */
         for(int i=0;i<8;i++) {
           servo[i].attach(servo_pin[i]);
           
         }
         
         /* Initialize ROS stuff */
         nh.initNode();  // initialize node
         
         nh.advertise(p);  // advertise on p
         
         nh.advertise(sensor);  // advertise on sensor
         
         nh.subscribe(subPWM);  // Subscribe 
         nh.subscribe(subServo);  // Subscribe 
         
         /* Advertise booting */
         hello_world();
}



/*****************************
* Main Loop 
*
* Watchdog timer : if no message recieved during 2s, set the motors in stby,
* computer may have crashed.
******************************/
void loop()  {
  static unsigned long t;
  static unsigned long t_ADC;
  static boolean low_batt = false ;
  nh.spinOnce();
  /* Watchdog timer */
  if(millis()-wdtime > 2000)  { 
    MotorA.Set_speed(0);
    MotorB.Set_speed(0);
    wdtime = millis() ;
  }

  /* Read IR sensors value every 100ms */
  if(millis()-t_ADC>100)
  { 
    Amsg.ch1 = analogRead(A8);
    Amsg.ch2 = analogRead(A9);
    Amsg.ch3 = analogRead(A10);
    Amsg.ch4 = analogRead(A11);
    Amsg.ch5 = analogRead(A12);
    Amsg.ch6 = analogRead(A13);
    Amsg.ch7 = analogRead(A14);
    Amsg.ch8 = analogRead(A15);  
    
    /* Publish sensor value */
    sensor.publish(&Amsg);
    t_ADC = millis();
  }
  
    if(millis()-t > 1000)  {
    float v1 = analogRead(A7)*0.0049*1.5106;
    float v2 = analogRead(A6)*0.0049*2.9583;
    float v3 = analogRead(A5)*0.0049*2.9583;
    
    Amsg.cell1 = v1;
    Amsg.cell2 = v2 - v1; 
    Amsg.cell3 = v3 - v2;
    Amsg.on_batt = digitalRead(10);
    
    for(int m=0;m<floor((v3-seuil_batt)*4);m++)  {
      digitalWrite(led_pin[m],HIGH);
    }
    for(int m=floor((v3-seuil_batt)*4);m<6;m++)  {
      digitalWrite(led_pin[m],LOW);
    }
    
    if((Amsg.cell1<seuil_cell || Amsg.cell2<seuil_cell || Amsg.cell3<seuil_cell) && v1>2)  {
      low_batt = true ;
    }
    
    if(low_batt && ((Amsg.cell1>seuil_cell+0.2 || Amsg.cell2>seuil_cell+0.2 || Amsg.cell3>seuil_cell+0.2) || v1<2))  {
      low_batt = false;
    }
    
    if(low_batt)  {
      tone(7,440,500); 
    }

    t = millis();
  }
  
  // ENCODERS
  /* get the time since last call */
  time_old = time ;  
  time = millis() ;
  
  /* Store encoders value and publish */
  encoder1_loc = encoder1 ;
  encoder2_loc = encoder2 ;
  
  imlost.encoder1 = encoder1;
  imlost.encoder2 = encoder2;
  imlost.delta_encoder1 = encoder1_loc-encoder1_old;
  imlost.delta_encoder2 = encoder2_loc-encoder2_old;
  
  imlost.timestamp = time - time_old ;
  p.publish(&imlost);
  
  /* Store encoders value */
  encoder1_old = encoder1_loc;
  encoder2_old = encoder2_loc;
}


/*********************************************
 *
 *  Interruption subroutines
 *  Called each changing edge of an encoder
 *
 *********************************************/

void interrupt2() {
  if(digitalRead(ChA_Encoder2))
  {
    if (digitalRead(ChA_Encoder1))  encoder1--;
    else                  encoder1++;
  }
   else
  {
    if (!digitalRead(ChA_Encoder1)) encoder1--;
    else                  encoder1++;
  }
}

void interrupt1() {
  if(digitalRead(ChA_Encoder1))
  {
    if (!digitalRead(ChA_Encoder2)) encoder1--;
    else                  encoder1++;
  }
  else
  {
    if (digitalRead(ChA_Encoder2))  encoder1--;
    else                  encoder1++;
  }
}

void interrupt4() {
  if(digitalRead(ChB_Encoder2))
  {
    if (digitalRead(ChB_Encoder1))  encoder2++;
    else                  encoder2--;
  }
   else
  {
    if (!digitalRead(ChB_Encoder1)) encoder2++;
    else                  encoder2--;
  }
}

void interrupt3() {
  if(digitalRead(ChB_Encoder1))
  {
    if (!digitalRead(ChB_Encoder2)) encoder2++;
    else                  encoder2--;
  }
  else
  {
    if (digitalRead(ChB_Encoder2))  encoder2++;
    else                  encoder2--;
  }
}


// -----------------------------------------------------

/* END OF FILE */


