#include <RP2040_PWM.h>
#include "RPi_Pico_TimerInterrupt.h" 
#include "RPi_Pico_ISR_Timer.h"
#include "pico/stdio.h"

RPI_PICO_Timer ITimer(0); 

#define TIMER_FREQ_HZ 1000


int pwm_pin=16 ;// pins 
int pin1=20;// used
float freq=25000.0f;
float duty_cycle=0.0f;

int enc1=17;// pins used
int enc2=18;// for encoder

double kp=0.01;  // range 0.0001~0.01, with prints 0.0002~0.005
double kd=0.15;//0.5;//1.5;
double ki=0;
double u=0; // pid control signal
double deltaT;


int pwm_pin2= 12;// pins 
int pin2=13;// used
float duty_cycle2=0.0f;

int enc12=14;// pins used
int enc22=15;// for encoder

double kp2=0.01;
double kd2=0.15;//0.5;//1.5;
double ki2=0;
double u2=0; // pid control signal

int pos=0; // variable that keeps position of motor
int prev_pos=0;
int currmillis=0; // for calculating
int prevmillis=0; // deltaT
int e=0; // error
int eprev=0;
int eint=0;
double dedt=0;
int target=0; // target angle in pulses
int target_angle1; // 0 for old angle , 1 for new angle


int pos2=0; // variable that keeps position of motor
int prev_pos2=0;
int e2=0; // error
int eprev2=0;
int eint2=0;
double dedt2=0;
int target2=0; // target angle in pulses
int target_angle2; // 0 for old angle , 1 for new angle

char buffer[8]; 
int len=0;

RP2040_PWM* PWM_Instance;

RP2040_PWM* PWM_Instance2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pwm_pin,OUTPUT);
  pinMode(pin1,OUTPUT);
  pinMode(enc1,INPUT);
  pinMode(enc2,INPUT);
  //pinMode(control_checkpin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(enc1),isrEncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2),isrEncoder,CHANGE);

  pinMode(pwm_pin2,OUTPUT);
  pinMode(pin2,OUTPUT);
  pinMode(enc12,INPUT);
  pinMode(enc22,INPUT);
  attachInterrupt(digitalPinToInterrupt(enc12),isrEncoder2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc22),isrEncoder2,CHANGE);

  PWM_Instance= new RP2040_PWM(pwm_pin,freq,duty_cycle);
  if(PWM_Instance)
  {
    PWM_Instance->setPWM();
  }

  PWM_Instance2= new RP2040_PWM(pwm_pin2,freq,duty_cycle2);
  if(PWM_Instance2)
  {
    PWM_Instance2->setPWM();
  }

  while(!Serial){
// waits until Serial monitor is connected
  }

  if (ITimer.attachInterrupt(TIMER_FREQ_HZ, TimerHandler)){
    Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  }else{
     Serial.println("Can't set ITimer. Select another freq. or timer");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  write_buffer();
   

// Serial.print("position:");
// Serial.print(pos);
// Serial.print(" ");
// Serial.print("target:");
// Serial.print(target);
// Serial.print(" ");
// Serial.print("u:");
// Serial.println(u);
// Serial.print(" ");
// Serial.print("error:");
// Serial.print(e);
// Serial.print(" ");
// Serial.print("duty cycle:");
// Serial.println(duty_cycle);


}


void isrEncoder() {
  // Read current states of encoder A and B
  static uint8_t prevState=0b00;
  bool currentA = digitalRead(enc1);
  bool currentB = digitalRead(enc2);

    // Encode the current state
  uint8_t currentState = (currentA << 1) | currentB;

    // Determine direction based on state transitions
  if ((prevState == 0b00 && currentState == 0b01) || 
        (prevState == 0b01 && currentState == 0b11) || 
        (prevState == 0b11 && currentState == 0b10) || 
        (prevState == 0b10 && currentState == 0b00)) {
        pos++; // Forward direction
  } else if ((prevState == 0b00 && currentState == 0b10) || 
               (prevState == 0b10 && currentState == 0b11) || 
               (prevState == 0b11 && currentState == 0b01) || 
               (prevState == 0b01 && currentState == 0b00)) {
        pos--; // Reverse direction
  }

    // Update previous state
  prevState = currentState;
}

void isrEncoder2() {
    // Read current states of encoder A and B
    static uint8_t prevState=0b00;
    bool currentA = digitalRead(enc12);
    bool currentB = digitalRead(enc22);

    // Encode the current state
    uint8_t currentState = (currentA << 1) | currentB;

    // Determine direction based on state transitions
    if ((prevState == 0b00 && currentState == 0b01) || 
        (prevState == 0b01 && currentState == 0b11) || 
        (prevState == 0b11 && currentState == 0b10) || 
        (prevState == 0b10 && currentState == 0b00)) {
        pos2++; // Forward direction
    } else if ((prevState == 0b00 && currentState == 0b10) || 
               (prevState == 0b10 && currentState == 0b11) || 
               (prevState == 0b11 && currentState == 0b01) || 
               (prevState == 0b01 && currentState == 0b00)) {
        pos2--; // Reverse direction
    }

    // Update previous state
    prevState = currentState;
}


bool TimerHandler(struct repeating_timer *t)
{
  //digitalWrite(control_checkpin,1);

  pid_calculation();

  pid_calculation2();

  Motor_Drive();

  Motor_Drive2();

  //digitalWrite(control_checkpin,0);

  return true;
}



void pid_calculation(){
  target=map(target_angle1,0,360,0,66000); //maps new target from degrees to pulses
  e=pos-target;// calculate error

  dedt=e-eprev;// derivative of error
  //eint=eint+e*deltaT; // integral of error (Taylor)
  eprev=e;

  u=kp*e+kd*dedt; //+ki*eint;//control signal
}

void pid_calculation2(){
  target2=map(target_angle2,0,360,0,66000); //maps new target from degrees to pulses
  e2=pos2-target2;// calculate error

  dedt2=e2-eprev2;// derivative of error
  //eint=eint+e*deltaT; // integral of error (Taylor)
  eprev2=e2;

  u2=kp2*e2+kd2*dedt2; //+ki*eint;//control signal
}

void Motor_Drive(){
  float pwmvalue=(float)fabs(u);
  if(pwmvalue>51){
    pwmvalue=51;
  }

  duty_cycle=(float)(pwmvalue/255)*100;
  if(u>0){
    digitalWrite(pin1,LOW);
  } 
  else if(u<0){
    digitalWrite(pin1,HIGH);
  }else{
    digitalWrite(pin1,LOW);
    duty_cycle=0.0f;
  }

  PWM_Instance->setPWM(pwm_pin, freq, duty_cycle);
}

void Motor_Drive2(){
  float pwmvalue=(float)fabs(u2);
  if(pwmvalue>51){
    pwmvalue=51;
  }

  duty_cycle2=(float)(pwmvalue/255)*100;
  if(u2>0){
    digitalWrite(pin2,LOW);
  } 
  else if(u2<0){
    digitalWrite(pin2,HIGH);
  }else{
    digitalWrite(pin2,LOW);
    duty_cycle2=0.0f;
  }

  PWM_Instance2->setPWM(pwm_pin2, freq, duty_cycle2);
}

void write_buffer(){
  while (Serial.available()>0) {
    char c =Serial.read();
    
    switch(c)
    {

      case 'm':
      menu();
      len=0;
      break;

      case '\r':
      case '\n':
      buffer[len]=0;
      two_angles(&target_angle1,&target_angle2);
      Serial.print(target_angle1);
      Serial.print(" ");
      Serial.println(target_angle2);
      len=0;
      break;

      default:
      if(len<8)
      buffer[len++]=c;
      break;
    }

   }
}

void two_angles(int* target_angle1_var,int* target_angle2_var){
  char* token=strtok(buffer,",");
  bool first_token=true;
  while(token!=NULL)
  {
    
    if(first_token)
    {
      *target_angle1_var=atoi(token);
      first_token=false;
    }else{
      *target_angle2_var=atoi(token);
    }
    token=strtok(NULL,",");
  }
}


void menu(){
  Serial.println("For kp press p, for kd press d, for ki press i");
  char c;

  while(true)
  {
    c=Serial.read();

    if(c=='\n' ||c=='\r')
    continue;

    switch(c)
    {
      case 'p':
      Serial.println("type new kp value:");
      update_var(&kp);
      Serial.print("new kp value is:");
      Serial.println(kp,5);
      return;

      case 'd':
      Serial.println("type new kd value:");
      update_var(&kd);
      Serial.print("new kd value is:");
      Serial.println(kd,5);
      return;

      case 'i':
      Serial.println("type new ki value:");
      update_var(&ki);
      Serial.print("new ki value is:");
      Serial.println(ki,5);
      return;

      case 'x':
      Serial.println("exit menu:");
      return;
    } 
  } 
  
}


void update_var(double* kp_var){
  String readString="";
  bool changed=false;
  while(true)
  {
    
    if(Serial.available()>0)
    {
      char c=Serial.read();

    if((c=='\n' || c=='\r') && changed)
    {
      *kp_var=readString.toDouble();
      return;
    }else if( isdigit(c)|| c=='.'){
      readString+=c;
      changed=true;
    }
   }
    
  } 
}


