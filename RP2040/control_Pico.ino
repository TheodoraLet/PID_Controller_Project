#include <RP2040_PWM.h>

int pwm_pin=16 ;// pins 
int pin1=20;// used
float freq=25000.0f;
float duty_cycle=0.0f;

int enc1=17;// pins used
int enc2=18;// for encoder

double kp=0.01;
double kd=0.15;//0.5;//1.5;
double ki=0;
double u=0; // pid control signal
double deltaT;

int pos=0; // variable that keeps position of motor
int prev_pos=0;
int currmillis=0; // for calculating
int prevmillis=0; // deltaT
int e=0; // error
int eprev=0;
int eint=0;
double dedt=0;
int target=0; // target angle in pulses
volatile int target_angle[2]; // 0 for old angle , 1 for new angle



RP2040_PWM* PWM_Instance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pwm_pin,OUTPUT);
  pinMode(pin1,OUTPUT);
  pinMode(enc1,INPUT);
  pinMode(enc2,INPUT);
  attachInterrupt(digitalPinToInterrupt(enc1),isrEncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2),isrEncoder,CHANGE);

  PWM_Instance= new RP2040_PWM(pwm_pin,freq,duty_cycle);
  if(PWM_Instance)
  {
    PWM_Instance->setPWM();
  }

  while(!Serial){
// waits until Serial monitor is connected
  }

}

void loop() {
  // put your main code here, to run repeatedly:
write();

pid_calculation();

Motor_Drive();

Serial.print("position:");
Serial.print(pos);
Serial.print(" ");
Serial.print("target:");
Serial.print(target);
Serial.print(" ");
Serial.print("u:");
Serial.print(u);
Serial.print(" ");
Serial.print("error:");
Serial.print(e);
Serial.print(" ");
Serial.print("duty cycle:");
Serial.println(duty_cycle);

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


// UART function to set the new target angle
 void write(){
while(Serial.available()>0){
  static uint  num=0;
  target_angle[0]=target_angle[1];
  uint new_key=Serial.parseInt();
  if(new_key!=0 ){
    num=num*10+new_key;
  }
  if(new_key==0){
    Serial.println(num);
    target_angle[1]=num;
    num=0;
  }
  }
}

// pid control signal calculation 
void pid_calculation(){
  target=map(target_angle[1],0,360,0,66000); //maps new target from degrees to pulses
  e=pos-target;// calculate error

  dedt=e-eprev;// derivative of error
  //eint=eint+e*deltaT; // integral of error (Taylor)
  eprev=e;

  u=kp*e+kd*dedt; //+ki*eint;//control signal
}


// function to set the pwm value and the direction of the motor 
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


