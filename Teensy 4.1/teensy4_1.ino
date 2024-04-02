#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
  
#include <SPI.h> 
#include <SD.h> 

#define fileName  "test.txt"

int pwm_pin= 33;// pins 
int pin1=34;// used
int pin2= 35;// for motor

int enc1=31;// pins used
int enc2=32;// for encoder

volatile uint kp=1.5;
volatile uint kd=0;
volatile uint ki=0;
int u=0; // pid control signal
volatile uint deltaT;

int pos=0; // variable that keeps position of motor
int prev_pos=0;
int currmillis=0; // for calculating
int prevmillis=0; // deltaT
int e=0; // error
int eprev=0;
int pwmvalue=0;
int eint=0;
int dedt=0;
int target=0; // target angle in pulses
volatile int target_angle[2]; // 0 for old angle , 1 for new angle
int prevtarget=0; 
int pos_angle=0;


char target_s[6]; // char arrays for SD card writing 
char pos_s[6];
char u_s[8];
char target_ang_s[4];
char time_s[4];
char error_s[5];
char prev_targ_s[6];
int chip_sel=BUILTIN_SDCARD; // chip select pin for SD card writing
int lastSerialPlot=0;
int interval =1000;
Encoder myEnc(enc1,enc2); // declare encoder pins

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_pin,OUTPUT);
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
  pinMode(enc1,INPUT);
  pinMode(enc2,INPUT);
  attachInterrupt(digitalPinToInterrupt(enc1),readEncoder,CHANGE);
Serial.begin(9600);
 if(!SD.begin(chip_sel)){ 
while(true); // Initialization of SD card
}
while(!Serial){
// waits until Serial monitor is connected
}
Menu(); // executes Menu function 

}

void loop() {
  // put your main code here, to run repeatedly:
currmillis=millis();
int deltaT=(currmillis-prevmillis)/1.0e6;
prevmillis=currmillis;

write();
// pos=myEnc.read();
pid_calculation();

Motor_Drive();

// if(currmillis-lastSerialPlot>interval){
// Serial.print("Elapsed time:");
// Serial.print(currmillis/1000);
// Serial.print(",");
// Serial.print("position pulses:");
// Serial.print(pos);
// Serial.print(" ");
// Serial.print("position:");
// Serial.print(pos_angle);
// Serial.print(" ");
// Serial.print("prevtarget:");
// Serial.print(prevtarget);
// Serial.print(" ");
// Serial.print("prev_angle:");
// Serial.print(target_angle[0]);
// Serial.print(" ");
// Serial.print("Target:");
// Serial.print(target);
// Serial.print(" ");
// Serial.print("t_angle:");
// Serial.print(target_angle[1]);
// Serial.print(" ");
// Serial.print("u:");
// Serial.print(u);
// Serial.print(" ");
// Serial.print("error:");
// Serial.println(pos_angle-target_angle[1]);
// lastSerialPlot=currmillis;
// }

SD_write();

}

void readEncoder(){
pos=myEnc.read();
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
pos_angle=map(pos,0,81600,0,360);// maps encoder pulses to angle in degrees for plotting
prevtarget=map(target_angle[0],0,360,0,81600);//maps previous target from degrees to pulses
target=map(target_angle[1],0,360,0,81600); //maps new target from degrees to pulses
e=pos-target;// calculate error

dedt=(e-eprev)/deltaT;// derivative of error
eint=eint+e*deltaT; // integral of error (Taylor)
eprev=e;

u=kp*e +kd*dedt +ki*eint;//control signal 
}


// function to set the pwm value and the direction of the motor 
void Motor_Drive(){
int pwmvalue=(int)fabs(u);
if(pwmvalue>255){
  pwmvalue=255;
}
if(u<0){
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,HIGH);
} 
else if(u>0){
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
}else{
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,LOW);
  pwmvalue=0;
}

analogWrite(pwm_pin,pwmvalue);
}

// writes the SD card after doing the proper convertions from integers to char arrays
void SD_write(){
File datafile =SD.open("test.txt",FILE_WRITE);
sprintf(target_s,"%d",target);
sprintf(pos_s,"%d",pos);
sprintf(u_s,"%d",u);
sprintf(error_s,"%d",pos_angle-target_angle[1]);
sprintf(target_ang_s,"%d",target_angle[1]);
sprintf(prev_targ_s,"%d",prevtarget);
sprintf(time_s,"%d",currmillis/1000);
  if (datafile){
datafile.print("Elapsed time:");
datafile.print(time_s);
datafile.print(",");
datafile.print("position:");
datafile.print(pos_s);
datafile.print(" ");
datafile.print("Target:");
datafile.print(target_s);
datafile.print(" ");
datafile.print("u:");
datafile.print(u_s);
datafile.print(" ");
datafile.print("t_angle:");
datafile.print(target_angle[1]);
datafile.print(" ");
datafile.print("error:");
datafile.print(error_s);
datafile.print(" ");
datafile.print("prevtarget:");
datafile.println(prev_targ_s);
datafile.close(); // always close the file after each write
  }
}

// function to write the new control values in Menu 
uint write2(uint val){
while(Serial.available()>0){
  static uint  num=0;
  uint new_key=Serial.parseInt();
  if(new_key!=0 ){
    num=num*10+new_key;
  }
  if(new_key==0){
    Serial.println(num);
    val=num;
    num=0;
    return val;
  }

  }
  return 0;
}

// executes once in setup to change the controller's gains 
void Menu(){
Serial.println("Menu choices, press :");
Serial.println("1 for kp");
Serial.println("2 for ki");
Serial.println("3 for kd");
Serial.println("4 to exit menu");
while(!Serial.available()){ }
for(;;){
uint menu_choice=Serial.parseInt();

  switch(menu_choice){
  case 1: 
  Serial.println("Enter new kp value:");
  kp=write2(kp);
  Serial.print("New value of kp is :");
  Serial.println(kp);
   break; 

  case 2:
  Serial.println("Enter new ki value:");
  ki=write2(ki);
  Serial.print("New value of ki is :");
  Serial.println(ki);
  break;

  case 3:
  Serial.println("Enter new kd value:");
  kd=write2(kd);
  Serial.print("New value of kd is :");
  Serial.println(kd);
  break;
  
   case 4:
  Serial.println("Leaving menu");
  return;

  }
}

}



