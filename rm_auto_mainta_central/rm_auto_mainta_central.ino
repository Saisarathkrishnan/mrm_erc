#include <HardwareSerial.h>
#include "arm_IK.h"
#include "driveM.h"
#include "driveA.h"

//---ESP32_PIN_CONFIG---
// Arm
#define pwm_swivel 13
#define dir_swivel 14 //J7


#define pwm_link1 17


#define dir_link1 16//J1

#define pwm_link2 19//J2
#define dir_link2 18


// Drive
 #define Rdir 26//j6
 #define Ldir 27//J6
 #define Rpwm 33 // j5
 #define Lpwm 25 // j5

// SERIAL
HardwareSerial SerialPort(1);  // use UART1
HardwareSerial Sender(0);      // default serial (UART0)

const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Pins
const int pwm_pin[] = { pwm_swivel, pwm_link1, pwm_link2, Rpwm, Lpwm };
const int dir_pin[] = { dir_swivel, dir_link1, dir_link2, Rdir, Ldir };

long prevT = 0;
int changeMode = 0;

// Arm variables
float linkLength1 = 39.8, linkLength2 = 37.5;
int L = 0, R = 0, T = 0, U = 0, G = 0, S=0;
int freq = 8000, Lchannel = 0, Rchannel = 1, resolution = 8;

// Class instances
IK ik;
DriveM driveM;
DriveA driveA;

const int freqLinks = 8000;      // 5 kHz frequency
const int ledChannelLink1 = 15;  // PWM channel (0-15)
const int resolutionLink1 = 8;

const int ledChannelLink2 = 14;  // PWM channel (0-15)
const int resolutionLink2 = 8;

const int ledChannelSwivel = 12;  // PWM channel (0-15)
const int resolutionSwivel = 8;

const int left_side_channel= 13;  // PWM channel (0-15)
const int left_side_resolution = 8;

const int right_side_channel= 11;  // PWM channel (0-15)
const int right_side_resolution = 8;

void gripper(int grip)
{
  switch (grip)
  {
    case 0: 
      Sender.write('Q');
         //  Serial.println("0");
      break;

    case 1:
      Sender.write('W');
      //      Serial.println("1");
      break;

    case 2: 
      Sender.write('E');
      //      Serial.println("2");
      break;

    case 3: 
      Sender.write('R');
      //      Serial.println("3");
      break;

    case 4: 
      Sender.write('T');
      //      Serial.println("4");
      break;

    case 5: 
      Sender.write('Y');
      //      Serial.println("5");
      break;

    case 6: 
      Sender.write('U');
      //      Serial.println("6");
      break;

    case 7: 
      Sender.write('I');
      //      Serial.println("7");
      break;

    case 8: 
      Sender.write('V');
      //      Serial.println("7");
      break;
    case 9: 
      Sender.write('B');
      //      Serial.println("7");
      break;
    default:
     Sender.write('Q');
        //   Serial.println("7");
     break;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  //Sender.begin(115200, SERIAL_8N1, 1, 3);
  Sender.begin(115200, SERIAL_8N1, -1, 1) ;
  SerialPort.begin(115200, SERIAL_8N1, 21, -1);

  pinMode(dir_link2, OUTPUT);
  ledcSetup(ledChannelLink1, freqLinks, resolutionLink1);
  ledcAttachPin(pwm_link2, ledChannelLink1);
  

  pinMode(dir_link1, OUTPUT);
  ledcSetup(ledChannelLink2, freqLinks, resolutionLink2);
  ledcAttachPin(pwm_link1, ledChannelLink2);

  pinMode(dir_swivel, OUTPUT);
  ledcSetup(ledChannelSwivel, freqLinks, resolutionSwivel);
  ledcAttachPin(pwm_swivel, ledChannelSwivel);

  pinMode(Ldir, OUTPUT);
  ledcSetup(left_side_channel, freqLinks, left_side_resolution);
  ledcAttachPin(Lpwm, left_side_channel);
  pinMode(Rdir, OUTPUT);
  ledcSetup(right_side_channel, freqLinks, right_side_resolution);
  ledcAttachPin(Rpwm, right_side_channel);

  Serial.println("Pins set!");
  delay(100);

  //ik.set_link_length(linkLength1, linkLength2);
  //ik.set_pid();
  Serial.println("PID set");
  delay(10);
}

void loop() {
  if (SerialPort.available()) {
    // Read incoming data
    while (SerialPort.available()) {
      rxBuffer[bufferIndex++] = (char)SerialPort.read();
      if (bufferIndex >= BUFFER_SIZE) bufferIndex = 0;
    }
   Serial.println(rxBuffer); 
    // Parse packet markers
    char *M_index = strchr(rxBuffer, 'M');
    char *L_index = strchr(rxBuffer, 'L');
    char *R_index = strchr(rxBuffer, 'R');
    char *T_index = strchr(rxBuffer, 'T');
    char *U_index = strchr(rxBuffer, 'U');
    char *S_index = strchr(rxBuffer, 'S');
    char *G_index = strchr(rxBuffer, 'G');
    char *Z_index = strchr(rxBuffer, 'Z');
    char *E_index = strchr(rxBuffer, 'E');

    // Check if packet has 'Z' == '0' (maybe you want to check value rather than pointer)
    if (Z_index != nullptr && *(Z_index + 1) == '0') {

      if (L_index) {
        L = atoi(L_index + 1);
      }
      if (R_index) {
        R = atoi(R_index + 1);
      }
      if (T_index) {
        T = atoi(T_index + 1);
      }
      if (U_index) {
        U = atoi(U_index + 1);
      }if(G_index) {
        G=atoi(G_index+1);
      }
      if(S_index){
        S=atoi(S_index+1);
      }

      ledcWrite(right_side_channel, abs(L) * 2.55);
      ledcWrite(left_side_channel, abs(R) * 2.55);

      if (L > 0) {
        digitalWrite(Ldir, LOW);
      } else if (L < 0) {
        digitalWrite(Ldir, HIGH);
      } else digitalWrite(Ldir, LOW);

      if (R > 0) digitalWrite(Rdir, LOW);
      else if (R < 0) digitalWrite(Rdir, HIGH);
      else digitalWrite(Rdir, LOW);


      // link1
      if (T > 0) {
        int xv=T;
        digitalWrite(dir_link2, HIGH);  //link2
        ledcWrite(ledChannelLink1, xv);
      } else if (T < 0) {
        int xv=-T;
        digitalWrite(dir_link2, LOW);
        ledcWrite(ledChannelLink1, xv);
      } else {
        digitalWrite(dir_link2, LOW);
        ledcWrite(ledChannelLink1, 0);
      }

      if (U > 0) {  
        int xv=U;
        digitalWrite(dir_link1, HIGH);  //link1
        ledcWrite(ledChannelLink2, xv);
      } else if (U < 0) {
        int xv=-U;
        digitalWrite(dir_link1, LOW);
        ledcWrite(ledChannelLink2, xv);
      } else {
        digitalWrite(dir_link1, LOW);
        ledcWrite(ledChannelLink2, 0);
      }

      if (S > 0) {
        int xv= S;
        digitalWrite(dir_swivel, HIGH);  //swivel
        ledcWrite(ledChannelSwivel, xv);
      } else if (S < 0) {
        int xv= -S;
        digitalWrite(dir_swivel, LOW);
        ledcWrite(ledChannelSwivel, xv);
      } else {
        digitalWrite(dir_swivel, LOW);
        ledcWrite(ledChannelSwivel, 0);
      }
      
      if (G == 0) {
        //gripper nothing
        gripper(0);
      } else if (G ==1) {
        Serial.println("PID set");
        gripper(4);
      } else if (G ==2)  {
        gripper(3);
        //gripper pitch down
      }else if (G ==3)  {
        gripper(2);
        //gripper roll left 
      }else if (G==4){
        //roll right
        gripper(1);
      }else if (G==5){
        //gripper open
        gripper(5);
      }else if (G==6){
        //gripper cloase
        gripper(6);
      }
      
    }

    bufferIndex = 0;
  }
}