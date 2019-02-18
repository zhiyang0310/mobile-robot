/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

/* Run the PID loop at 30 times per second */
#define PID_RATE           100     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;
  
/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 1000
long lastMotorCommand = AUTO_STOP_INTERVAL;

long LEFT_TICKS = 0L;
long RIGHT_TICKS = 0L;

unsigned char reverse = 0;

#define LEFT_ENC_PIN_A 2  //pin 2
#define LEFT_ENC_PIN_B 20  //pin 20
#define RIGHT_ENC_PIN_A 3  //pin 3
#define RIGHT_ENC_PIN_B 21  //pin 21
#define RIGHT_MOTOR_BACKWARD 10
#define LEFT_MOTOR_BACKWARD  9
#define RIGHT_MOTOR_FORWARD  11
#define LEFT_MOTOR_FORWARD   8


#include "commands.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "PID.h"

// A pair of varibles to help parse serial commands
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
////////////////////////////////////////////////////////
/* process commands */
void runCommand(){
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd){
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case UPDATE_PID_L:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
      pid_args[i] = atoi(str);
      i++;
    }
    leftPID.Kp = pid_args[0];
    leftPID.Kd = pid_args[1];
    leftPID.Ki = pid_args[2];
    leftPID.Ko = pid_args[3];
    Serial.println("OK");
    break;
  case UPDATE_PID_R:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
      pid_args[i] = atoi(str);
      i++;
    }
    rightPID.Kp = pid_args[0];
    rightPID.Kd = pid_args[1];
    rightPID.Ki = pid_args[2];
    rightPID.Ko = pid_args[3];
    Serial.println("OK");
    break;
  case READ_PID:
    if(argv1[0] == 'l'){
      Serial.print(leftPID.Kp);
      Serial.print(" ");
      Serial.print(leftPID.Kd);
      Serial.print(" ");
      Serial.print(leftPID.Ki);
      Serial.print(" ");
      Serial.print(leftPID.Ko);
    }
    else if(argv1[0] == 'r'){
      Serial.print(rightPID.Kp);
      Serial.print(" ");
      Serial.print(rightPID.Kd);
      Serial.print(" ");
      Serial.print(rightPID.Ki);
      Serial.print(" ");
      Serial.print(rightPID.Ko);
    }
    break;
  }
}



void setup() {
  leftPID.Kp = 7;
  leftPID.Kd = 110;
  leftPID.Ki = 0;
  leftPID.Ko = 200;

  rightPID.Kp = 7;
  rightPID.Kd = 45;
  rightPID.Ki = 0;
  rightPID.Ko = 200;
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A),leftCounter_A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A),rightCounter_A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B),leftCounter_B,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B),rightCounter_B,CHANGE);

  pinMode(RIGHT_MOTOR_BACKWARD,OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD,OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD,OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD,OUTPUT);

  Serial.begin(BAUDRATE);
}

void loop() {
  /* receive command from raspberrypi */
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  /* finish receive command */
  // PID control
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}
