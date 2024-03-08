/********************************************************
 * PID Proportional on measurement Example
 * Setting the PID to use Proportional on measurement will 
 * make the output move more smoothly when the setpoint 
 * is changed.  In addition, it can eliminate overshoot
 * in certain processes like sous-vides.
 ********************************************************/
#include <DueTimer.h>
#include <PID_v1.h>
#define ENC_RESOLUTION          4000          // Encoder resolution 360 x 4
#define PWM_RESOLUTION          4095          // 4095 = 2^12
#define DEADZONE                15            // Motors dead zone
#define MEDIUM_POS_PID          0.0,0.0,0.0
#define START_CHAR              '>'
#define PIDCOMMAND              "pid" 

// PIN connections
const byte LED = 13, 
L_EN = 10, R_EN = 7, 
RPWM = 8, LPWM = 9, 
TRIG = 22, 
L_IS = 12, 
R_IS = 11, 
ENC_A = 2, 
ENC_B = 3;

volatile long encoderPos;
// __________PID - position
volatile double pSetpoint, pInput, pOutput;
volatile double pKp, pKi, pKd;
PID pPID(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
#define pPID_sampleTime 10

void setup()
{
  //initialize the variables we're linked to
  Serial.begin(9600);
  
  // pinMode
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  analogWriteResolution(12);
  // motor
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  digitalWrite(LED, LOW);
  digitalWrite(TRIG, LOW);

  // PID - direction
  pPID.SetSampleTime(pPID_sampleTime);
  pPID.SetTunings(MEDIUM_POS_PID, P_ON_M);
  pPID.SetOutputLimits(-1 * (PWM_RESOLUTION - DEADZONE), PWM_RESOLUTION - DEADZONE);
  pPID.SetIntergralMemory(-1);//50
  pPID.SetTolerance(0);//2
  pPID.SetMode(MANUAL); //turn the PID off
  pSetpoint = 0;
  encoderPos = 0;

  // Timers
  DueTimer positionPID_Timer = Timer.getAvailable();
  positionPID_Timer.attachInterrupt(positionPID_ISR);
  positionPID_Timer.setFrequency(1000.0 / (double)pPID_sampleTime);
  positionPID_Timer.start();

  // Encoder
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(ENC_A, doEncoderA, CHANGE);
  attachInterrupt(ENC_B, doEncoderB, CHANGE);

  //turn the PID on
  pPID.SetMode(AUTOMATIC);
}

void loop()
{
  // ReadSerial();
  fixPositionPID();
  // Serial.print("encoderPos= ");
  // Serial.print(encoderPos);
  // Serial.println("");
  delay(100);
}

void fixPositionPID()
{
  double a, b, c, d;
  if (Serial.available() > 0)
  {
    if (Serial.read() != START_CHAR)return;
    a = Serial.parseFloat();
    b = Serial.parseFloat();
    c = Serial.parseFloat();
    d = Serial.parseFloat();
    pPID.SetTunings(a, b, c);
    pSetpoint = d < 0 ? -1 * d + floor(encoderPos / ENC_RESOLUTION) * ENC_RESOLUTION : d + floor(encoderPos / ENC_RESOLUTION) * ENC_RESOLUTION ;
    Serial.print("kp="); Serial.print(a); Serial.print("  ki="); Serial.print(b); Serial.print("  kd="); Serial.print(c); Serial.print("  setpnt="); Serial.println(d);
  }
  Serial.print("pSetpoint= "); Serial.print(pSetpoint); Serial.print("   actualPos= "); Serial.print(pInput); Serial.print("    pOutput= "); Serial.println(pOutput);
  delay(10);
}

void positionPID_ISR()
{
  pInput = encoderPos;
  bool active = pPID.Compute();

  if (active)
  {
    if (pOutput > 0)
    {
      analogWrite(LPWM, 0);
      analogWrite(RPWM, abs(pOutput) + DEADZONE);
    } else if (pOutput == 0)
    {
      analogWrite(LPWM, 0);
      analogWrite(RPWM, 0);
    } else
    {
      analogWrite(LPWM, abs(pOutput) + DEADZONE);
      analogWrite(RPWM, 0);
    }
  }
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(ENC_A) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENC_B) == LOW) {encoderPos = encoderPos + 1;         /* CW */}
    else {encoderPos = encoderPos - 1;         /* CCW */}
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENC_B) == HIGH) {encoderPos = encoderPos + 1;          /* CW */}
    else {encoderPos = encoderPos - 1;          /* CCW */}
  }
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(ENC_B) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(ENC_A) == HIGH) {encoderPos = encoderPos + 1;         /* CW */}
    else {encoderPos = encoderPos - 1;         /* CCW */}
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENC_A) == LOW) {encoderPos = encoderPos + 1;          /* CW */}
    else {encoderPos = encoderPos - 1;          /* CCW */}
  }
}

bool ReadSerial()
{
  if (Serial.available() > 0)
  {
    char tempChar = Serial.read();
    if (tempChar != START_CHAR) return false;
    String tempRead;
    String mode = Serial.readStringUntil(',');
    if (tempRead == PIDCOMMAND)
    {
      pPID.SetTunings(Serial.parseFloat(), Serial.parseFloat(), Serial.parseFloat(), P_ON_M); 
    }
  }
}
