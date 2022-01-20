/**
 * Author:  O'Connor Chris
 * Date:    27.10.2021
 * Version: 3.1
 */

#include <Wire.h>

volatile bool PCI_channel_state[4] = {false, false, false, false};
volatile int  PWM_channel_input[4];
volatile long PCI_current_time, PCI_channel_timer[4];

unsigned long ESC_loop_time, current_time, ESC_PWM_target_time[4];

int ESC_PWM[4], gyro_axis_cal[3], gyro_axis[3], acc_axis[3], temp, flight_state;

float gyro_pitch_angle, pitch_offset, acc_pitch_angle,
      gyro_roll_angle , roll_offset , acc_roll_angle,
      gyro_yaw_angle  , yaw_offset, gravity_vector;

float pid_p_pitch, pid_i_pitch, pid_d_pitch, pid_d_pitch_last_error, pid_desired_pitch_angle, pid_pitch, pid_error;
float pid_p_roll , pid_i_roll , pid_d_roll , pid_d_roll_last_error , pid_desired_roll_angle , pid_roll;
float pid_p_yaw  , pid_i_yaw  , pid_d_yaw  , pid_d_yaw_last_error  , pid_desired_yaw_angle  , pid_yaw;

bool setup_complete = false;

const int PWM_MAX    = 2000;
const int PWM_MIN    = 1100;
const int PWM_OFF    = 1000;

/**
 * PID Settings
 */
const float PID_KP  = 0.0;
const float PID_KI  = 0.0;
const float PID_KD  = 3.0;
const int   PID_MAX = 200;


void setup() {

  Wire.begin();
  TWBR = 12;
  
  DDRB |= B00100000;  // Output Digital port 13
  DDRD |= B11110000;  // Output Digital port 4, 5, 6 & 7

  digitalWrite(12, HIGH);

  init_mpu_6050();

  gyro_cal();

  /**
   * Setup PinChangeInterrupt
   */
  PCICR  |= 0b00000001;     // PCIE0 for PCMSK0
  PCMSK0 |= 0b00001111;     // PCINT 0:3 => Input 8:11

  digitalWrite(13, HIGH);

  /**
   * Wait to exit Setup until Receiver Channel 3 (Throttle) is in lowest position
   */
  while(PWM_channel_input[2] < 1000 || PWM_channel_input[2] > 1020) {
    PORTD |= B11110000;           // PORTD HIGH
    delayMicroseconds(1000);
    PORTD &= B00001111;           // PORTD LOW
    delayMicroseconds(3000);
  }
  
  setup_complete = true;
  flight_state = 0;
  current_time = micros();

  digitalWrite(13, LOW);

}

void loop() {

  read_mpu_6050_data();
  
  offset_calc();

  flight_state_update();

  pry_calc();

  pid_calc();
  
  esc_pwm_calc();

  /**
   * Set looptime to 250Hz (4ms)
   */
  while(micros() < current_time + 4000);
  current_time = micros();

  PORTD |= B11110000; // SET HIGH on all ESCs
  
  // Calculate target time of each ESC
  ESC_PWM_target_time[0] = ESC_PWM[0] + current_time;
  ESC_PWM_target_time[1] = ESC_PWM[1] + current_time;
  ESC_PWM_target_time[2] = ESC_PWM[2] + current_time;
  ESC_PWM_target_time[3] = ESC_PWM[3] + current_time;
  
  while(PORTD >= 16) {
    ESC_loop_time = micros();
    if(ESC_loop_time >= ESC_PWM_target_time[0]) PORTD &= B11101111;
    if(ESC_loop_time >= ESC_PWM_target_time[1]) PORTD &= B11011111;
    if(ESC_loop_time >= ESC_PWM_target_time[2]) PORTD &= B10111111;
    if(ESC_loop_time >= ESC_PWM_target_time[3]) PORTD &= B01111111;
  }

}

/**
 * Calculate Motorspeeds
 */
void esc_pwm_calc() {
  
  if(flight_state == 2) {
    
    ESC_PWM[0] = PWM_channel_input[2] - pid_pitch + pid_roll - pid_yaw;
    ESC_PWM[1] = PWM_channel_input[2] + pid_pitch + pid_roll + pid_yaw;
    ESC_PWM[2] = PWM_channel_input[2] + pid_pitch - pid_roll - pid_yaw;
    ESC_PWM[3] = PWM_channel_input[2] - pid_pitch - pid_roll + pid_yaw;

    /** 
     *  PWM MIN Failsafe
     */
    if(ESC_PWM[0] < PWM_MIN) ESC_PWM[0] = PWM_MIN;
    if(ESC_PWM[1] < PWM_MIN) ESC_PWM[1] = PWM_MIN;
    if(ESC_PWM[2] < PWM_MIN) ESC_PWM[2] = PWM_MIN;
    if(ESC_PWM[3] < PWM_MIN) ESC_PWM[3] = PWM_MIN;
    
    /**
     *  PWM MAX Failsafe
     */
    if(ESC_PWM[0] > PWM_MAX) ESC_PWM[0] = PWM_MAX;
    if(ESC_PWM[1] > PWM_MAX) ESC_PWM[1] = PWM_MAX;
    if(ESC_PWM[2] > PWM_MAX) ESC_PWM[2] = PWM_MAX;
    if(ESC_PWM[3] > PWM_MAX) ESC_PWM[3] = PWM_MAX;
    
  } else {
    /**
     * Turn Motors off
     */
    ESC_PWM[0] = 1000;
    ESC_PWM[1] = 1000;
    ESC_PWM[2] = 1000;
    ESC_PWM[3] = 1000;
  }
}

void pid_calc() {
 
 /** 
  *  Roll
  */
  gyro_roll_angle  = (gyro_roll_angle  * 0.7) + ((gyro_axis[1] / 65.5) * 0.3);
  pid_error        = gyro_roll_angle - pid_desired_roll_angle;

  pid_p_roll  = PID_KP * pid_error;
  pid_i_roll += PID_KI * pid_error;
  pid_d_roll  = PID_KD * (pid_error - pid_d_roll_last_error);
  
  if     (pid_i_roll > PID_MAX)      pid_i_roll = PID_MAX;
  else if(pid_i_roll < PID_MAX * -1) pid_i_roll = PID_MAX * -1;
  
  pid_d_roll_last_error = pid_error;
  
  pid_roll = pid_p_roll + pid_i_roll + pid_d_roll;

  if     (pid_roll > PID_MAX     ) pid_roll = PID_MAX;
  else if(pid_roll < PID_MAX * -1) pid_roll = PID_MAX * -1;


 /**
  *  Pitch
  */
  gyro_pitch_angle = (gyro_pitch_angle * 0.7) + ((gyro_axis[0] / 65.5) * 0.3);
  pid_error        = gyro_pitch_angle - pid_desired_pitch_angle;

  pid_p_pitch  = PID_KP * pid_error;
  pid_i_pitch += PID_KI * pid_error;
  pid_d_pitch  = PID_KD * (pid_error - pid_d_pitch_last_error);
  
  if     (pid_i_pitch > PID_MAX     ) pid_i_pitch = PID_MAX;
  else if(pid_i_pitch < PID_MAX * -1) pid_i_pitch = PID_MAX * -1;

  pid_d_pitch_last_error = pid_error;

  pid_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  
  if     (pid_pitch > PID_MAX)      pid_pitch = PID_MAX;
  else if(pid_pitch < PID_MAX * -1) pid_pitch = PID_MAX * -1;

  
 /**
  *  Yaw
  */
  gyro_yaw_angle   = (gyro_yaw_angle   * 0.7) + ((gyro_axis[2] / 65.5) * 0.3);
  pid_error        = gyro_yaw_angle - pid_desired_yaw_angle;

  pid_p_yaw  = 3    * pid_error;
  pid_i_yaw += 0.02 * pid_error;
  pid_d_yaw  = 0    * (pid_error - pid_d_yaw_last_error);

  if     (pid_i_yaw > PID_MAX     ) pid_i_yaw = PID_MAX;
  else if(pid_i_yaw < PID_MAX * -1) pid_i_yaw = PID_MAX * -1;

  pid_d_yaw_last_error = pid_error;

  pid_yaw = pid_p_yaw + pid_i_yaw + pid_d_yaw; 
  
  if     (pid_yaw > PID_MAX     ) pid_yaw = PID_MAX;
  else if(pid_yaw < PID_MAX * -1) pid_yaw = PID_MAX * -1;
  
}

/**
 * Reset PID values
 */
 
void pid_reset() {
  
    roll_offset  = 0;
    pitch_offset = 0;

    pid_i_roll  = 0;
    pid_i_pitch = 0;
    pid_i_yaw   = 0;

    pid_d_roll_last_error  = 0;
    pid_d_pitch_last_error = 0;
    pid_d_yaw_last_error   = 0;
    
}

void offset_calc() {

  /**
   * Calculate Basic Offsets
   * 0.0000611 = 1 / (250Hz / 65.5)
   * 
   */
  pitch_offset += gyro_axis[0] * 0.0000611;
  roll_offset  += gyro_axis[1] * 0.0000611;
  yaw_offset    = gyro_axis[2] * 0.0000611;

  /**
   * Calculate Offsets with Yaw Offset
   * 0.01745555 = 1 / (3.142(PI) / 180°) ° to Radiant
   */
   
  pitch_offset -= roll_offset  * sin(yaw_offset * 0.01745555);
  roll_offset  += pitch_offset * sin(yaw_offset * 0.01745555);
  
  /**
   * Accelerometer calculations
   * 1 rad = 57.296°
   */
  gravity_vector  = sqrt((acc_axis[0]*acc_axis[0])+(acc_axis[1]*acc_axis[1])+(acc_axis[2]*acc_axis[2]));
  if(abs(acc_axis[1]) < gravity_vector) acc_pitch_angle = asin((float) acc_axis[1] / gravity_vector) *  57.296;
  if(abs(acc_axis[0]) < gravity_vector) acc_roll_angle  = asin((float) acc_axis[0] / gravity_vector) * -57.296;

  /**
   * Complementary Filter Gyrodrift
   */
  pitch_offset = pitch_offset * 0.9996 + acc_pitch_angle * 0.0004;
  roll_offset  = roll_offset  * 0.9996 + acc_roll_angle  * 0.0004;
  
}

/**
 * Manipulate Offsets for Movement
 */
void pry_calc() {
  
  if(PWM_channel_input[0] < 1492 || PWM_channel_input[0] > 1508) {
    pid_desired_roll_angle   = map(PWM_channel_input[0], PWM_OFF, PWM_MAX, -100, 100) - roll_offset;
  } else {
    pid_desired_roll_angle   = 0 - roll_offset;
  }

  if(PWM_channel_input[1] < 1492 || PWM_channel_input[1] > 1508) {
    pid_desired_pitch_angle  = map(PWM_channel_input[1], PWM_OFF, PWM_MAX, -100, 100) - pitch_offset;
  } else {
    pid_desired_pitch_angle  = 0 - pitch_offset;
  }

  pid_desired_yaw_angle = 0;
  if(PWM_channel_input[2] > 1050) {
    pid_desired_yaw_angle = map(PWM_channel_input[3], PWM_OFF, PWM_MAX, -100, 100);
  }
}

/**
 * Gyro Calibration
 */
void gyro_cal() {

  for (int i = 0; i < 2000 ; i ++) {
    if(i % 200 == 0)digitalWrite(13, !digitalRead(13));
    
    read_mpu_6050_data();
    
    gyro_axis_cal[0] += gyro_axis[0];
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];

    PORTD |= B11110000;           // Port 4, 5, 6 and 7 HIGH
    delayMicroseconds(1000);
    PORTD &= B00001111;           // Port 4, 5, 6 and 7 LOW
    delayMicroseconds(3000);
  }

  /**
   * Calculate Gyro Offset
   */
  gyro_axis_cal[0] /= 2000;
  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;
}

void flight_state_update() {
  /**
   * Turn Off Motors
   */
  if(flight_state == 2 && PWM_channel_input[2] < 1050 && PWM_channel_input[3] > 1950) 
    flight_state = 0;
    
  /**
   * Start Motors
   */
  if(PWM_channel_input[2] < 1050 && PWM_channel_input[3] < 1050) 
    flight_state = 1;

  /**
   * Reset PID
   */
  if(flight_state == 1 && PWM_channel_input[2] < 1050 && PWM_channel_input[3] > 1480) {
    flight_state = 2;
    pid_reset();
  }
}

void read_mpu_6050_data() {

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  
  while(Wire.available() < 14);  

  acc_axis[0]  = Wire.read()<<8|Wire.read();
  acc_axis[1]  = Wire.read()<<8|Wire.read();
  acc_axis[2]  = Wire.read()<<8|Wire.read();
  temp         = Wire.read()<<8|Wire.read();
  gyro_axis[0] = Wire.read()<<8|Wire.read();
  gyro_axis[1] = Wire.read()<<8|Wire.read();
  gyro_axis[2] = Wire.read()<<8|Wire.read();

  if(setup_complete) {
    gyro_axis[0] -= gyro_axis_cal[0];
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
  }

  gyro_axis[2] *= -1;
   acc_axis[2] *= -1;

}

void init_mpu_6050() {

  // WakeUp
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // Gyro Config
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  // Accel Config
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  // DLPF
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  /**
   * 4s delay to connect Battery properly without ESC BEEP
   */
  for (int i = 0; i < 1000; i++) {
    PORTD |= B11110000;            // PORTD HIGH
    delayMicroseconds(1000);
    PORTD &= B00001111;            // PORTD LOW
    delayMicroseconds(3000);
  }

}

/**
 * PinChangeInterrupt
 */
ISR(PCINT0_vect) {
  PCI_current_time = micros();
  
  // CH 1
  if(PINB & B00000001) {                                            // Input 8 HIGH?
    if(!PCI_channel_state[0]) {                                     // Input 8 changed from 0 to 1
      PCI_channel_state[0] = 1;                                     // Remember state
      PCI_channel_timer[0] = PCI_current_time;                      // Start timer
    }
  }
  else if(PCI_channel_state[0]) {                                   // Input 8 !HIGH? && changed from 1 to 0.
    PCI_channel_state[0] = 0;                                       // Remember state
    PWM_channel_input[0] = PCI_current_time - PCI_channel_timer[0]; // Calculate Time (PWM)
  }
  // CH 2
  if(PINB & B00000010) {
    if(!PCI_channel_state[1]) {
      PCI_channel_state[1] = 1;
      PCI_channel_timer[1] = PCI_current_time;
    }
  }
  else if(PCI_channel_state[1]) {
    PCI_channel_state[1] = 0;
    PWM_channel_input[1] = PCI_current_time - PCI_channel_timer[1];
    PWM_channel_input[1] = 1500 - PWM_channel_input[1] + 1500;
  }
  // CH 3
  if(PINB & B00000100) {
    if(!PCI_channel_state[2]) {
      PCI_channel_state[2] = 1;
      PCI_channel_timer[2] = PCI_current_time;
    }
  }
  else if(PCI_channel_state[2]) {
    PCI_channel_state[2] = 0;
    PWM_channel_input[2] = PCI_current_time - PCI_channel_timer[2];
    if(PWM_channel_input[2] > 1800) PWM_channel_input[2] = 1800;
  }
  // CH 4
  if(PINB & B00001000) {
    if(!PCI_channel_state[3]) {
      PCI_channel_state[3] = 1;
      PCI_channel_timer[3] = PCI_current_time;
    }
  }
  else if(PCI_channel_state[3]) {
    PCI_channel_state[3] = 0;
    PWM_channel_input[3] = PCI_current_time - PCI_channel_timer[3];
  }
}
