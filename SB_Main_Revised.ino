
#include <Wire.h>
#include <AFMotor.h>
#include <Arduino.h>

#include "MPU6050_Reg_Def.h"
#include "MPU6050_Data_Process.h"

//definition for using data of x or y or z axis
//#define sensor_accel accel_z
#define angle_xyz last_x_angle
#define pre_angle_xyz pre_angle_x

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// MPU-6050 Accelerometer + Gyro
// -----------------------------
//
// By arduino.cc user "Krodal".
// June 2012
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version, 
// since Wire.endTransmission() uses a parameter 
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-6000 and MPU-6050 Product Specification",
//     PS-MPU-6000A.pdf
//   - "MPU-6000 and MPU-6050 Register Map and Descriptions",
//     RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
//   - "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide"
//     AN-MPU-6000EVB.pdf
// 
// The accuracy is 16-bits.
//
// Temperature sensor from -40 to +85 degrees Celsius
//   340 per degrees, -512 at 35 degrees.
//
// At power-up, all registers are zero, except these two:
//      Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
//      Register 0x75 (WHO_AM_I)   = 0x68.
// 






/*global variable definition
*/
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
float pre_angle_x = 0;// store the previous angle of the x axis angle
float pre_angle_y = 0;
float pre_angle_z = 0;

float sensor_x_dot_dot = 0;
float pre_sensor_x_dot_dot = 0;

float x_dot = 0;

/*constant variable definition
*/
const float wheel_r=0.03225;
const float motor_max_power = 3.75;
const int loop_delay=100;


//=====================================================================================================
//=====================================================================================================
// coefficient of P, I & D. 
double Kp=50;
double Ki=3.2;
double Kd=0;
/*
*Ki>3.179;
*Kd>-0.0719;
*Kp>46.4748+(1/1.461)*(Ki-3.1791)/(Kd+0.0719);
*/

//=====================================================================================================
//=====================================================================================================
  
float P=0, I=0, D=0;
// E is used to calculate P+I+D
float E;
float Isum=0;


void setup(){
	//Serial.begin(9600);          
	Serial.println("Motor test!");
	// turn on motor
	motor1.setSpeed(0);
	motor2.setSpeed(0);
	motor1.run(RELEASE);
	motor2.run(RELEASE);
	
	int error;
	uint8_t c;
	
	
	Serial.begin(19200);
	
	Wire.begin();
	
	error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
	MPU6050_ERROR_MSG (error);
	// According to the datasheet, the 'sleep' bit
	// should read a '1'. But I read a '0'.
	// That bit has to be cleared, since the sensor
	// is in sleep mode at power-up. Even if the
	// bit reads '0'.
	error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
	MPU6050_ERROR_MSG (error);
	// Clear the 'sleep' bit to start the sensor.
	MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
	
	//Initialize the angles
	calibrate_sensors();  
	set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}

  
void loop(){
	
	float dt=Read_angle_Refresh_gyro_data();
	
	float angle;
	float pre_angle;
	angle=angle_xyz;
	pre_angle=pre_angle_xyz;
	
	float x_dot_seg = (pre_sensor_x_dot_dot+sensor_x_dot_dot)*dt/2;
	x_dot += x_dot_seg;
	
	boolean Forward;
	P = angle;
	I = (angle+pre_angle)*dt/2;
	D = (angle-pre_angle)/dt;
	Isum+=I;
	E = Kp*P+Ki*Isum+Kd*D;
	  
	float wheel_x_dot = x_dot + D*wheel_r*cos(angle*3.14159/180);
	
	if (E<0)
		Forward=true;
	else 
		Forward=false;
	//===================================================================================================
	//=================================================================================================== 
	
	float power_input=E*wheel_x_dot;
	//Serial.print(sensor_x_dot_dot);
        //Serial.print("   ");
	float set_speed = abs(power_input/motor_max_power*255);
	
        Serial.print("Ki*I= ");
        Serial.print(Ki*Isum);
        Serial.print("   ");
        
        
	if (set_speed>=255)
		set_speed=255;
	
        Serial.print("angle= ");
	Serial.print(angle);
	Serial.print("  ");
	//Serial.println((float)(millis())/1000);
	
	
	motor1.setSpeed(set_speed);
	motor2.setSpeed(set_speed);
        Serial.print("set_speed= ");
        Serial.println(set_speed);
        Serial.print("    ");
	//===================================================================================================
	//===================================================================================================  
	if (Forward){
		//this direction may be the reverse. 
		motor1.run(FORWARD);
		motor2.run(FORWARD);
	}
	else {
		motor1.run(BACKWARD);
		motor2.run(BACKWARD);
	}
	
	delay(loop_delay);
}






//***************************************************************************************************
float Read_angle_Refresh_gyro_data(){
  
	int error;
	accel_t_gyro_union accel_t_gyro;
	
	
	// Read the raw values.
	error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
	MPU6050_ERROR_MSG(error);
	
	// Get the time of reading for rotation computations
	unsigned long t_now = millis();
	
	// Convert gyro values to degrees/sec
	float FS_SEL = 131;
	
	float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
	float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
	float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
	
	
	// Get raw acceleration values
	const float G_CONVERT = 16384;
        const float g=9.81;
	float accel_x = accel_t_gyro.value.x_accel*g/G_CONVERT;
	float accel_y = accel_t_gyro.value.y_accel*g/G_CONVERT;
	float accel_z = accel_t_gyro.value.z_accel*g/G_CONVERT;
        
        /**
	Serial.print("Axyz= ");
        Serial.print(accel_x);
        Serial.print("   ");
        Serial.print(accel_y);
        Serial.print("   ");
        Serial.print(accel_z);
        Serial.print("   ");
        */

	// Get angle values from accelerometer
	float RADIANS_TO_DEGREES = 180/3.14159;
	//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
	float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
	float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;	
	float accel_angle_z = 0;
	
	// Compute the (filtered) gyro angles
	float dt =(t_now - get_last_time())/1000.0;
	float gyro_angle_x = gyro_x*dt + get_last_x_angle();
	float gyro_angle_y = gyro_y*dt + get_last_y_angle();
	float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
	
	// Compute the drifting gyro angles
	float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
	float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
	float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
	
	// Apply the complementary filter to figure out the change in angle - choice of alpha is
	// estimated now.  Alpha depends on the sampling rate...
	float alpha = 0.96;
	float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
	float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
	float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
	
	pre_angle_x = last_x_angle;
	pre_angle_y = last_y_angle;
	pre_angle_z = last_z_angle; 
	// Update the saved data with the latest values
	set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
	
        sensor_x_dot_dot = -accel_z*sin(angle_x*3.14159/180)+accel_y*cos(angle_x*3.14159/180);
        
        Serial.print("accel= ");
        Serial.print(sensor_x_dot_dot);
        Serial.print("   ");
	// Delay so we don't swamp the serial port
	delay(5);
	return dt;
}


