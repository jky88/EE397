#include "Wire.h"
#include “I2Cdev.h” 
#include “MPU6050.h” 
#include “math.h” 
#include <NewPing.h>

#define leftMotorPWMPin 6 
#define leftMotorDirPin 7 
#define rightMotorPWMPin 5 
#define rightMotorDirPin 4

#define TRIGGER_PIN 9 
#define ECHO_PIN 8 

#define MAX_DISTANCE 75

#define Kp 40 
#define Kd 0.05 
#define Ki 40 

#define sampleTime 0.005 
#define targetAngle -2.5

MPU6050 mpu; 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t accY, accZ, gyroX; 
volatile int motorPower, gyroRate; 
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0; 
volatile byte count=0; int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) 
{ 
	if(leftMotorSpeed >= 0) 
	{ 
		analogWrite(leftMotorPWMPin, leftMotorSpeed); 
		digitalWrite(leftMotorDirPin, LOW); 
	} else { 
		analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed); 
		digitalWrite(leftMotorDirPin, HIGH); 
	} 
	if(rightMotorSpeed >= 0) 
	{ 
		analogWrite(rightMotorPWMPin, rightMotorSpeed); 
		digitalWrite(rightMotorDirPin, LOW); 
	} else { 
		analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed); 
		digitalWrite(rightMotorDirPin, HIGH); 
	} 
}

void init_PID() { 
	// disable global interrupts 
	cli(); 
	// set entire TCCR1A register to 0
	TCCR1A = 0; 
	TCCR1B = 0; 
	// set compare match register to set sample time 5ms 
	OCR1A = 9999; 
	// turn on CTC mode 
	TCCR1B |= (1 << WGM12); 
	// Set CS11 bit for prescaling by 8 
	TCCR1B |= (1 << CS11); 
	// enable timer compare interrupt 
	TIMSK1 |= (1 << OCIE1A); 
	// enable global interrupts 
	sei(); 
}

void setup() { 
	// set the motor control and PWM pins to output mode 
	pinMode(leftMotorPWMPin, OUTPUT); 
	pinMode(leftMotorDirPin, OUTPUT); 
	pinMode(rightMotorPWMPin, OUTPUT); 
	pinMode(rightMotorDirPin, OUTPUT); 
	// set the status LED to output mode 
	pinMode(13, OUTPUT); 
	// initialize the MPU6050 and set offset values 
	mpu.initialize(); 
	mpu.setYAccelOffset(1593); 
	mpu.setZAccelOffset(963); 
	mpu.setXGyroOffset(40); 
	// initialize PID sampling loop 
	init_PID(); 
}

void loop() { 
	// read acceleration and gyroscope values 
	accY = mpu.getAccelerationY(); 
	accZ = mpu.getAccelerationZ(); 
	gyroX = mpu.getRotationX(); 
	// set motor power after constraining it 
	motorPower = constrain(motorPower, -255, 255); 
	setMotors(motorPower, motorPower); 
	// measure distance every 100 milliseconds 
	if((count%20) == 0){ 
		distanceCm = sonar.ping_cm(); 
	} 
	if((distanceCm < 20) && (distanceCm != 0)) { 
		setMotors(-motorPower, motorPower); 
	} 
} 

// The ISR will be called every 5 milliseconds 
ISR(TIMER1_COMPA_vect) { 
	// calculate the angle of inclination 
	accAngle = atan2(accY, accZ)*RAD_TO_DEG; 
	gyroRate = map(gyroX, -32768, 32767, -250, 250); 
	gyroAngle = (float)gyroRate*sampleTime; 
	currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle); 
	error = currentAngle – targetAngle; 
	errorSum = errorSum + error; 
	errorSum = constrain(errorSum, -300, 300); 
	//calculate output from P, I and D values 
	motorPower = Kp*(error) + Ki*(errorSum)*sampleTime – Kd*(currentAngle-prevAngle)/sampleTime; 
	prevAngle = currentAngle; 
	// toggle the led on pin13 every second 
	count++; 
	if(count == 200) { 
		count = 0; 
		digitalWrite(13, !digitalRead(13)); 
	} 
}

//1. Set Ki and Kd to zero and gradually increase Kp so that the robot starts to oscillate about the zero position.
//2. Increase Ki so that the response of the robot is faster when it is out of balance.
//   Ki should be large enough so that the angle of inclination does not increase.
//   The robot should come back to zero position if it is inclined.
//3. Increase Kd so as to reduce the oscillations. The overshoots should also be reduced by now.
//4. Repeat the above steps by fine tuning each parameter to achieve the best result.


//PID stands for Proportional, Integral, and Derivative. Each of these terms provides a unique response to our self-balancing robot.
//The proportional term, as its name suggests, generates a response that is proportional to the error. 
//    For our system, the error is the angle of inclination of the robot.
//The integral term generates a response based on the accumulated error. 
//    This is essentially the sum of all the errors multiplied by the sampling period. 
//    This is a response based on the behavior of the system in past.
//The derivative term is proportional to the derivative of the error. 
//    This is the difference between the current error and the previous error divided by the sampling period. 
//    This acts as a predictive term that responds to how the robot might behave in the next sampling loop.
