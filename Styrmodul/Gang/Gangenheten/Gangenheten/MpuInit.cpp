#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "MpuInit.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO


#define BaudRate 9600


volatile uint8_t mpuInterrupt = false; 

ISR (PCINT0_vect)
{
	mpuInterrupt = true;
}

void enable_interrupt0()
{


	/**
	 * Pin Change Interrupt enable on PCINT0 (PA0)
	 */
	PCMSK0 |= _BV(PCINT0);
	PCICR |= _BV(PCIE0);

	sei();
}

float MPU_get_y()
{
	return ypr[0];
}

float MPU_get_p()
{
	return ypr[1];
}

float MPU_get_r()
{
	return ypr[2];
}


void MPU_init(void)
{


	Fastwire::setup(400, true);
	
	//accelgyro.initialize();
	// load and configure the DMP
	
	devStatus = mpu.dmpInitialize();

	 // supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default


	
	 // make sure it worked (returns 0 if so)
	 if (devStatus == 0) {
		 // turn on the DMP, now that it's ready
		 mpu.setDMPEnabled(true);

		enable_interrupt0();
		 	
		 mpuIntStatus = mpu.getIntStatus();

		 // set our DMP Ready flag so the main loop() function knows it's okay to use it
		 //Serial.println(F("DMP ready! Waiting for first interrupt..."));
		 dmpReady = true;

		 // get expected DMP packet size for later comparison
		 packetSize = mpu.dmpGetFIFOPacketSize();
		 } else {
		 // ERROR!
		 // 1 = initial memory load failed
		 // 2 = DMP configuration updates failed
	 }
}


void MPU_update()
{
	// wait for MPU interrupt or extra packet(s) available
		if (!mpuInterrupt && fifoCount < packetSize) {
			return;
		}
		

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
			} else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			
			/*
			USART_WriteByte(ypr[0] * 180/M_PI);
			USART_WriteByte('\t');
			USART_WriteByte(ypr[1] * 180/M_PI);
			USART_WriteByte('\t');
			USART_WriteByte(ypr[2] * 180/M_PI);
			USART_WriteByte('\n');
			*/
		}
}