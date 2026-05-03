/*
	Program for outdoor wireless temperature and humidity sensor.

	Feather 32u4 8MHz with RFM69HCW radio module
	SHT40 sensor
	
	Read temperature and humidity
	Send these values through RF 900MHz
	Sleep

*/

#include "Adafruit_SHT4x.h"
#include <SPI.h>
#include <RH_RF69.h> //RadioHead - modified RH_RF69.cpp line 182: disable CLKOUT to minimize the current consumption (see 3.2.2)
#include <avr/power.h>
#include "LowPower.h"

//Pinout
#define BATTERY_VOLTAGE A10
#define ANALOG_ENABLE 5
#define RF_CS 8
#define RF_RST 4
#define RF_IRQ 7

//RF
#define RF_FREQ 915.0
#define PACKET_SIZE 5
#define PREAMBLE_LENGTH 40 //long preamble

#define VOLTAGE_DIVIDER 66 // 3V3 * (15k+15k)/15k * 10 ; x10 to get results in deciVolts
#define ADC_RES 10 // 10bits

// Some macros is still missing from AVR GCC distribution for ATmega32U4
#define PRTIM4 4
#define power_timer4_disable()	(PRR1 |= (uint8_t)(1 << PRTIM4))

//#define DEBUG

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
RH_RF69 rf69(RF_CS, RF_IRQ); // Singleton instance of the radio driver

void setup()
{
#ifndef DEBUG
	delay(5000); //time to be able to upload a new sketch before entering sleep mode
#else
	delay(2000);
	Serial.println("Start");
#endif

	pinMode(RF_RST, OUTPUT);
	pinMode(RF_CS, OUTPUT);
	pinMode(ANALOG_ENABLE, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

#ifndef DEBUG
// Power optimization
	reduceActivePower();
#endif

	rf69.init();
	rf69.setPreambleLength(PREAMBLE_LENGTH);
	rf69.setFrequency(RF_FREQ);
	rf69.setTxPower(0, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
	rf69.sleep(); 

#ifndef DEBUG
	Serial.println("RF OK");
#endif

	sht4.begin();
	sht4.setPrecision(SHT4X_LOW_PRECISION);
	sht4.setHeater(SHT4X_NO_HEATER);

#ifndef DEBUG
	Serial.println("SHT40 OK");
#endif
	
}

void loop()
{
	uint16_t battery_voltage;
	uint8_t radiopacket[PACKET_SIZE]; 
	sensors_event_t hum, temp;

	digitalWrite(LED_BUILTIN, HIGH);

	//Read sensors
	delay(10);
	digitalWrite(ANALOG_ENABLE, HIGH); //Enable voltage divider
	battery_voltage = analogRead(BATTERY_VOLTAGE) * VOLTAGE_DIVIDER ;
	battery_voltage = battery_voltage >> ADC_RES;
	digitalWrite(ANALOG_ENABLE, LOW); //Disable voltage divider

	sht4.getEvent(&hum, &temp);// populate temp and humidity objects with fresh data ; 3.4ms
	int temperature = temp.temperature*10;
	int humidity = hum.relative_humidity;

	//Send data
	radiopacket[0] = highByte(temperature);
	radiopacket[1] = lowByte(temperature);
	radiopacket[2] = humidity;
	radiopacket[3] = battery_voltage;	

	#define NB_RETRY 3
	for(int i=0 ; i<NB_RETRY ; i++)
	{	
		radiopacket[4] = i; //message id
		rf69.send(radiopacket, PACKET_SIZE); //500u
		rf69.waitPacketSent(); //850us with preamble=4 ; 2000us with preamble=40
		rf69.sleep();
		if(i==NB_RETRY-1) break; //avoid sleep after the last emission
		//LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); //too slow to wake up
		//LowPower.powerSave(SLEEP_1S, ADC_OFF, BOD_OFF); //same as powerdown
#ifndef DEBUG
		LowPower.powerStandby(SLEEP_30MS, ADC_OFF, BOD_OFF);
#else
		delay(30);
#endif
	}
	digitalWrite(LED_BUILTIN, LOW);

#ifndef DEBUG
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
#else
	Serial.print(battery_voltage); Serial.print("V ");
	Serial.print(temperature); Serial.print("° ");
	Serial.print(humidity); Serial.print("%");
	Serial.println();
	delay(1000);
#endif	
}


void reduceActivePower(void)
{
	//Disable analog comparator and it's interrupts
	ACSR = (ACSR & ~_BV(ACIE)) | _BV(ACD);

	//power_timer1_disable(); //used by delay, micros, millis
	power_timer1_disable();
	//power_timer2_disable(); does not exist on 32u4
	power_timer3_disable();
	power_timer4_disable();
	power_usart0_disable(); //used by Serial
	power_usart1_disable();

	//power_spi_disable(); //used by RF
	//power_twi_disable(); //used by SHT40
	
//	Power Off the USB interface
// 	- Detach USB interface
//  - Disable USB interface
//  - Disable PLL
//  - Disable USB pad regulator
     
	UDCON &= ~(1<< DETACH); //Set to physically detach de device (disconnect internal pull-up on D+ or D-).    

	USBCON &= ~(1 << USBE ); // disable the USB
	USBCON &= ~(1 << OTGPADE ); // disable the VBUS pad
	USBCON &= ~(1 << VBUSTE ); //disable the VBUS Transition interrupt generation
	USBCON |= (1 << FRZCLK); // freeze the USB Clock
	UDIEN &= ~(1<< WAKEUPE); //disable the WAKEUPI interrupt.
	
	UDINT &= ~(1<< WAKEUPI); //clear WAKEUPI interrupt
	USBINT &= ~(1<< VBUSTI); //clear transition Interrupt Flag
	
	PLLCSR &= ~(1 << PLLE); // disable the USB Clock (PPL)
	
	UHWCON &= ~(1 << UVREGE); //disable the USB pad regulator

	power_usb_disable();
	
}
