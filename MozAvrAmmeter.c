/*
 *             LUFA Library
 *     Copyright (C) Dean Camera, 2013.
 * 
 *  dean [at] fourwalledcubicle [dot] com
 *           www.lufa-lib.org
 */

/*
 *  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)
 * 
 *  Permission to use, copy, modify, distribute, and sell this
 *  software and its documentation for any purpose is hereby granted
 *  without fee, provided that the above copyright notice appear in
 *  all copies and that both that the copyright notice and this
 *  permission notice and warranty disclaimer appear in supporting
 *  documentation, and that the name of the author not be used in
 *  advertising or publicity pertaining to distribution of the
 *  software without specific, written prior permission.
 * 
 *  The author disclaims all warranties with regard to this
 *  software, including all implied warranties of merchantability
 *  and fitness.  In no event shall the author be liable for any
 *  special, indirect or consequential damages or any damages
 *  whatsoever resulting from loss of use, data or profits, whether
 *  in an action of contract, negligence or other tortious action,
 *  arising out of or in connection with the use or performance of
 *  this software.
 */

/** \file
 * 
 *  Main source file for the MozAvrAmmeter project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "Delay.h"
#include "UART.h"
#include "adc.h"
#include "MozAvrAmmeter.h"
#include "PacketParser.h"
#include "I2C_Master.c"

/** Circular buffer to hold data received from the host. */
static RingBuffer_t USB_Receive_Buffer; // USBtoUSART_Buffer

/** Underlying data buffer for \ref USB_Receive_Buffer, where the stored bytes are located. */
static uint8_t      USB_Receive_Buffer_Data[128];

/** Circular buffer to hold data before it is sent to the host. */
static RingBuffer_t Send_USB_Buffer; // USARTtoUSB_Buffer

/** Underlying data buffer for \ref Send_USB_Buffer, where the stored bytes are located. */
static uint8_t      Send_USB_Buffer_Data[400];

static Sample_t samples[10];
static uint8_t currentSample = 0;
static uint8_t sendingSamples = 0;

static uint32_t heartbeatCycle;
static uint32_t heartbeatOnTime;
static uint32_t heartbeatCycleSize;

static int16_t lastCurrentReading = 0;

static float calibrationFloor;
static float calibrationScale;

static uint16_t serialNumber = 0;

// msTickCountBase is updated once per 250 ms in the Timer1 ISR
static uint32_t msTickCountBase = 0;

static uint8_t debugMode = 0;

static uint8_t compensation = 0;
static float baselineVoltage = 4.2;

static uint8_t loggingFlag = 0;
static uint8_t batteryCheckedFlag = 0;


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
	.Config =
	{
		.ControlInterfaceNumber         = 0,
		.DataINEndpoint                 =
		{
			.Address                = CDC_TX_EPADDR,
			.Size                   = CDC_TXRX_EPSIZE,
			.Banks                  = 1,
		},
		.DataOUTEndpoint                =
		{
			.Address                = CDC_RX_EPADDR,
			.Size                   = CDC_TXRX_EPSIZE,
			.Banks                  = 1,
		},
		.NotificationEndpoint           =
		{
			.Address                = CDC_NOTIFICATION_EPADDR,
			.Size                   = CDC_NOTIFICATION_EPSIZE,
			.Banks                  = 1,
		},
	},
};


#define MOSI_DDR	DDRB
#define MOSI_PORT   PORTB
#define MOSI_MASK   ( 1 << 2 )

#define MISO_DDR	DDRD
#define MISO_PORT   PORTD
#define MISO_PIN	PIND
#define MISO_MASK   ( 1 << 1 )

#define SCK_DDR	 DDRD
#define SCK_PORT	PORTD
#define SCK_MASK	( 1 << 5 )

#define SS_DDR	 DDRD
#define SS_PORT	PORTD
#define SS_MASK	( 1 << 0 )

#define BATTERY_DDR DDRB
#define BATTERY_PORT PORTB
#define BATTERY_MASK ( 1 << 6 )

#define ANALOG_ENABLE_DDR DDRF
#define ANALOG_ENABLE_PORT PORTF
#define ANALOG_ENABLE_MASK ( 1 << 6 )

#define CHARGE_FLAG_DDR DDRB
#define CHARGE_FLAG_PORT PORTB
#define CHARGE_FLAG_PIN PINB
#define CHARGE_FLAG_MASK ( 1 << 4 )

#define LED_DDR DDRE
#define LED_PORT PORTE
#define LED_MASK ( 1 << 6 )

#define FLAG_DDR DDRD
#define FLAG_PORT PORTD
#define FLAG_MASK ( 1 << 2 )

#define FLAG2_DDR DDRD
#define FLAG2_PORT PORTD
#define FLAG2_MASK ( 1 << 3 )

#define AUX_USB_POWER_DDR DDRB
#define AUX_USB_POWER_PORT PORTB
#define AUX_USB_POWER_MASK ( 1 << 0 )

#define AUX_USB_SWITCH_DDR DDRB
#define AUX_USB_SWITCH_PORT PORTB
#define AUX_USB_SWITCH_MASK ( 1 << 7 )


#define SPI_CLOCK_TIME 10

#define SetDirectionIn(x)   x ## _DDR &= ~ x ## _MASK; x ## _PORT &= ~ x ##_MASK
#define SetDirectionOut(x)  x ## _DDR |=   x ## _MASK

/**
 *   Functions for manipulating SCK, MOSI and MISO
 */

static inline void SPI_ClockLow( void )
{
	SCK_PORT &= ~SCK_MASK;
}

static inline void SPI_ClockHigh( void )
{
	SCK_PORT |= SCK_MASK;
}

static inline void SPI_SlaveSelectLow( void )
{
	SS_PORT &= ~SS_MASK;
}

static inline void SPI_SlaveSelectHigh( void )
{
	SS_PORT |= SS_MASK;
}

static inline void SPI_DataOut( uint16_t bit )
{
	if ( bit )
	{
		MOSI_PORT |= MOSI_MASK;
	}
	else
	{
		MOSI_PORT &= ~MOSI_MASK;
	}
}

static inline uint8_t SPI_DataIn( void )
{
	return ( MISO_PIN & MISO_MASK ) != 0;  
}

static inline void Battery_EnableHigh( void )
{
	BATTERY_PORT |= BATTERY_MASK;
}

static inline void Battery_EnableLow( void )
{
	BATTERY_PORT &= ~BATTERY_MASK;
}

static inline void Analog_EnableHigh( void )
{
	ANALOG_ENABLE_PORT |= ANALOG_ENABLE_MASK;
}

static inline void Analog_EnableLow( void )
{
	ANALOG_ENABLE_PORT &= ~ANALOG_ENABLE_MASK;
}

static inline void turnOnLED( void )
{
	LED_PORT |= LED_MASK;
}

static inline void turnOffLED( void )
{
	LED_PORT &= ~LED_MASK;
}

static inline void turnOnFlag( void )
{
	FLAG_PORT |= FLAG_MASK;
}

static inline void turnOffFlag( void )
{
	FLAG_PORT &= ~FLAG_MASK;
}

static inline void turnOnFlag2( void )
{
	FLAG2_PORT |= FLAG2_MASK;
}

static inline void turnOffFlag2( void )
{
	FLAG2_PORT &= ~FLAG2_MASK;
}

static inline uint8_t ChargeFlagIn( void )
{
	return (CHARGE_FLAG_PIN & CHARGE_FLAG_MASK) != 0;
}


//***************************************************************************
/**
 *   Initialize the SPI for Master mode
 */

static void SPI_MasterInit( void )
{
	SetDirectionOut(MOSI);
	SetDirectionIn(MISO);
	SetDirectionOut(SCK);
	SetDirectionOut(SS);
	
	// SCK idles low
	SPI_ClockLow();
	SPI_SlaveSelectHigh();
	
} // SPI_MasterInit

//***************************************************************************
/**
 *   Do a conversion according to the argument, then read and return the value
 */

static uint16_t SPI_DoConversion ( void )
{
	uint8_t i;
	uint16_t value = 0;
	
	cli();
	SPI_ClockHigh();
	SPI_SlaveSelectLow();
	for ( i = 0; i < 4; i++ )
	{
		// spin the clock 4 times to ignore the leading zero bits
		us_spin (SPI_CLOCK_TIME);
		SPI_ClockLow();
		us_spin (SPI_CLOCK_TIME);
		SPI_ClockHigh();
	}
	
	// retrieve 16 bits
	for (i = 0; i < 16; i++)
	{
		us_spin (SPI_CLOCK_TIME);
		SPI_ClockLow();
		us_spin (SPI_CLOCK_TIME);
		value <<= 1;
		value |= ( SPI_DataIn() & 0x01 );
		SPI_ClockHigh();
	}
	
	for ( i = 0; i < 4; i++ )
	{
		// spin the clock 4 times to ignore the trailing zero bits
		us_spin (SPI_CLOCK_TIME);
		SPI_ClockLow();
		us_spin (SPI_CLOCK_TIME);
		SPI_ClockHigh();
	}
	sei();
	
	SPI_SlaveSelectHigh();
	
	return value;
	
} // SPI_DoConversion


static void ReadCalibrationValues ( void )
{
	//printf("reading calibration constants from EEPROM\n");
	uint8_t signature = eeprom_read_byte((uint8_t*)(CALIBRATION_EEPROM_BASE + CALIBRATION_SIGNATURE_LOCATION));
	if (signature == CALIBRATION_SIGNATURE) {
		calibrationFloor = eeprom_read_float((float*)(CALIBRATION_EEPROM_BASE + CALIBRATION_FLOOR_LOCATION));
		calibrationScale = eeprom_read_float((float*)(CALIBRATION_EEPROM_BASE + CALIBRATION_SCALE_LOCATION));
	} else {
		//printf("Calibration signature doesn't match - using default values\n");
		calibrationFloor = 255.0;
		calibrationScale = 3.1909;
	}
	//char output[16];
	//dtostrf(calibrationFloor, 7, 4, output);
	//printf("FLOOR: %s\n", output);
	//dtostrf(calibrationScale, 7, 4, output);
	//printf("SCALE: %s\n", output);
}


static void ReadSerialNumber ( void )
{
	//printf("reading serial number from EEPROM\n");
	serialNumber = eeprom_read_word((uint16_t*)(CALIBRATION_EEPROM_BASE + SERIAL_NUMBER_LOCATION));
	//printf("Serial Number: %d\n", serialNumber);
}


static void ReadCompensationFlag ( void )
{
	compensation = eeprom_read_byte((uint16_t*)(CALIBRATION_EEPROM_BASE + COMPENSATION_FLAG_LOCATION));
	if (compensation == 0 || compensation == 1) {
		baselineVoltage = eeprom_read_float((float*)(CALIBRATION_EEPROM_BASE + BASELINE_VOLTAGE_LOCATION));
	} else {
		baselineVoltage = 4.2;
	}
}


static void ReadLoggingFlag ( void )
{
	loggingFlag = eeprom_read_byte((uint16_t*)(CALIBRATION_EEPROM_BASE + LOGGING_FLAG_LOCATION));
	if (loggingFlag) {
		InitUART ();
		fdevopen (UART1_PutCharStdio, UART1_GetCharStdio);
		turnOnAsyncCapture();
	}
}


void turnOnAsyncCapture ( void )
{
	cli(); // turn off interrupts so we can zero the ms counter
	currentSample = 0;
	msTickCountBase = 0;
	sei(); // turn interrupts back on
	sendingSamples = 1;
}


float readAuxBatteryVoltage ( void )
{
    I2C_Init(); // We bit-bang an I2C master on a couple pins from the programming port
    I2C_Start();
    I2C_Write(0x6C); // Write address of device
    I2C_Write(0x02); // register 2 is VCELL, which is the voltage of the cell
    I2C_Stop();
    I2C_Start();
    I2C_Write(0x6D); // Read address of device
    // Read VCELL register
    uint8_t highByte = I2C_Read(1);
    uint8_t lowByte = I2C_Read(0); // only the high nibble of the low byte is used
    I2C_Stop();
//     printf("High/Low: %d/%d\n", highByte, lowByte);
    uint16_t total = ((uint16_t)highByte * 16) + (lowByte / 16); // shift the high byte up 4 bits, and the low byte down 4 bits
    float voltage = (float)total * 0.00125; // the total is in units of 1.25 mV, so convert it to volts
//     printf("Total: %d\n", total);
    return voltage;
}


void AuxUSB_Disable ( void )
{
    // both of these switches are high disable/low enable...
    AUX_USB_POWER_PORT |= AUX_USB_POWER_MASK;
    AUX_USB_SWITCH_PORT |= AUX_USB_SWITCH_MASK;
}


void AuxUSB_Enable ( void )
{
    // both of these switches are high disable/low enable...
    AUX_USB_POWER_PORT &= ~AUX_USB_POWER_MASK;
    AUX_USB_SWITCH_PORT &= ~AUX_USB_SWITCH_MASK;
}


//
// The values in the table below are resistance values,
// provided by the manufacturer of the thermistor we're using.
// The table is indexed by temperature in degrees C, with
// the first value representing -30, and incrementing by 1 degree C
// for each table entry. The last entry is for 90 degrees C.
//
// http://www.ussensor.com/sites/default/files/downloads/USP12397%20(R-T%20Table).xls
//

const uint32_t temperatureLookup[] PROGMEM =
    {
        111337, 105805, 100579, 95639, 90969, 86551, 82373, 78418, 
        74675, 71131, 67774, 64594, 61580, 58723, 56014, 53444,
        51006, 48692, 46496, 44411, 42430, 40548, 38760, 37060, 
        35443, 33906, 32444, 31052, 29728, 28467, 27266, 26122, 
        25032, 23994, 23004, 22060, 21159, 20300, 19481, 18699, 
        17952, 17239, 16558, 15908, 15286, 14692, 14124, 13581, 
        13062, 12565, 12089, 11634, 11199, 10782, 10383, 10000, 
        9634, 9282, 8946, 8623, 8314, 8017, 7732, 7459, 
        7197, 6945, 6704, 6472, 6249, 6035, 5830, 5632, 
        5442, 5260, 5084, 4915, 4753, 4597, 4446, 4302, 
        4163, 4028, 3899, 3775, 3655, 3540, 3429, 3322, 
        3218, 3119, 3023, 2930, 2841, 2755, 2671, 2591, 
        2514, 2439, 2367, 2297, 2230, 2165, 2102, 2041, 
        1982, 1926, 1871, 1818, 1766, 1717, 1669, 1623, 
        1578, 1534, 1492, 1451, 1412, 1374, 1337, 1301, 
        1266
    };

uint8_t temperatureLookupCount = sizeof(temperatureLookup) / sizeof(temperatureLookup[0]);

float ReadTemperatureProbe ( void )
{
    uint32_t voltageValue = ADC_Read (1);
    uint32_t millivolts = voltageValue * 5070 / 1023;
    float realVoltage = (float)millivolts / 1000.0;
    uint32_t calculatedResistance = (uint32_t)((realVoltage * 10000.0) / (5.0 - realVoltage));
    int degrees = -100;
    for (int index = 0; index < temperatureLookupCount; index++) {
        uint32_t resistance = pgm_read_dword(&(temperatureLookup[index]));
        if ((resistance <= calculatedResistance) && (degrees == -100)) {
            degrees = index - 30;
        }
    }
    return (float)degrees;
//     float value = (float)ADC_Read(8);
//     return value;
}


static void PacketReceived (PACKET_Instance_t *inst, PACKET_Packet_t *packet, PACKET_Error_t err)
{
	if (err == PACKET_ERROR_NONE)
	{
		// we really only care about packets for us...
		if (packet->m_id == 0x01)
		{
			switch (packet->m_cmd)
			{
				case PACKET_CMD_SET_ID:
				{
					//printf ("got SET_ID command\n");
					break;
				}
				
				case PACKET_CMD_START_ASYNC:
				{
					//printf ("got START_ASYNC command\n");
					turnOnAsyncCapture();
					break;
				}
				
				case PACKET_CMD_STOP_ASYNC:
				{
					//printf ("got STOP_ASYNC command\n");
					sendingSamples = 0;
					break;
				}
				
				case PACKET_CMD_TURN_OFF_BATTERY:
				{
					//printf ("got TURN_OFF_BATTERY command\n");
					Battery_EnableLow();
					break;
				}
				
				case PACKET_CMD_TURN_ON_BATTERY:
				{
					//printf ("got TURN_ON_BATTERY command\n");
					Battery_EnableHigh();
					break;
				}
				
				case PACKET_CMD_SEND_SAMPLE:
				{
					//printf ("got PACKET_CMD_SEND_SAMPLE command\n");
					//turnOnFlag();
					currentSample = 0;
					CreateSample(SAMPLE_NORMAL);
					SendSamples(1, PACKET_CMD_SAMPLE);
					break;
				}
				
				case PACKET_CMD_SET_CALIBRATION:
				{
					//printf ("got PACKET_CMD_SET_CALIBRATION command\n");
					calibrationFloor = *(float *)&packet->m_param[0];
					calibrationScale = *(float *)&packet->m_param[4];
					eeprom_write_float((float*)(CALIBRATION_EEPROM_BASE + CALIBRATION_FLOOR_LOCATION), calibrationFloor);
					eeprom_write_float((float*)(CALIBRATION_EEPROM_BASE + CALIBRATION_SCALE_LOCATION), calibrationScale);
					eeprom_write_byte((uint8_t*)(CALIBRATION_EEPROM_BASE + CALIBRATION_SIGNATURE_LOCATION), CALIBRATION_SIGNATURE);
					//printf("Calibration parameters saved to EEPROM\n");
					//char output[16];
					//dtostrf(calibrationFloor, 7, 4, output);
					//printf("FLOOR: %s\n", output);
					//dtostrf(calibrationScale, 7, 4, output);
					//printf("SCALE: %s\n", output);
					break;
				}
				
				case PACKET_CMD_GET_CALIBRATION:
				{
					if (debugMode) {
						printf ("got PACKET_CMD_GET_CALIBRATION command\n");
					}
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0x01); // ammeter id
					uint8_t checksum = 0x01;
					uint8_t floatSize = sizeof(calibrationFloor);
					uint8_t packetLength = (floatSize * 2) + 2;
					RingBuffer_Insert(&Send_USB_Buffer, packetLength); // packet length, including all framing
					checksum += packetLength;
					RingBuffer_Insert(&Send_USB_Buffer, PACKET_CMD_CALIBRATION); // command
					checksum += PACKET_CMD_CALIBRATION;
					for (int index=0; index < floatSize; index++) {
						uint8_t value = ((uint8_t *)&calibrationFloor)[index];
						RingBuffer_Insert(&Send_USB_Buffer, value);
						checksum += value;
					}
					for (int index=0; index < floatSize; index++) {
						uint8_t value = ((uint8_t *)&calibrationScale)[index];
						RingBuffer_Insert(&Send_USB_Buffer, value);
						checksum += value;
					}
					RingBuffer_Insert(&Send_USB_Buffer, ~checksum);
					break;
				}
				
				case PACKET_CMD_GET_COMPENSATION:
				{
					if (debugMode) {
						printf ("got PACKET_CMD_GET_COMPENSATION command\n");
					}
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0x01); // ammeter id
					uint8_t checksum = 0x01;
					uint8_t floatSize = sizeof(calibrationFloor);
					uint8_t packetLength = 1 + floatSize + 2;
					RingBuffer_Insert(&Send_USB_Buffer, packetLength); // packet length, including all framing
					checksum += packetLength;
					RingBuffer_Insert(&Send_USB_Buffer, PACKET_CMD_COMPENSATION); // command
					checksum += PACKET_CMD_COMPENSATION;
					RingBuffer_Insert(&Send_USB_Buffer, compensation);
					checksum += compensation;
					for (int index=0; index < floatSize; index++) {
						uint8_t value = ((uint8_t *)&baselineVoltage)[index];
						RingBuffer_Insert(&Send_USB_Buffer, value);
						checksum += value;
					}
					RingBuffer_Insert(&Send_USB_Buffer, ~checksum);
					break;
				}
				
				case PACKET_CMD_GET_RAW_SAMPLE:
				{
					//printf ("got PACKET_CMD_GET_RAW_SAMPLE command\n");
					currentSample = 0;
					CreateSample(SAMPLE_RAW);
					SendSamples(1, PACKET_CMD_GET_RAW_SAMPLE);
					break;
				}
				
				case PACKET_CMD_GET_VERSION:
				{
					//printf ("got PACKET_CMD_GET_VERSION command\n");
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0x01); // ammeter id
					uint8_t checksum = 0x01;
					uint8_t packetLength = 1 + 2;
					RingBuffer_Insert(&Send_USB_Buffer, packetLength); // packet length, including all framing
					checksum += packetLength;
					RingBuffer_Insert(&Send_USB_Buffer, PACKET_CMD_VERSION); // command
					checksum += PACKET_CMD_VERSION;
					RingBuffer_Insert(&Send_USB_Buffer, AMMETER_VERSION);
					checksum += AMMETER_VERSION;
					RingBuffer_Insert(&Send_USB_Buffer, ~checksum);
					break;
				}
				
				case PACKET_CMD_GET_SERIAL:
				{
					if (debugMode) {
						printf ("got PACKET_CMD_GET_SERIAL command\n");
					}
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
					RingBuffer_Insert(&Send_USB_Buffer, 0x01); // ammeter id
					uint8_t checksum = 0x01;
					uint8_t packetLength = 2 + 2;
					RingBuffer_Insert(&Send_USB_Buffer, packetLength); // packet length, including all framing
					checksum += packetLength;
					RingBuffer_Insert(&Send_USB_Buffer, PACKET_CMD_SERIAL); // command
					checksum += PACKET_CMD_SERIAL;
					RingBuffer_Insert(&Send_USB_Buffer, serialNumber & 0xFF);
					checksum += serialNumber & 0xFF;
					RingBuffer_Insert(&Send_USB_Buffer, serialNumber >> 8);
					checksum += serialNumber >> 8;
					RingBuffer_Insert(&Send_USB_Buffer, ~checksum);
					break;
				}

				case PACKET_CMD_SET_SERIAL:
				{
					//printf ("got PACKET_CMD_SET_SERIAL command\n");
					serialNumber = *(uint16_t *)&packet->m_param[0];
					eeprom_write_word((float*)(CALIBRATION_EEPROM_BASE + SERIAL_NUMBER_LOCATION), serialNumber);
					//printf("Serial number saved to EEPROM\n");
					//printf("Serial Number: %d\n", serialNumber);
					break;
				}

				case PACKET_CMD_DUMP_DEBUG_INFO:
				{
					dumpDebugInfo();
					break;
				}

				case PACKET_CMD_TURN_ON_COMPENSATION:
				{
					float value = *(float *)&packet->m_param[0];
					if ((value >= 3.7) && (value <= 4.2)) {
						compensation = 1;
						eeprom_write_byte((float*)(CALIBRATION_EEPROM_BASE + COMPENSATION_FLAG_LOCATION), compensation);
						baselineVoltage = value;
						eeprom_write_float((float*)(CALIBRATION_EEPROM_BASE + BASELINE_VOLTAGE_LOCATION), baselineVoltage);
					}
					break;
				}
				
				case PACKET_CMD_TURN_OFF_COMPENSATION:
				{
					compensation = 0;
					eeprom_write_byte((float*)(CALIBRATION_EEPROM_BASE + COMPENSATION_FLAG_LOCATION), compensation);
					break;
				}
				
				case PACKET_CMD_START_LOGGING:
				{
					loggingFlag = 1;
					if (debugMode) {
						debugMode = 0;
					} else {
						InitUART ();
						fdevopen (UART1_PutCharStdio, UART1_GetCharStdio);
					}
					eeprom_write_byte((float*)(CALIBRATION_EEPROM_BASE + LOGGING_FLAG_LOCATION), loggingFlag);
					turnOnAsyncCapture();
					break;
				}
				
				case PACKET_CMD_STOP_LOGGING:
				{
					sendingSamples = 0;
					loggingFlag = 0;
					eeprom_write_byte((float*)(CALIBRATION_EEPROM_BASE + LOGGING_FLAG_LOCATION), loggingFlag);
					break;
				}
				
				case PACKET_CMD_CHECK_AUX_BATTERY:
				{
					loggingFlag = 0;
					if (debugMode) {
						debugMode = 0;
					} else {
						InitUART ();
						fdevopen (UART1_PutCharStdio, UART1_GetCharStdio);
					}
					printf("I2C Test\n");
                    float voltage = readAuxBatteryVoltage();
                    char output[16];
                    dtostrf(voltage, 4, 2, output);
                    printf("Battery Voltage: %s volts\n", output);
					break;
				}

                case PACKET_CMD_TURN_OFF_AUX_USB:
                {
                    //printf ("got TURN_OFF_BATTERY command\n");
                    AuxUSB_Disable();
                    break;
                }

                case PACKET_CMD_TURN_ON_AUX_USB:
                {
                    //printf ("got TURN_ON_BATTERY command\n");
                    AuxUSB_Enable();
                    break;
                }

                case PACKET_CMD_GET_TEMPERATURE:
                {
                    if (debugMode) {
                        printf ("got PACKET_CMD_GET_TEMPERATURE command\n");
                    }
                    float temperature = ReadTemperatureProbe();
                    RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
                    RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
                    RingBuffer_Insert(&Send_USB_Buffer, 0x01); // ammeter id
                    uint8_t checksum = 0x01;
                    uint8_t floatSize = sizeof(temperature);
                    uint8_t packetLength = floatSize + 2;
                    RingBuffer_Insert(&Send_USB_Buffer, packetLength); // packet length, including all framing
                    checksum += packetLength;
                    RingBuffer_Insert(&Send_USB_Buffer, PACKET_CMD_TEMPERATURE); // command
                    checksum += PACKET_CMD_TEMPERATURE;
                    for (int index=0; index < floatSize; index++) {
                        uint8_t value = ((uint8_t *)&temperature)[index];
                        RingBuffer_Insert(&Send_USB_Buffer, value);
                        checksum += value;
                    }
                    RingBuffer_Insert(&Send_USB_Buffer, ~checksum);
                    break;
                }
                
                case PACKET_CMD_SOFT_RESET:
                {
                    if (debugMode) {
                        printf ("got PACKET_CMD_SOFT_RESET command\n");
                    }
                    wdt_enable(WDTO_60MS);
                    while(1) {};
                    
                    break;
                }

                default:
				{
					// there are other commands that we don't care about....
					//printf ("ID:0x%02x Cmd: 0x%02x *** Unknown ***\n", packet->m_id, packet->m_cmd);
					break;
				}
			}
		}
		else {
			//printf ("Got packet for ID: %3d\n", packet->m_id);
		}
	}
	else if (packet->m_id == 0x01)
	{
		//printf ("CRC Error\n");
	}
}

static void ProcessUSB(PACKET_Instance_t *inst)
{
	/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
	if (!(RingBuffer_IsFull(&USB_Receive_Buffer)))
	{
		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		
		/* Store received byte into the USART transmit buffer */
		if (!(ReceivedByte < 0))
			RingBuffer_Insert(&USB_Receive_Buffer, ReceivedByte);
	}
	
	uint16_t BufferCount = RingBuffer_GetCount(&Send_USB_Buffer);
	if (BufferCount)
	{
		Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);
		
		/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
		 * until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
		if (Endpoint_IsINReady())
		{
			/* Never send more than one bank size less one byte to the host at a time, so that we don't block
			 * while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
			uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));
			if (debugMode) {
				printf ("Sending %d bytes\n", BytesToSend);
			}
			
			/* Read bytes from the USART receive buffer into the USB IN endpoint */
			while (BytesToSend--)
			{
				
				/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
				if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
					RingBuffer_Peek(&Send_USB_Buffer)) != ENDPOINT_READYWAIT_NoError)
				{
					if (debugMode) {
						printf("Error sending\n");
					}
					break;
				}
				
				/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
				if (debugMode) {
					printf ("Sent %x\n", RingBuffer_Peek(&Send_USB_Buffer));
				}
				RingBuffer_Remove(&Send_USB_Buffer);
			}
			//turnOffFlag();
			//printf("#");
		}
	}
	
	/* Process the next byte from USB */
	if (!(RingBuffer_IsEmpty(&USB_Receive_Buffer))) {
		uint8_t receivedByte = RingBuffer_Remove(&USB_Receive_Buffer);
		if (debugMode) {
			printf("USB got byte: 0x%02x\n", receivedByte);
		}
		
		PACKET_ProcessChar(inst, receivedByte);
	}
	
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
}


void LogSamples(uint8_t packetCount) {
	for (int i=0; i<packetCount; i++) {
		printf ("%u,%d,%lu\n", samples[i].voltage, samples[i].current, samples[i].msCounter);
	}	
}


void SendSamples(uint8_t packetCount, uint8_t command) {

	if (loggingFlag) {
		LogSamples(packetCount);
	} else {
		RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
		RingBuffer_Insert(&Send_USB_Buffer, 0xFF);
		RingBuffer_Insert(&Send_USB_Buffer, 0x01); // ammeter id
		uint8_t checksum = 0x01;
		uint8_t packetLength = (sizeof(Sample_t) * packetCount) + 2;
		RingBuffer_Insert(&Send_USB_Buffer, packetLength); // packet length, including all framing
		checksum += packetLength;
		RingBuffer_Insert(&Send_USB_Buffer, command); // command
		checksum += command;
		for (int i=0; i<packetCount; i++) {
			uint8_t * sampleStructPtr = (uint8_t*)&samples[i];
			for (int j=0; j<sizeof(samples[i]); j++) {
				RingBuffer_Insert(&Send_USB_Buffer, sampleStructPtr[j]);
				checksum += sampleStructPtr[j];
			}
		}
		RingBuffer_Insert(&Send_USB_Buffer, ~checksum);
	}
}

void CreateSample(int sampleType) {
	
	//turnOnFlag2();
	uint32_t total = 0;
	for (int index=0; index < 10; index++) {
		uint16_t value = SPI_DoConversion();
		total += value;
	}
	float f_value = (float)total / 10.0;
	float f_current;
	if (sampleType == SAMPLE_NORMAL) {
		if (f_value > calibrationFloor)
			f_value -= calibrationFloor;
		else
			f_value = 0.0;
		f_current = f_value / calibrationScale; // 3.1909 - we're adding a 10x factor here to give us tenths of a mA
	} else {
		f_current = f_value;
	}
	int16_t current = ((int16_t)f_current);
	//
	//		We're getting spurious negative readings when the current is low (<10 mA)
	//		Filter them out here
	//
	if (!ChargeFlagIn() && f_value > 500) {
		if (lastCurrentReading < 0) { // only go negative if we get two consencutive readings that are negative
			current *= -1;
		}
		lastCurrentReading = ((int16_t)f_current) * -1; // force lastCurrentReading negative regardless
	} else {
		lastCurrentReading = current;
	}
	uint32_t voltageValue = ADC_Read (0);
	uint32_t millivolts = voltageValue * 5000 / 1023;
	if ((sampleType == SAMPLE_NORMAL) && compensation) {
		float realVoltage = (float)millivolts / 1000.0;
		current = (int16_t)((float)current * realVoltage / baselineVoltage);
	}
	samples[currentSample].current = current;
	samples[currentSample].voltage = (uint16_t)millivolts;
	samples[currentSample].msCounter = getMsTickCount();
	//turnOffFlag2();
}

void ProcessSample() {
	
	if (!sendingSamples)
		return;
	
	CreateSample(SAMPLE_NORMAL);
	currentSample++;
	if (currentSample >= 10) {
		SendSamples(10, PACKET_CMD_ASYNC);
		currentSample = 0;
	}
}


void LEDHeartbeat (void)
{
	heartbeatCycle = getMsTickCount();
	heartbeatCycle %= heartbeatCycleSize;
	
	if (heartbeatCycle < heartbeatOnTime)
		turnOffLED();
	else
		turnOnLED();
}

void dumpDebugInfo (void)
{
	if (!debugMode) {
		InitUART ();
		fdevopen (UART1_PutCharStdio, UART1_GetCharStdio);
		debugMode = 1;
	}
	printf ("\n===\nDebug Mode Info\n\n");
	printf ("msTickCountBase: %lu\n", msTickCountBase);
	printf ("USB_Receive_Buffer Count: %u\n", RingBuffer_GetCount(&USB_Receive_Buffer));
	printf ("Send_USB_Buffer_Data Count: %u\n", RingBuffer_GetCount(&Send_USB_Buffer));
	if (UEINTX & (1 << TXINI)) {
		printf ("UEINTX -> TXINI: set\n");
	} else {
		printf ("UEINTX -> TXINI: clear\n");
	}
	printf ("Compensation: %u\n", compensation);
	char output[16];
	dtostrf(baselineVoltage, 7, 4, output);
	printf ("Baseline Voltage: %s\n", output);
	
	printf ("===\n\n");
}


void checkBattery (void)
{
    if ((getMsTickCount() % 60000) < 500) {
        if (!batteryCheckedFlag) {
            float voltage = readAuxBatteryVoltage();
            if ((voltage > 1.0) && (voltage < 3.2)) { // voltages below 1.0 indicate a spurious reading, so we'll ignore those...
                heartbeatCycle = 0;
                heartbeatOnTime = 400;
                heartbeatCycleSize = 500;
            }
            char output[16];
            dtostrf(voltage, 4, 2, output);
            printf("Battery Voltage: %s volts\n", output);
            batteryCheckedFlag = 1;
        }
    } else {
        batteryCheckedFlag = 0;
    }
}
            

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	PACKET_Instance_t inst;
	uint32_t previousCount;
	
	SetupHardware();
	
	// initialize the packet parser
	PACKET_Init (&inst);
	inst.m_pktRcvd = PacketReceived;
	
	RingBuffer_InitBuffer(&USB_Receive_Buffer, USB_Receive_Buffer_Data, sizeof(USB_Receive_Buffer_Data));
	RingBuffer_InitBuffer(&Send_USB_Buffer, Send_USB_Buffer_Data, sizeof(Send_USB_Buffer_Data));
	
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();
	
	for (;;) {
		previousCount = getMsTickCount();
		while (getMsTickCount() == previousCount) {
			ProcessUSB(&inst);
		}
		ProcessSample();
		LEDHeartbeat();
        if (loggingFlag) {
            checkBattery();
        }
	}
}

//===========================================================
//
//	SetupTickTimer()
//
//	We use a CTC 16 bit timer here to count ms in hardware.

void SetupTickTimer(void)
{
	// Set up Timer1 with prescaler = 64 and CTC mode
	// This assumes a 16 MHz crystal on the MCU
	TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
	
	// initialize counter
	TCNT1 = 0;
	
	// initialize compare value - 62499 = 250 ms ISR
	OCR1A = 62499;
	
	// enable compare interrupt
	TIMSK1 |= (1 << OCIE1A);
	
	// enable global interrupts
	sei();
}

//	This ISR will fire once per 250 ms.
//
//	The timer is in no way is dependent on the ISR, so any lag
//	introduced with servicing the ISR will be averaged out 
//	over many interrupts.

ISR (TIMER1_COMPA_vect)
{
	msTickCountBase += 250; // increment the ms counter base
	// toggle the I/O line so we can see it on a logic analyzer
	// FLAG2_PORT ^= FLAG2_MASK;
}

// Because of possible rollover issues, getMsTickCount() may not return
// the correct answer if called from inside an ISR or from inside a piece of
// code that has interrupts turned off.

uint32_t getMsTickCount(void)
{
	uint32_t subCount = TCNT1; // apparently the compiler makes this assignment atomic
	// TCNT1 is incrementing at a rate of 250 KHz, so by dividing its value
	// by 250, we get a 1000 Hz (or one millisecond) clock value.
	// Add that value to the base that is incremented in the ISR every 250 ms to get
	// a true monotonically increasing ms counter that is accurate regardless of processor load.
	return (msTickCountBase + (subCount / 250));
}

//===========================================================

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	SPI_MasterInit();
	
	SetDirectionIn(CHARGE_FLAG);
	SetDirectionOut(ANALOG_ENABLE);
	Analog_EnableHigh();
	
	// We set the battery enable PORT high first before the DDR so it will not get pulled low
	// in between setting the DDR and then the PORT
	Battery_EnableHigh();
	SetDirectionOut(BATTERY);
	Battery_EnableHigh();
	
	SetDirectionOut(LED);
	turnOffLED();

    SetDirectionOut(AUX_USB_POWER);
    SetDirectionOut(AUX_USB_SWITCH);
    AuxUSB_Enable();

// 	SetDirectionOut(FLAG);
// 	turnOffFlag();
// 	SetDirectionOut(FLAG2);
// 	turnOffFlag2();
	
	#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	
	/* Disable clock division */
	clock_prescale_set(clock_div_1);
	#endif
	
	SetupTickTimer();
	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	ADC_Init (ADC_PRESCALAR_AUTO);
	
	heartbeatCycle = 0;
	heartbeatOnTime = 1350;
	heartbeatCycleSize = 1500;
	
	float version = (float)AMMETER_VERSION / 10.0;
	char output[16];
	dtostrf(version, 2, 1, output);
	ReadCalibrationValues();
	ReadSerialNumber();
	ReadCompensationFlag();
	ReadLoggingFlag();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;
	
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
	
	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 * 
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;
	
	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}
	
	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
		ConfigMask |= (1 << USBS1);
	
	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}
}

