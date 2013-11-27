/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the MozAvrAmmeter project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "Delay.h"
#include "Timer.h"
#include "UART.h"
#include "adc.h"
#include "MozAvrAmmeter.h"
#include "PacketParser.h"

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

static uint16_t heartbeatCycle;
static uint16_t heartbeatOnTime;
static uint16_t heartbeatCycleSize;


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
					printf ("got SET_ID command\n");
					break;
				}

				case PACKET_CMD_START_ASYNC:
				{
					printf ("got START_ASYNC command\n");
					cli(); // turn off interrupts so we can zero the ms counter
					currentSample = 0;
					gMsTickCount = 0;
					sei(); // turn interrupts back on
					sendingSamples = 1;
					break;
				}

				case PACKET_CMD_STOP_ASYNC:
				{
					printf ("got STOP_ASYNC command\n");
					sendingSamples = 0;
					break;
				}

				case PACKET_CMD_TURN_OFF_BATTERY:
				{
					printf ("got TURN_OFF_BATTERY command\n");
					Battery_EnableLow();
					break;
				}

				case PACKET_CMD_TURN_ON_BATTERY:
				{
					printf ("got TURN_ON_BATTERY command\n");
					Battery_EnableHigh();
					break;
				}

				case PACKET_CMD_SEND_SAMPLE:
				{
					printf ("got PACKET_CMD_SEND_SAMPLE command\n");
					currentSample = 0;
					CreateSample();
					SendSamples(1, PACKET_CMD_SAMPLE);
					break;
				}

				default:
				{
					// there are other commands that we don't care about....
					printf ("ID:0x%02x Cmd: 0x%02x *** Unknown ***\n", packet->m_id, packet->m_cmd);
					break;
				}
			}
		}
		else
			printf ("Got packet for ID: %3d\n", packet->m_id);
	}
	else if (packet->m_id == 0x01)
	{
		printf ("CRC Error\n");
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
			// printf ("Sending %d bytes\n", BytesToSend);

			/* Read bytes from the USART receive buffer into the USB IN endpoint */
			while (BytesToSend--)
			{

				/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
				if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
										RingBuffer_Peek(&Send_USB_Buffer)) != ENDPOINT_READYWAIT_NoError)
				{
					printf("Error sending\n");
					break;
				}

				/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
				// printf ("Sent %x\n", RingBuffer_Peek(&Send_USB_Buffer));
				RingBuffer_Remove(&Send_USB_Buffer);
			}
		}
	}

	/* Process the next byte from USB */
	if (!(RingBuffer_IsEmpty(&USB_Receive_Buffer))) {
		uint8_t receivedByte = RingBuffer_Remove(&USB_Receive_Buffer);
// 		printf("USB got byte: 0x%02x\n", receivedByte);
		PACKET_ProcessChar(inst, receivedByte);
	}

	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
}

void SendSamples(uint8_t packetCount, uint8_t command) {

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
	RingBuffer_Insert(&Send_USB_Buffer, checksum);
}

void CreateSample() {

	uint32_t total = 0;
	for (int index=0; index < 10; index++) {
	  uint16_t value = SPI_DoConversion();
	  total += value;
	}
	float f_value = (float)total / 10.0;
	if (f_value > 220.0)
		f_value -= 220.0;
	else
		f_value = 0.0;
	float f_current = f_value / 26.679;
	int16_t current = ((int)f_current);
	if (!ChargeFlagIn() && f_value > 500)
		current *= -1;
	uint16_t voltageValue = ADC_Read (0);
	uint16_t voltage = voltageValue * 5;
	samples[currentSample].current = current;
	samples[currentSample].voltage = voltage;
	samples[currentSample].msCounter = getMsTickCount();
}

void ProcessSample() {
	
	if (!sendingSamples)
		return;

	CreateSample();
	currentSample++;
	if (currentSample >= 10) {
		SendSamples(10, PACKET_CMD_ASYNC);
		currentSample = 0;
	}
}


void LEDHeartbeat (void)
{
	heartbeatCycle += 1;
	heartbeatCycle %= heartbeatCycleSize;

	if (heartbeatCycle == 0)
		turnOffLED();
	if (heartbeatCycle == heartbeatOnTime)
		turnOnLED();
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	PACKET_Instance_t inst;
	msTick_t previousCount;

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
	}
}

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

	InitTimer();
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	ADC_Init (ADC_PRESCALAR_AUTO);
	InitUART ();
	fdevopen (UART1_PutCharStdio, UART1_GetCharStdio);

	heartbeatCycle = 0;
	heartbeatOnTime = 1350;
	heartbeatCycleSize = 1500;

	printf ("\nUSB Test\n\n");
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

