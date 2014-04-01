// demo: CAN-BUS Shield, receive data
#include "mcp2515_can.h"
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[9];

MCP_CAN CAN0(10);                               // Set CS to pin 10

const uint8 pinCS = 10;

unsigned char stmp[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};

extern "C" void setup()
{
	Serial.begin(9600);
	pinMode (pinCS, OUTPUT);

	if(CAN0.begin(CAN_125KBPS) == CAN_OK)
		Serial.print("can init ok!!\r\n");
	else
		Serial.print("Can init fail!!\r\n"); 

	CAN0.reportState(0x2b);
	Serial.println("MCP2515 Library Receive Example...");
}

extern "C" void loop()
{
	char tmp[64];
	CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
	rxId = CAN0.getCanId();                    // Get message ID
	rxBuf[len] = '\0';
	sprintf(tmp, "id:%x, len:0x%x", rxId, len);
	Serial.println(tmp);

	Serial.print(rxId, HEX);
	Serial.print("  Data: ");
	for(int i = 0; i<len; i++)                // Print each byte of the data
	{
	  if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
	  {
	    Serial.print("0");
	  }
	  Serial.print(rxBuf[i], HEX);
	  Serial.print(" ");
	}
	Serial.println();
	delay(1000);
}

