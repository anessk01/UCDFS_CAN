// Code for a non-master tranceiver on CAN
//
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2 // Set INT to pin 2
#define SYNC_SIGNAL 0x69
#define DEV_ID 0x02

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128]; // Array to store serial string

MCP_CAN CAN0(10);  // Set CS to pin 10

/*********************************************************************************************************
  HELPER FUNCTIONS
*********************************************************************************************************/
void CAN_INIT()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

  pinMode(5, OUTPUT);

  Serial.println("MCP2515 Library Receive Example...");
}

void CAN_READ()
{
  if (!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

    if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    Serial.print(msgString);

    if ((rxId & 0x40000000) == 0x40000000)
    { // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    }
    else
    {
      for (byte i = 0; i < len; i++)
      {
        if (i == 0 && rxBuf[i] == 0x01)
        {
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
          digitalWrite(5, status1);
          status1 = !status1;
        }
        else
        {
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
        }
        Serial.print(msgString);
      }
    }

    Serial.println();
  }
}

void CAN_WRITE(byte *data)
{
  // send data:  ID = DEV_ID, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(DEV_ID, 0, 8, data);
  if (sndStat == CAN_OK)
  {
    Serial.println("Message Sent Successfully!");
  }
  else
  {
    Serial.println("Error Sending Message...");
  }
}

void CAN_SYNC()  //function for master TX to send a unified sync signal every cycle that marks beginning of transmission
{
  byte data[8] = {SYNC_SIGNAL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // send data:  ID = DEV_ID, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(DEV_ID, 0, 8, data);
  if (sndStat == CAN_OK)
  {
    Serial.println("Message Sent Successfully!");
  }
  else
  {
    Serial.println("Error Sending Message...");
  }
}


int simulateSensor(int minDelayMS, int maxDelayMS, int minNumBytes, int maxNumBytes)
{
  int sensorVal, timedelay;
  randomSeed(analogRead(0)); //assumes pin 0 is unused
  sensorVal = random(pow(2, minNumBytes), pow(2, maxNumBytes));
  timeDelay = random(minDelayMS, maxDelayMS + 1);
  delay(timeDelay);
  return (sensorVal);
}

void fillDataArray(byte *data, int value, int position)
{ //data passed by reference
  if (position < 8)
    data[position] = value;
}

void clearDataArray(byte *data){
  for(int i =0; i<8; i++){
    fillDataArray(data, 0, i);
  }
}


/*********************************************************************************************************
  END FUNCTIONS
*********************************************************************************************************/

void setup()
{
  CAN_INIT();
}

int status1 = 0;

void loop()
{
  CAN
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
