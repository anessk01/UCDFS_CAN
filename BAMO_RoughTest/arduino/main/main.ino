//#include "VCU.h"
#include "CAN.h"
#define regReadBamocarData        0x3D

//BAMO Registers:
static const byte MOTOR_CONTROLLER_ADDRESS_VOLTAGE_BUS = 0xEB;
const byte MOTOR_CONTROLLER_ADDRESS_VOLTAGE_OUTPUT = 0x8A;
static const byte MOTOR_CONTROLLER_ADDRESS_CURRENT_PACK = 0x20;
const byte MOTOR_CONTROLLER_ADDRESS_CURRENT_PHASE = 0x5F;
const byte MOTOR_CONTROLLER_ADDRESS_MOTOR_TEMPERATURE = 0x49;
const byte MOTOR_CONTROLLER_ADDRESS_IGBT_TEMPERATURE = 0x4A;
const byte MOTOR_CONTROLLER_ADDRESS_SETPOINT_CURRENT = 0x22;
const byte MOTOR_CONTROLLER_ADDRESS_SETPOINT_TORQUE = 0x90;
const byte MOTOR_CONTROLLER_ADDRESS_RPM = 0xA8;
const byte MOTOR_CONTROLLER_ADDRESS_MOTOR_POSITION = 0x6D;  // TODO: Unused

int led = 13;
unsigned const int canMotorControllerAddress = 528; // 0x210
char msg[120];

//CANControllerClass CAN;
void sendMotorControllerMessage(unsigned char *bytes, int messageLength) {
    CAN.beginPacket(canMotorControllerAddress); // ???
    for (int i = 0; i < messageLength; i++) {
        CAN.write(bytes[i]);
        sprintf(msg, "Sent %d \n", bytes[i]);
        Serial.print(msg);
    }
    CAN.endPacket();
}

void setTorqueValue(int value) {
    unsigned char canTorquePacket[3] = {MOTOR_CONTROLLER_ADDRESS_SETPOINT_TORQUE, (byte)value, (byte)(value>>8)};
    sendMotorControllerMessage(canTorquePacket, 3);
}

void requestMotorControllerRegisterOnce(byte registerAddress) {  //function to read from reg with passed address
  unsigned char packetToSend[3] = {regReadBamocarData, registerAddress, 0x00};  // sends 0x3D to specify read from reg
  sendMotorControllerMessage(packetToSend, 3);
}
//_____________________________________________________________________________________

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);

  //init CAN
  CAN.setPins(10, 2);
    if (!CAN.begin((long)500E3)) {
      while(true){
        Serial.println("Starting CAN failed!");
        delay(100);
      }
    }else{
      Serial.println("CAN started");
    }
}

// the loop routine runs over and over again forever:
void loop() {
  setTorqueValue(20);
  delay(1000);
  requestMotorControllerRegisterOnce(MOTOR_CONTROLLER_ADDRESS_MOTOR_TEMPERATURE);  //0x49
  delay(1000);
}
