// Test
// This is the most upto date code for IESC 2026 Ortus 
// Utalises xbee protocol 
// 8 bit-7 seg codeing 
// Future updates include a update date here:
// Updated by: JLCJ 

// ----- LIBRARYS -----
#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_LEDBackpack.h>
#include "Ticker.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "DFRobot_LedDisplayModule.h"

// ----- DEFINE CONSTANTS -----
#define WHEEL_DIA 0.558               // Meters
#define MARAND_TRITIUM_AC_LIMIT 60.0  //Max Bus Current (Amps) Set in Tritium Configuration as well
#define ACC_THRESHOLD 0.1             // Percentage of the accelerator pedal must be down for it to be "Activated"

// ----- MCP2515 Transiver (CAN) -----
MCP2515 mcp2515(53);

// ----- Start LED 8-Bit Module -----
DFRobot_LedDisplayModule LED(&Wire, 0xE0);

// ----- Tickers (Functions that run at set time intervals) -----
Ticker timer(&setDriverControls22, 100, 0); // Set -- update set speed????
Ticker timer3(&dispSpeed, 200, 0);          // Can Be Whatever -- Timer to update the 7 seg speed 
Ticker timer4(&onCanReceived, 200, 0);      // Set -- Checks a can message was recived?
Ticker timer5(&TelemCANMsg, 1000, 0);       // Can Be Whatever -- Sends messages to CAN Network?

// ----- Select Input Pins -----
int SpeedPot = A0;
int CurrentPot = A1;
int AcceleratorPot  = A3; //select the input pin for the accelerator hall sensor

// ----- Set Important Variables to zero -----
float pot_speed_reading = 0; // From Pot
float pot_current_Read = 0; // From Pot
float pot_accelerator = 0;      //variable to store the value coming from the accelerator hall sensor
float raw_current = 0; //variable to store raw value from dash current pot - only needed for the accelerator mapping function at the very bottom (hall effect sensor code)
float send_Current = 0;
float send_Speed = 0;

// ----- Set Other Variables to zero -----
float pot_speed_reading_ms = 0;
float pot_speed_reading_kmh = 0;
float Sent_Speed_RPM = 0;
float maxBusCurrent = 1.0;
float actualspeed_ms = 0;
float actualspeedrpm = 0;
float actualspeedkmh = 0;
float Marand_SendAmps = 0;

// ----- Set Boolean & CAN Variables -----
bool ignition = 0;
bool regen = 0;
bool rev = 0;
bool brake = 0;
bool digital_accel = 0;
bool Battery_Flag = 0;
bool battery_temp = 0;
bool regenactive = 0;

float lowcell_voltage = 0;
float highcell_voltage = 0;
float motor_temp = 0;
float heat_sink_temp = 0;

int pwm_fan_pin = 10;  //set as output pin in the setup.

// ----- CAN Struct Frames -----
struct can_frame DriverControls_WS22; // CAN Message to WS22
struct can_frame busCurrent_WS22;     // CAN Message Bus Current WS22
struct can_frame msgReceived;         // Where Message is initally stored
struct can_frame TelemMessages;       // Telemetry Messages

// ----- IMPORTANT CAN STUFF -----
uint8_t conv_bytes[sizeof(float)];    //for conversion of float to bytes
uint8_t byte_buffer[11];              //for CAN messages in byte format to be transmitted. 11 is the max size.

// ----- CAN ID's -----
const uint16_t DrC22_POTENTIOMETER_ID = 0x101;
const uint16_t DrC22_BUS_ID = 0x102;

const uint16_t DrC_TELEM_ID = 0x500; 

const uint16_t TRITIUM22_BUS_MEASUREMENT_ID = 0x302;
const uint16_t TRITIUM22_VELOCITY_ID = 0x303;
const uint16_t TRITIUM22_TEMP_ID = 0x30b;

const uint16_t ORION_TEMP_ID = 0x701;
const uint16_t ORION_VOLTAGE_ID = 0x702;

///////////////////////////
//// ----- SETUP ----- ////
///////////////////////////
void setup() {

  Serial.begin(9600);
  
  // ----- SET UP DIGITAL PINS -----
  pinMode(45, INPUT_PULLUP);  // IGNITION
  pinMode(43, INPUT_PULLUP);  // REGEN
  pinMode(41, INPUT_PULLUP);  // REVERSE
  pinMode(37, INPUT_PULLUP);  // BRAKE PRESSED
  pinMode(39, INPUT_PULLUP);  // BRAKE NOT PRESSED
  pinMode(33, INPUT_PULLUP);  // ACCELERATOR
  pinMode(10, OUTPUT);        // pwm for tritium cooling fan.

  // ----- INITIALIZE 8-BIT & WAIT -----
  LED.begin(LED.e8Bit);
  delay(3000);
  LED.setDisplayArea(1,2,3,4,5,6,7,8);

  // ----- START MCP2515 CHIP -----
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // ----- Configure Timers -----
  timer.start();
  timer3.start();
  timer4.start();
  timer5.start();
}

/////////////////////////////////
//// ----- CAN DECODER ----- ////
/////////////////////////////////
void CANdecoder(float &variable1, float &variable2) {

  // ----- Set Up Variables for Data Frame -----
  // ----- All Data Transmitted at IEEE 754 -----
  uint32_t variable1_IEEE, variable2_IEEE;

  // ----- Extract First 4 BYTES -----
  variable1_IEEE = (((uint32_t)msgReceived.data[3] << 24) + ((uint32_t)msgReceived.data[2] << 16) + ((uint32_t)msgReceived.data[1] << 8) + (uint32_t)msgReceived.data[0]);
  // ----- Convert to IEEE 754 -----
  variable1 = *(float*)&variable1_IEEE;
    
  // ----- Extract Last 4 BYTES -----
  variable2_IEEE = (((uint32_t)msgReceived.data[7] << 24) + ((uint32_t)msgReceived.data[6] << 16) + ((uint32_t)msgReceived.data[5] << 8) + (uint32_t)msgReceived.data[4]);
  // ----- Convert to IEEE 754 -----
  variable2 = *(float*)&variable2_IEEE;

}

////////////////////////////////////////////////
//// ----- WHEN RECIVED A CAN MESSAGE ----- ////
////////////////////////////////////////////////

void onCanReceived() {
  // ----- If CAN Message with no Error -----
  if (mcp2515.readMessage(&msgReceived) == MCP2515::ERROR_OK) { 
    Serial.println("CAN ID MESSAGE RECIVED");
  
    // ----- Prints CAN Message Bit By Bit in HEX -----
    for (int i = 0; i < msgReceived.can_dlc; i++) {
      Serial.println(msgReceived.data[i], HEX);
      Serial.print(" ");
    }
  }

  // ----- READ TRITIUM VELOCITY -----
  if (msgReceived.can_id == TRITIUM22_VELOCITY_ID) {
    CANdecoder(actualspeedrpm, actualspeed_ms);
    dispSpeed();
  }

  // ----- READ TRITUM CASE TEMP -----
  if (msgReceived.can_id == TRITIUM22_TEMP_ID) {
    CANdecoder(motor_temp, heat_sink_temp);

    if (heat_sink_temp < 45) {
    analogWrite(pwm_fan_pin, 0);
  } else if (heat_sink_temp < 65) {
    analogWrite(pwm_fan_pin, 255);  //100% is 255
  } else {
    analogWrite(pwm_fan_pin, (255 * ((0.05 * heat_sink_temp) - 2.25))); // seems wrong e.g. if heat_sink_temp is 50 degrees 0.05*50 = 2.5, 2.5-2.25 = 0.25, 255/0.25 = 1025 where the max is 255
  }
  }

  // ----- READ ORION TEMP MESSAGES -----
  // WORK TO BE DONE FIND MESSAGED & WHAT TO DO WITH TEMP
  if (msgReceived.can_id == ORION_TEMP_ID) {
    battery_temp = msgReceived.data[4];
  }
  if (battery_temp > 45) {
    Battery_Flag = 1;
  }

  // ----- READ ORION VOLTAGE MESSAGES -----
  if (msgReceived.can_id == ORION_VOLTAGE_ID) {
    lowcell_voltage = ( ((msgReceived.data[1] << 8) + msgReceived.data[0]) ) / 10000.0;
    highcell_voltage = ( ((msgReceived.data[3] << 8) + msgReceived.data[2]) ) / 10000.0;
  }
  if (highcell_voltage > 4.45) {
    Battery_Flag = 1;
  }
  if (lowcell_voltage < 2.75){
    Battery_Flag = 1;
  }
}

//////////////////////////////////////////////
//// ----- SEND MESSAGES TO TRITIUM ----- ////
//////////////////////////////////////////////

void setDriverControls22() {
  // ----- CAN ID & DLC -----
  DriverControls_WS22.can_id = DrC22_POTENTIOMETER_ID;
  DriverControls_WS22.can_dlc = 8;

  // ----- CLEAR MESSAGE -----
  for (int c = 0; c < 8; c++) {
    DriverControls_WS22.data[c] = 0x00;
  }

  // ----- WRITE DESIRED SPEED & CURRENT -----
  // ----- BIT 00-31: RPM -----
  *(float*)(conv_bytes) = send_Speed * 60 / (3.1415 * WHEEL_DIA);
  DriverControls_WS22.data[0] = conv_bytes[0];
  DriverControls_WS22.data[1] = conv_bytes[1];
  DriverControls_WS22.data[2] = conv_bytes[2];
  DriverControls_WS22.data[3] = conv_bytes[3];

  // ----- BIT 32-63: Set Current -----
  *(float*)(conv_bytes) = send_Current;
  DriverControls_WS22.data[4] = conv_bytes[0];
  DriverControls_WS22.data[5] = conv_bytes[1];
  DriverControls_WS22.data[6] = conv_bytes[2];
  DriverControls_WS22.data[7] = conv_bytes[3];

  // ----- SEND MESSAGE TO CAN -----
  mcp2515.sendMessage(&DriverControls_WS22);

  // ----- CAN ID & DLC -----
  busCurrent_WS22.can_id = DrC22_BUS_ID;
  busCurrent_WS22.can_dlc = 8;

  // ----- CLEAR MESSAGE -----
  for (int c = 0; c < 8; c++) {
    busCurrent_WS22.data[c] = 0x00;
  }

  // ----- WRITE BUS CURRENT -----
  // ----- BIT 00-31: BUS CURRENT -----
  *(float*)(conv_bytes) = maxBusCurrent;
  busCurrent_WS22.data[4] = conv_bytes[0];
  busCurrent_WS22.data[5] = conv_bytes[1];
  busCurrent_WS22.data[6] = conv_bytes[2];
  busCurrent_WS22.data[7] = conv_bytes[3];

  // ----- SEND MESSAGE TO CAN -----
  mcp2515.sendMessage(&busCurrent_WS22);

}

////////////////////////////////////
//// ----- TELEM MESSAGES ----- ////
////////////////////////////////////

void TelemCANMsg(){

  // ----- CAN ID & DLC -----
  TelemMessages.can_id = DrC_TELEM_ID;
  TelemMessages.can_dlc = 8;

  // ----- CLEAR MESSAGE -----
  for (int c = 0; c < 8; c++) {
    TelemMessages.data[c] = 0x00;
  }

  // ----- WRITE DRIVER SET CURRENT & SEND CURRENT -----
  // ----- BIT 00-31: DRIVER SET CURRENT -----
  *(float*)(conv_bytes) = pot_current_Read;
  TelemMessages.data[0] = conv_bytes[0];
  TelemMessages.data[1] = conv_bytes[1];
  TelemMessages.data[2] = conv_bytes[2];
  TelemMessages.data[3] = conv_bytes[3];
  
  // ----- BIT 32-63: SEND Current -----
  *(float*)(conv_bytes) = send_Current;
  TelemMessages.data[4] = conv_bytes[0];
  TelemMessages.data[5] = conv_bytes[1];
  TelemMessages.data[6] = conv_bytes[2];
  TelemMessages.data[7] = conv_bytes[3];

  // ----- SEND MESSAGE TO CAN -----
  mcp2515.sendMessage(&TelemMessages);
}

///////////////////////////////////
//// ----- 8-BIT DISPLAY ----- ////
///////////////////////////////////
void dispSpeed() {
  // ----- Set Speed Position(1&2), Actual Speed Position(3&4), Set Current Position(7&8) -----
  // ----- Set Brightness -----
  LED.setBrightness(5);

  // ----- Combine Values into one string (8 digits + null terminator) -----
  char buffer[9];

  // ----- Set Variables -----
  int acc = 0;

  // ----- Get Variables -----
  int current_disp = pot_current_Read * 100.0;
  actualspeedkmh = abs(actualspeed_ms) * 3.6;
  int acc_disp = pot_accelerator * 100.0; // *********************
  regenactive = !regen && !brake;

  // ----- Display -----
  // ----- If Acceleration 0 (ON) -----
  if (!digital_accel == 1){ 
    LED.setDisplayArea(1,2,3,4,5,6,7,8);
    sprintf(buffer, "%02d%02d%01d%01d%02d", int(pot_speed_reading_kmh), int(actualspeedkmh), regenactive, !digital_accel, current_disp);

  // ----- If Acceleration 1 (OFF) -----
  } else { 
    LED.setDisplayArea(1,2,3,4,5,6,7,8);
    sprintf(buffer, "%02d%02d%01d%01d%02d", int(pot_speed_reading_kmh), int(actualspeedkmh), regenactive, !digital_accel, current_disp); 
  }
  
  // ----- Print String to 8-BIT LED -----
  // ----- atof to convert string to float -----

  LED.print(atof(buffer));

}

///////////////////////////////
//// ----- Main Loop ----- ////
///////////////////////////////
void loop() {

   // ----- 800 Used as when read pot would not give full output as they are connected in Parallel -----
  pot_current_Read = analogRead(CurrentPot) / 800.0;
  pot_speed_reading = analogRead(SpeedPot) / 800.0;

  // ----- Calc Variable -----
  pot_speed_reading_ms = pot_speed_reading * 30.0;
  pot_speed_reading_kmh = pot_speed_reading_ms * 3.6;
  Sent_Speed_RPM = send_Speed * 60 / (3.1415 * WHEEL_DIA);

  // ----- Calc Fraction of Max Current (60A) to send -----
  Marand_SendAmps = pot_current_Read * MARAND_TRITIUM_AC_LIMIT;

  // ----- SAFTEY -----
  if (Marand_SendAmps > MARAND_TRITIUM_AC_LIMIT) {
    Marand_SendAmps = MARAND_TRITIUM_AC_LIMIT;
  }

  // ----- READ INPUTS -----
  ignition = digitalRead(45);
  regen = digitalRead(43);
  rev = digitalRead(41);
  brake = digitalRead(37);
  digital_accel = digitalRead(33);

  // ----- PRINT USEFUL INFOMATION -----
  // ----- PRINT POT INFOMATION -----
  Serial.println("--------------------------------------------------------------------------------------");
  Serial.print("Current Pot: ");
  Serial.print(pot_current_Read);
  Serial.print(" Speed Pot: ");  
  Serial.println(pot_speed_reading);
  
  // ----- PRINT REAL TIME INFOMATION -----
  Serial.print("Current: ");
  Serial.print(Marand_SendAmps); // This is the fraction of the current otherwise you dont know what fraction is being send contiuniely, the current always gets sent at 0.0 A through the pedal
  Serial.print(" Sent Speed: ");
  Serial.print(Sent_Speed_RPM);
  Serial.print("  Motor Speed (kmh): ");
  Serial.println(abs(actualspeed_ms)*3.6);

  // ----- PRINT SWITCH INFOMATION -----
  Serial.print("Ignition: ");
  Serial.print(!ignition);
  Serial.print(" Direction: ");
  Serial.print(!rev);
  Serial.print(" Brake: ");
  Serial.print(!brake);
  Serial.print(" Regen: ");
  Serial.print(!regen);
  Serial.print(" Acc: ");
  Serial.println(!digital_accel);
  Serial.print("Actively Reg: ");
  Serial.println(regenactive);

  //////////////////////////////////////
  //// ----- BIG IF STATEMENT ----- ////
  //////////////////////////////////////

  // -- IF IGNITION ON --
  if (!ignition) {
    // -- IF BRAKE ON --
    if (!brake) {
      // -- Send Speed: 0.0 --
      send_Speed = 0.0;
      // -- Check if regen is on --
      if (!regen){
        // -- Send Current to desired regen current (%) --
        send_Current = Marand_SendAmps / MARAND_TRITIUM_AC_LIMIT;
      // -- Regen Off --
      } else {
        // -- Set Both to Zero --
        send_Current = 0.0;

      }

    // -- Not Breaking --
    } else {
      // -- Accerating (digital_accel = 0) therfore (!digital_accel = 1 / True) --
      if (!digital_accel){
        // -- Reverse --
        if (rev){
          send_Speed = (-1) * pot_speed_reading_ms;
          send_Current = Marand_SendAmps / MARAND_TRITIUM_AC_LIMIT;
        // -- Forward --
        } else {
          send_Speed = pot_speed_reading_ms;
          send_Current = Marand_SendAmps / MARAND_TRITIUM_AC_LIMIT;
        }
      
      // -- Not Accelerating -- 
      } else {
        send_Current = 0.0;
      }

    }

  } // -- Ignition Off --
  else {
    send_Current = 0.0;
  }


  timer.update();
  timer3.update();
  timer4.update();
  timer5.update();


} // end main loop
