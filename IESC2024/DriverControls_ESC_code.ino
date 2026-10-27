// IESC 2024 Code
// date: N/A
// Updated by: EGG, Vedika bede
// Should inlcude a 8-bit 7 seg and MCP2515 CAN communication 


// ---------- Libaries ----------------
#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_LEDBackpack.h>
#include "Ticker.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "DFRobot_LedDisplayModule.h"

#define WHEEL_DIA 0.558                     //in meters

#define MARAND_TRITIUM_AC_LIMIT 60.0  //Bus Current Limit (Amps) Set In Tritium Configuration
#define GERALD_TRITIUM_AC_LIMIT 60.0  //Bus Current Limit (Amps) Set In Tritium Configuration

#define MARAND_FRACTION 0.6
#define GERALD_FRACTION 0.4

#define ACC_THRESHOLD 0.1 // what percentage the accelerator pedal starts to accelerate at. 10% at the moment - for safety purposes. 


float TotalSendAmps = 0;
float Marand_SendAmps = 0;
float Gerald_SendAmps = 0;

float MaxTotalAmps = MARAND_TRITIUM_AC_LIMIT + GERALD_TRITIUM_AC_LIMIT; // 120A (if both set at 60A)



//CAN transceiver using pin MCP_SS
MCP2515 mcp2515(53);

// In default, the IIC address of 8 bits digital tube is 0xE0
// The IIC address of 8 bits digital tube can be changed by combining A1 and A0
DFRobot_LedDisplayModule LED(&Wire, 0xE0);  // Initialise an instance of the Adafruit_7segment Class


Ticker timer(&setDriverControls22, 200, 0);  // do this every 200 ms to be safe
Ticker timer2(&setDriverControls20, 200, 0);
Ticker timer3(&dispSpeed, 200, 0);
Ticker timer4(&onCanReceived, 200, 0);
Ticker timer5(&TelemCANMsg, 1000, 0); // no frequency requirement here.


int SpeedPot = A0;    //select the input pin for the speed potentiometer (integer because pin A0 typically has value pin14)
int CurrentPot = A1;  //select the input pin for the current potentiometer
int AcceleratorPot  = A3; //select the input pin for the accelerator hall sensor

float pot_speed_reading = 0;    //variable to store the value coming from the potentiometer
float pot_current_reading = 0;  //variable to store the value coming from the potentiometer

float pot_accelerator = 0;      //variable to store the value coming from the accelerator hall sensor
float raw_current = 0; //variable to store raw value from dash current pot - only needed for the accelerator mapping function at the very bottom (hall effect sensor code)
float raw_speed = 0;
float raw_accelerator_pot = 0;

float which_accelerator_current22 = 0;
float which_accelerator_current20 = 0;
bool acceleration = 0;

float pot_speed_reading_ms = 0;
float pot_speed_reading_kmh = 0;

// float accelerator_current = 0;
float send_current_22 = 0;
float send_current_20 = 0;
float send_speed = 0;

float maxBusCurrent = 1.0;

float DCbus_current_22 = 0;
float DCbus_voltage_22 = 0;

float DCbus_current_20 = 0;
float DCbus_voltage_20 = 0;

float actualspeed_ms = 0;
float actualspeedrpm = 0;
float actualspeedkmh = 0;

bool ignition = 0;
bool regen = 0;
bool rev = 0;
bool brake = 0;
bool digital_accel = 0;

bool batteryfail = 0;
bool battery_temp = 0;
float lowcell_voltage = 0;
float highcell_voltage = 0;

int pwm_fan_pin = 10;  //set as output pin in the setup.
float motor_temp = 0;
float heat_sink_temp = 0;




struct can_frame DriverControls_WS20;  // CAN message for driver controls to communicate with WS20
struct can_frame DriverControls_WS22;  // CAN message for driver controls to communicate with WS22
struct can_frame busCurrent_WS20;      // reserved for status on bus current to communicate with WS20
struct can_frame busCurrent_WS22;      // reserved for status on bus current to communicate with WS22
struct can_frame msgReceived;     //where the received CAN messages will be stored
struct can_frame TelemMessages;     //where the received CAN messages will be stored


uint8_t conv_bytes[sizeof(float)];  //for conversion of float to bytes
uint8_t byte_buffer[11];            //for CAN messages in byte format to be transmitted. 11 is the max size.


//DRIVER CONTROLS ID for WS22 = 0X100
//DRIVER CONTROLS ID for WS20 = 0X200
// TRITIUM 22 ID = 0X300
// TRITIUM 20 ID = 0X400
// BMS 2 ID = 0X700


const uint16_t DrC20_POTENTIOMETER_ID = 0x201;
const uint16_t DrC20_BUS_ID = 0x202;

const uint16_t DrC22_POTENTIOMETER_ID = 0x101;
const uint16_t DrC22_BUS_ID = 0x102;

const uint16_t DrC_TELEM_ID = 0x500; 

const uint16_t TRITIUM22_BUS_MEASUREMENT_ID = 0x302; // this is the DC Bus measurement from tritium (power flow in/out of tritium)
const uint16_t TRITIUM22_VELOCITY_ID = 0x303;
const uint16_t TRITIUM22_TEMP_ID = 0x30b;

const uint16_t TRITIUM20_BUS_MEASUREMENT_ID = 0x402;
const uint16_t TRITIUM20_VELOCITY_ID = 0x403;
const uint16_t TRITIUM20_TEMP_ID = 0x40b;

const uint16_t ORION_TEMP_ID = 0x701; // check ORION ID
const uint16_t ORION_VOLTAGE_ID = 0x702;

const uint8_t DISPLAY_ADDR = 0x70;  // I2C address of the 7 seg


// uint8_t dig1 = 0;
// uint8_t dig2 = 0;



void setup() {

  Serial.begin(9600);
  // Wire.setClock(50000); //default clock speed of I2C is 100kHz. 
  // Wire.begin(0x70);

  // put your setup code here, to run once:
  //sets the digital pins as an input pin
  pinMode(45, INPUT_PULLUP);  // IGNITION
  pinMode(43, INPUT_PULLUP);  // REGEN
  pinMode(41, INPUT_PULLUP);  // REVERSE
  pinMode(37, INPUT_PULLUP);  // BRAKE PRESSED
  pinMode(39, INPUT_PULLUP);  // BRAKE NOT PRESSED
  pinMode(33, INPUT_PULLUP);  // ACCELERATOR
  pinMode(10, OUTPUT);  //pwm for tritium cooling fan.

  pinMode(13, OUTPUT);

  /*Wait for the chip to be initialized completely, and then exit*/

  LED.begin(LED.e8Bit);
  delay(3000);

  // while(LED.begin(LED.e8Bit) != 0) {
  //   Serial.println("Failed to initialize the chip , please confirm the chip connection!");
  //   delay(1000);
  // }

  LED.setDisplayArea(1,2,3,4,5,6,7,8);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  timer.start();
  timer2.start();
  timer3.start();
  timer4.start();
  timer5.start();
}



///////////////// CAN DECODER /////////////

void CANdecoder(float &variable1, float &variable2) {

  uint32_t variable1_IEEE, variable2_IEEE;

  // Extract the first 4 bytes for variable1
  variable1_IEEE = (((uint32_t)msgReceived.data[3] << 24) + ((uint32_t)msgReceived.data[2] << 16) + ((uint32_t)msgReceived.data[1] << 8) + (uint32_t)msgReceived.data[0]);
  variable1 = *(float*)&variable1_IEEE; // Function to convert IEEE 754 bytes to float
    
  // Extract the last 4 bytes for actualspeed_ms
  variable2_IEEE = (((uint32_t)msgReceived.data[7] << 24) + ((uint32_t)msgReceived.data[6] << 16) + ((uint32_t)msgReceived.data[5] << 8) + (uint32_t)msgReceived.data[4]);
  variable2 = *(float*)&variable2_IEEE; // Function to convert IEEE 754 bytes to float

}

////////////////////////////////////////////////



void onCanReceived() {


  if (mcp2515.readMessage(&msgReceived) == MCP2515::ERROR_OK) {  //can probably take this out of the IF statement and get rid of serial commands?
    Serial.println("GENERAL CAN MESSAGE RECEIVED ID");
    // Serial.println(msgReceived.can_id, HEX);  // print ID
    // Serial.print(" ");
    // Serial.print(canMsg.can_dlc, HEX); // print DLC
    // Serial.print(" ");
    for (int i = 0; i < msgReceived.can_dlc; i++) {  // print the data
      Serial.println(msgReceived.data[i], HEX);      //length of data message - can mssg is an array
      Serial.print(" ");
    }
    //Serial.println();
  }

  //READ TRITIUM CAN MESSAGE: VEHICLE VELOCITY RPM AND M/S
  if (msgReceived.can_id == TRITIUM22_VELOCITY_ID) {
    CANdecoder(actualspeedrpm, actualspeed_ms);
    dispSpeed();
  }


  // READ THE TEMPERATURE CAN MESSAGE FROM TRITIUM:
  if (msgReceived.can_id == TRITIUM22_TEMP_ID) {
    CANdecoder(motor_temp, heat_sink_temp);

    Serial.println("HEAT SINK TEMP");
    Serial.println(heat_sink_temp);

    if (heat_sink_temp < 45) {
    analogWrite(pwm_fan_pin, 0);
  } else if (heat_sink_temp < 65) {
    analogWrite(pwm_fan_pin, 255);  //100% is 255
  } else {
    analogWrite(pwm_fan_pin, (255 / ((0.05 * heat_sink_temp) - 2.25)));
  }

  }

  
  //READ THE ORION MESSAGE:
  if (msgReceived.can_id == ORION_TEMP_ID) {
    // battery_voltage = msgReceived.data[0];
    battery_temp = msgReceived.data[4];
  }
  if (battery_temp > 45) {
    batteryfail = 1;
  }

  //READ THE ORION MESSAGE: TEMPERATURE:
  if (msgReceived.can_id == ORION_VOLTAGE_ID) {
    lowcell_voltage = ( ((msgReceived.data[1] << 8) + msgReceived.data[0]) ) / 10000.0;
    highcell_voltage = ( ((msgReceived.data[3] << 8) + msgReceived.data[2]) ) / 10000.0;
  }
  if (highcell_voltage > 4.45) {
    batteryfail = 1;
  }
  if (lowcell_voltage < 2.75){
    batteryfail = 1;
  }
}




// void sendCAN(){
//   mcp2515.sendMessage(&DriverControls);
// }


void setDriverControls22() {
  DriverControls_WS22.can_id = DrC22_POTENTIOMETER_ID;
  DriverControls_WS22.can_dlc = 8;  //Length is 8 bytes of the Data Length Code

  int c = 0;
  // this is an attempt to clear the can message before trying to write something new on to it.
  for (int c = 0; c < 8; c++) {
    DriverControls_WS22.data[c] = 0x00;
  }

  //WRITE THE DESIRED SPEED AND CURRENT TO CAN
  // bit 00-31: RPM
  // bit 32-63: Motor set current (%)
  *(float*)(conv_bytes) = send_speed * 60 / (3.1415 * WHEEL_DIA);
  DriverControls_WS22.data[0] = conv_bytes[0];
  DriverControls_WS22.data[1] = conv_bytes[1];
  DriverControls_WS22.data[2] = conv_bytes[2];
  DriverControls_WS22.data[3] = conv_bytes[3];


  *(float*)(conv_bytes) = send_current_22;
  DriverControls_WS22.data[4] = conv_bytes[0];
  DriverControls_WS22.data[5] = conv_bytes[1];
  DriverControls_WS22.data[6] = conv_bytes[2];
  DriverControls_WS22.data[7] = conv_bytes[3];


  mcp2515.sendMessage(&DriverControls_WS22);

  busCurrent_WS22.can_id = DrC22_BUS_ID;
  busCurrent_WS22.can_dlc = 8;  //Length is 8 bytes of the Data Length Code

  // this is an attempt to clear the can message before trying to write something new on to it.
  for (int c = 0; c < 8; c++) {
    busCurrent_WS22.data[c] = 0x00;
  }

  *(float*)(conv_bytes) = maxBusCurrent;
  busCurrent_WS22.data[4] = conv_bytes[0];
  busCurrent_WS22.data[5] = conv_bytes[1];
  busCurrent_WS22.data[6] = conv_bytes[2];
  busCurrent_WS22.data[7] = conv_bytes[3];

  mcp2515.sendMessage(&busCurrent_WS22);

}

void setDriverControls20() {
  
  /////// TRITIUM 20 /////

  DriverControls_WS20.can_id = DrC20_POTENTIOMETER_ID;
  DriverControls_WS20.can_dlc = 8;  //Length is 8 bytes of the Data Length Code

  // this is an attempt to clear the can message before trying to write something new on to it.
  for (int c = 0; c < 8; c++) {
    DriverControls_WS20.data[c] = 0x00;
  }

  //WRITE THE DESIRED SPEED AND CURRENT TO CAN
  // bit 00-31: RPM
  // bit 32-63: Motor set current (%)
  *(float*)(conv_bytes) = send_speed * 60 / (3.1415 * WHEEL_DIA);
  DriverControls_WS20.data[0] = conv_bytes[0];
  DriverControls_WS20.data[1] = conv_bytes[1];
  DriverControls_WS20.data[2] = conv_bytes[2];
  DriverControls_WS20.data[3] = conv_bytes[3];


  *(float*)(conv_bytes) = send_current_20;
  DriverControls_WS20.data[4] = conv_bytes[0];
  DriverControls_WS20.data[5] = conv_bytes[1];
  DriverControls_WS20.data[6] = conv_bytes[2];
  DriverControls_WS20.data[7] = conv_bytes[3];


  mcp2515.sendMessage(&DriverControls_WS20);

  busCurrent_WS20.can_id = DrC20_BUS_ID;
  busCurrent_WS20.can_dlc = 8;  //Length is 8 bytes of the Data Length Code

  // this is an attempt to clear the can message before trying to write something new on to it.
  for (int c = 0; c < 8; c++) {
    busCurrent_WS20.data[c] = 0x00;
  }

  *(float*)(conv_bytes) = maxBusCurrent;
  busCurrent_WS20.data[4] = conv_bytes[0];
  busCurrent_WS20.data[5] = conv_bytes[1];
  busCurrent_WS20.data[6] = conv_bytes[2];
  busCurrent_WS20.data[7] = conv_bytes[3];

  mcp2515.sendMessage(&busCurrent_WS20);
}


void TelemCANMsg(){

  /////////// TELEM MESSAGES !!!


  TelemMessages.can_id = DrC_TELEM_ID;
  TelemMessages.can_dlc = 8;  //Length is 8 bytes of the Data Length Code

  // this is an attempt to clear the can message before trying to write something new on to it.
  for (int c = 0; c < 8; c++) {
    TelemMessages.data[c] = 0x00;
  }

  *(float*)(conv_bytes) = pot_current_reading; // sending driver set current over CAN to Telem
  TelemMessages.data[0] = conv_bytes[0];
  TelemMessages.data[1] = conv_bytes[1];
  TelemMessages.data[2] = conv_bytes[2];
  TelemMessages.data[3] = conv_bytes[3];


  *(float*)(conv_bytes) = TotalSendAmps; // Sending total amps between both motors over CAN to Telem
  TelemMessages.data[4] = conv_bytes[0];
  TelemMessages.data[5] = conv_bytes[1];
  TelemMessages.data[6] = conv_bytes[2];
  TelemMessages.data[7] = conv_bytes[3];

  mcp2515.sendMessage(&TelemMessages);
}







void dispSpeed() {
  // displays the set speed followed by a dot(postions 1 and 2). Then the actual speed (positions 3 and 4). Then the set current (positions 7 and 8)

  //The parameter region is 1~8
  LED.setBrightness(5);

  // Combine the values into one string
  char buffer[9];  // 8 digits + null terminator

  // //sprintf(buffer, "%02d.%02d%02d", val1, val2, val3);
  // if (pot_speed_reading_kmh > 100){
  //   pot_speed_reading_kmh = pot_speed_reading_kmh - 100;
  // }

  int current_disp = pot_current_reading * 100.0;
  int acc = 0;
  int acc_disp = pot_accelerator * 100.0;
  float actualspeedkmh = abs(actualspeed_ms) * 3.6;

  if (pot_accelerator >= ACC_THRESHOLD){ // Display acceleration percentage in middle if accelerating.
    sprintf(buffer, "%02d%02d%02d%02d", int(pot_speed_reading_kmh), int(actualspeedkmh), acc ,current_disp);
    LED.setDisplayArea(1,2,3,4,5,6,7,8);
  } else{
    if (digital_accel == 0){ 
      sprintf(buffer, "%02d%02d%02d%02d", int(pot_speed_reading_kmh), int(actualspeedkmh), acc ,current_disp);
      LED.setDisplayArea(1,2,3,4,5,6,7,8);
  } else { 
    LED.setDisplayArea(1,2,3,4,7,8);
    sprintf(buffer, "%02d%02d%02d", int(pot_speed_reading_kmh), int(actualspeedkmh), current_disp); 
  }}
  // Print the combined string to the display
  
  LED.print(atof(buffer));  // Use atof to convert the string to a float if needed

  // if (batteryfail == 1){
  //   LED.print("B","A","F","E"); // battery failiure 
  // }

}



float interpolate(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




//****************************************************************************************

//MAIN LOOP
int test_val = 0;
void loop() {

  // put your main code here, to run repeatedly:
  // timer.update();
  // timer2.update();
  // timer3.update();

  //Serial.println("RAW POT VALUES");
  //Serial.println(analogRead(CurrentPot));
  //Serial.println(analogRead(SpeedPot));

  pot_current_reading = analogRead(CurrentPot) / 1024.0;
  pot_speed_reading = analogRead(SpeedPot) / 1024.0;

  pot_accelerator = (1 - ( (analogRead(AcceleratorPot) ) / 1024.0)); // this is the new active current pot - the current pot on the dash is just the set limit
  // 1 minus value since the pot reads high when not turned
  pot_accelerator = (pot_accelerator - 0.11)/(0.26-0.11);

  raw_current = analogRead(CurrentPot);
  raw_speed = analogRead(SpeedPot);
  raw_accelerator_pot = analogRead(AcceleratorPot);
  
  Serial.println("RAW POT VALUES");
  Serial.println(raw_current);
  Serial.println(raw_speed);
  Serial.println(raw_accelerator_pot);

  pot_speed_reading_ms = pot_speed_reading * 30.0;
  pot_speed_reading_kmh = pot_speed_reading_ms * 3.6;


  TotalSendAmps = pot_current_reading * MaxTotalAmps; // e.g. if pot at 50%: 0.5 * 120 = 60A
  
  // // Excel spreadsheet blend: 

  // if (TotalSendAmps <= 10) {
  //   Marand_SendAmps = TotalSendAmps;
  //   Gerald_SendAmps = 0;
  // } else {
  //   if (TotalSendAmps >= 10 && TotalSendAmps <=30){
  //     Marand_SendAmps = interpolate(TotalSendAmps, 10.0, 30.0, 10.0, 20.0); // Use the custom interpolation function to map the current value (10 to 30) to a current (10 to 20)
  //     Gerald_SendAmps = interpolate(TotalSendAmps, 10.0, 30.0, 0.0, 10.0);
  //   }
  //   if (TotalSendAmps >= 30 && TotalSendAmps <=40){
  //     Marand_SendAmps = 20.0;
  //     Gerald_SendAmps = interpolate(TotalSendAmps, 30.0, 40.0, 10.0, 20.0);
  //   }
  //   if (TotalSendAmps >= 40 && TotalSendAmps <=60){
  //     Marand_SendAmps = interpolate(TotalSendAmps, 40.0, 60.0, 20.0, 40.0);
  //     Gerald_SendAmps = 20.0;
  //   }
  //   if (TotalSendAmps >= 60 && TotalSendAmps <=80){
  //     Marand_SendAmps = interpolate(TotalSendAmps, 60.0, 80.0, 40.0, 55.0);
  //     Gerald_SendAmps = interpolate(TotalSendAmps, 60.0, 80.0, 20.0, 25.0);
  //   }
  //   if (TotalSendAmps >= 80 && TotalSendAmps <=100){
  //     Marand_SendAmps = interpolate(TotalSendAmps, 80.0, 100.0, 55.0, 60.0);
  //     Gerald_SendAmps = interpolate(TotalSendAmps, 80.0, 100.0, 25.0, 40.0);
  //   }
  //   if (TotalSendAmps >= 100 && TotalSendAmps <=120){
  //     Marand_SendAmps = 60.0;
  //     Gerald_SendAmps = interpolate(TotalSendAmps, 100.0, 120.0, 40.0, 60.0);
  //   }
  // }

  /////////

  // //10A Gerald Marand difference

  // if (TotalSendAmps <= 10) {
  //   Marand_SendAmps = TotalSendAmps;
  //   Gerald_SendAmps = 0;
  // } else {
  //   if (TotalSendAmps >= 10){
  //     Gerald_SendAmps = (TotalSendAmps - 10)/2;
  //     Marand_SendAmps = TotalSendAmps - Gerald_SendAmps;
  //   }
  //   if (Marand_SendAmps >= MARAND_TRITIUM_AC_LIMIT){
  //     Marand_SendAmps = MARAND_TRITIUM_AC_LIMIT;
  //   }
  //   if (Gerald_SendAmps >= GERALD_TRITIUM_AC_LIMIT){
  //     Gerald_SendAmps = GERALD_TRITIUM_AC_LIMIT;
  //   }
  // }

  
  // Set Fraction % for Marand and Gerald

  Marand_SendAmps = TotalSendAmps * MARAND_FRACTION;
  Gerald_SendAmps = TotalSendAmps * GERALD_FRACTION;

  if (Marand_SendAmps >= MARAND_TRITIUM_AC_LIMIT){
    Marand_SendAmps = MARAND_TRITIUM_AC_LIMIT;
  }
  if (Gerald_SendAmps > GERALD_TRITIUM_AC_LIMIT) {
    Gerald_SendAmps = GERALD_TRITIUM_AC_LIMIT;
  }


  
  Serial.println("CURRENT POT VALUE");
  Serial.println(pot_current_reading);
  Serial.println("SENDING SPEED VALUE");
  Serial.println(send_speed);
  Serial.println("SENDING MARAND CURRENT VALUE");
  Serial.println(send_current_22);
  Serial.println("SENDING GERALD CURRENT VALUE");
  Serial.println(send_current_20);
  Serial.println("MOTOR SPEED:");
  Serial.println(actualspeedkmh);


  ignition = digitalRead(45);
  regen = digitalRead(43);
  rev = digitalRead(41);
  brake = digitalRead(37); // 39 is brake not pressed input
  digital_accel = digitalRead(33); // digital accel - just for backup

  //int ignition = 1; // setting the digital inputs to pins
  //int regen = 2;
  //int rev = 3;
  //int brake = 4;
  //int accel = 5;

  Serial.println("***************************************");
  Serial.println("Ignition");
  Serial.println(ignition);
  Serial.println("-----------------");
  Serial.println("Reverse");
  Serial.println(rev);
  Serial.println("-----------------");
  Serial.println("Forward");
  Serial.println(!rev);
  Serial.println("-----------------");
  Serial.println("Brake");
  Serial.println(brake);
  Serial.println("-----------------");
  Serial.println("Regen");
  Serial.println(regen);
  Serial.println("DIGITAL ACC");
  Serial.println(digital_accel);
  Serial.println("VARIABLE ACC");
  Serial.println(pot_accelerator);


  if (!digital_accel){
    which_accelerator_current22 = Gerald_SendAmps / GERALD_TRITIUM_AC_LIMIT;
    which_accelerator_current20 = Marand_SendAmps / MARAND_TRITIUM_AC_LIMIT;
    acceleration = 1;
  } else {
    if (pot_accelerator >= ACC_THRESHOLD && pot_accelerator <= 1.0 ){
    which_accelerator_current22 = (pot_accelerator * (Marand_SendAmps/MARAND_TRITIUM_AC_LIMIT)); // percentage on accelerator pot x original percentage
    which_accelerator_current20 = (pot_accelerator * (Gerald_SendAmps/GERALD_TRITIUM_AC_LIMIT));
    acceleration = 1;
  }
  else {
    acceleration = 0 ;
  }}
  

  

  if (!ignition) {
    // ignition = 0 (ignition on)

    if (!brake) {
      // brake = 0 (brake on)
      send_speed = 0.0;
      Serial.println("CAR IS BRAKING");
      if (!regen){
        // regen = 0 (regen on)
        send_current_20 = Gerald_SendAmps / GERALD_TRITIUM_AC_LIMIT;
        send_current_22 = Marand_SendAmps / MARAND_TRITIUM_AC_LIMIT;
        Serial.println("CAR IS REGENING");
      } else {
        //regen = 1 (regen off)
        send_current_20 = 0.0;
        send_current_22 = 0.0;
        Serial.println("CAR IS BRAKING");
      } // endif regen

    } else {
      // brake = 1 (not braking)
      Serial.println("CAR IS NOT BRAKING");

      if (acceleration){
        // accel = 0 (accelerating)
        Serial.println("CAR IS ACCELERATING");

        if (rev){ // reverse = 1
          send_speed = (-1) * pot_speed_reading_ms; //reverse
          send_current_20 = which_accelerator_current20;
          send_current_22 = which_accelerator_current22;
          Serial.println("CAR IS REVERSING");
        } else { // reverse = 0
          send_speed = pot_speed_reading_ms; //forwards
          send_current_20 = which_accelerator_current20;
          send_current_22 = which_accelerator_current22;
          Serial.println("CAR IS FORWARD DRIVING");
        }// endif reverse
        
      } else {
        send_current_22 = 0.0;
        send_current_20 = 0.0;
        Serial.println("CAR IS NOT ACCELERATING");
      } // endif digital acceleration

    } // endif brake

  } else {
    // ignition = 1 (ignition off)
    Serial.println("IGNITION OFF");
    send_speed = 0.0;
    send_current_20 = 0.0;
    send_current_22 = 0.0;
  }


  timer.update();
  timer2.update();
  timer3.update();
  timer4.update();
  timer5.update();


} // end main loop


