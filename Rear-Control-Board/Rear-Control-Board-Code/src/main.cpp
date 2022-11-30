/**
 * @file main.cpp
 * @author Dominic Gasperini - UVM '23
 * @brief this drives the rear control board on clean speed 5.5
 * @version 0.9
 * @date 2022-08-04
 */

// *** includes *** // 
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <mcp_can.h>
#include "pinConfig.h"
#include <ArduinoJson.h>



// *** defines *** // 
#define TIMER_INTERRUPT_PRESCALER       80          // this is based off to the clock speed (assuming 80 MHz), gets us to microseconds
#define SENSOR_POLL_INTERVAL            50000       // 0.05 seconds in microseconds
#define LOG_CAR_DATA_INTERVAL           500000      // 0.5 seconds in microseconds
#define CAN_WRITE_INTERVAL              100000      // 0.1 seconds in microseconds


// *** global variables *** //
// Drive Mode Enumeration Type
enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
};

// Car Data Struct
struct CarData
{
  struct DrivingData
  {
    bool readyToDrive = false;
    bool enableInverter = false;

    bool imdFault = false;
    bool bmsFault = false;

    uint16_t commandedTorque = 0;
    float currentSpeed = 0.0f;
    bool driveDirection = true;             // true = forward | false = reverse
    DriveModes driveMode = ECO;
  } drivingData;
  
  struct BatteryStatus
  {
    uint16_t batteryChargeState = 0;
    int16_t busVoltage = 0;
    int16_t rinehartVoltage = 0;
    float pack1Temp = 0.0f;
    float pack2Temp = 0.0f;
  } batteryStatus;

  struct Sensors
  {
    uint16_t wheelSpeedFR = 0;
    uint16_t wheelSpeedFL = 0;
    uint16_t wheelSpeedBR = 0;
    uint16_t wheelSpeedBL = 0;

    uint16_t wheelHeightFR = 0;
    uint16_t wheelHeightFL = 0;
    uint16_t wheelHeightBR = 0;
    uint16_t wheelHeightBL = 0;

    uint16_t steeringWheelAngle = 0;
  } sensors;

  struct Inputs
  {
    uint16_t pedal0 = 0;
    uint16_t pedal1 = 0;
    uint16_t brake0 = 0;
    uint16_t brake1 = 0;
    uint16_t brakeRegen = 0;
    uint16_t coastRegen = 0;

    float vicoreTemp = 0.0f;
    float pumpTempIn = 0.0f;
    float pimpTempOut = 0.0f;
  } inputs;

  struct Outputs
  {
    bool buzzerActive = false;
    uint8_t buzzerCounter = 0;
    bool brakeLight = false;
  } outputs;
  
};
CarData carData;

// ESP-Now Connection
uint8_t wcbAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info wcbInfo;


// CAN
MCP_CAN CAN0(10);       // set CS pin to 10

// Hardware ISR Timers
hw_timer_t* timer0 = NULL;
hw_timer_t* timer1 = NULL;
hw_timer_t* timer2 = NULL;
hw_timer_t* timer3 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// *** function declarations *** //
void PollSensorData();
void CANRead();
void CANWrite();
void LogCarData();


// *** setup *** //
void setup()
{
  // --- initialize serial --- //
  Serial.begin(9600);

  // --- initialize sensors --- //
  pinMode(WHEEL_SPEED_BR_SENSOR, INPUT);
  pinMode(WHEEL_SPEED_BL_SENSOR, INPUT);
  
  pinMode(WHEEL_HEIGHT_BR_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_BL_SENSOR, INPUT);
  
  pinMode(WATER_TMP_IN_PIN, INPUT);
  pinMode(WATER_TMP_OUT_PIN, INPUT);

  // --- initialize inputs --- //

  pinMode(CAN_MESSAGE_INTERRUPT_PIN, INPUT);
  pinMode(VICORE_PIN, INPUT);
  

  // --- initalize outputs --- //
  pinMode(PUMP_ENABLE_PIN, OUTPUT);
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(FAN_ENABLE_PIN, OUTPUT); 
  pinMode(IMD_FAULT_PIN, OUTPUT);  
  pinMode(BMS_FAULT_PIN, OUTPUT);


  // --- initalize CAN --- //
  pinMode(CAN_MESSAGE_INTERRUPT_PIN, INPUT);
  // create interrupt when the message arrived pin is pulled low
  attachInterrupt(CAN_MESSAGE_INTERRUPT_PIN, CANRead, LOW);
  
  // init CAN
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("CAN INIT [ SUCCESS ]");
  else
    Serial.println("CAN INIT [ FAILED ]");
  
  // set mode to read and write
  CAN0.setMode(MCP_NORMAL);

  // setup mask and filter
  CAN0.init_Mask(0, 0, 0xFFFFFFFF);       // check all ID bits, excludes everything except select IDs
  CAN0.init_Filt(0, 0, 0x100);            // RCB ID 1
  CAN0.init_Filt(1, 0, 0x101);            // RCB ID 2
  CAN0.init_Filt(2, 0, 0x102);            // Rinehart ID


  // --- initialize ESP-NOW ---//
  // turn on wifi access point 
  WiFi.mode(WIFI_STA);

  // init ESP-NOW service
  if (esp_now_init() != ESP_OK)
    Serial.println("ESP-NOW INIT [ SUCCESS ]");
  else
    Serial.println("ESP-NOW INIT [ FAILED ]");
  
  // TODO: finish this


  // --- initialize timer interrupts --- //
  timer0 = timerBegin(0, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer0, &PollSensorData, true);
  timerAlarmWrite(timer0, SENSOR_POLL_INTERVAL, true);
  timerAlarmEnable(timer0);

  timer1 = timerBegin(1, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer1, &CANWrite, true);
  timerAlarmWrite(timer1, CAN_WRITE_INTERVAL, true);
  timerAlarmEnable(timer1);

  timer2 = timerBegin(2, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer2, &LogCarData, true);
  timerAlarmWrite(timer2, LOG_CAR_DATA_INTERVAL, true);
  timerAlarmEnable(timer2);


  // --- End Setup Section in Serial Monitor --- //
  Serial.print("|--- END SETUP ---|\n\n\n");
}

// *** loop *** // 
void loop()
{
  // everything is on a timer so only brake light happens here, since it is relatively low-priority! 


  // write brake light status
  analogWrite(BRAKE_LIGHT_PIN, (carData.inputs.brake0 || carData.inputs.brake1));
}


/**
 * @brief Interrupt Handler for Timer 0
 * This ISR is for reading sensor data from the car 
 */
void PollSensorData()
{

  // what pins?
  // vicoretemp
  // pumptempin
  // pumptempout


  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // turn off wifi for ADC channel 2 to function
  WiFi.mode(WIFI_OFF);


  // TODO: gather data from sensors and store in car data struct
  carData.inputs.vicoreTemp = analogRead(VICORE_PIN);
  carData.inputs.pumpTempIn = analogRead(WATER_TMP_IN_PIN);
  carData.inputs.pimpTempOut = analogRead(WATER_TMP_OUT_PIN);



  // turn wifi back on to re-enable esp-now connection to wheel board
  WiFi.mode(WIFI_STA);

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Interrupt Handler for on CAN Message Received
 * This ISR is for reading an incoming message from the CAN bus
 */
void CANRead()
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  unsigned long receivingID = 0;
  byte messageLength = 0;
  byte receivingBuffer[8];

  // read the message
  CAN0.readMsgBuf(&receivingID, &messageLength, receivingBuffer);

  // filter for only the IDs we are interested in
  switch (receivingID)
  {
    // message from RCB: Sensor Data
    case 0x100:
    // do stuff with the data in the message
    break;
    // message from RCB: BMS and electrical data
    case 0x101:
    // do stuff with the data in the message
    break;

    default:
    // do nothing because we didn't get any messages of interest
    return;
    break;
  }

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Interrupt Handler for Timer 1
 * This ISR is for reading and writing to the CAN bus
 */
void CANWrite()
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  byte outgoingMessage[8];

  // build message
  outgoingMessage[0] = 0x00;
  outgoingMessage[1] = 0x01;
  outgoingMessage[2] = 0x02;
  outgoingMessage[3] = 0x03;
  outgoingMessage[4] = 0x04;
  outgoingMessage[5] = 0x05;
  outgoingMessage[6] = 0x06;
  outgoingMessage[7] = 0x07;

  // send the message and get its sent status
  byte sentStatus = CAN0.sendMsgBuf(0x100, 0, sizeof(outgoingMessage), outgoingMessage);    // (sender address, STD CAN frame, size of message, message)

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Interrupt Handler for Timer 3
 * log car data frames to the SD card
 */
void LogCarData()
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);


  // TODO: write data frame to SD card
  DynamicJsonDocument dataFrame(1024);

  dataFrame["time"] = esp_timer_get_time();

  // --- INPUTS --- //

  dataFrame["DrivingData"]["readyToDrive"] = carData.drivingData.readyToDrive;
  dataFrame["DrivingData"]["enableInverter"] = carData.drivingData.enableInverter;
  
  dataFrame["DrivingData"]["imdFault"] = carData.drivingData.imdFault;
  dataFrame["DrivingData"]["bmsFault"] = carData.drivingData.bmsFault;

  dataFrame["DrivingData"]["commandedTorque"] = carData.drivingData.commandedTorque;
  dataFrame["DrivingData"]["currentSpeed"] = carData.drivingData.currentSpeed;
  dataFrame["DrivingData"]["driveDirection"] = carData.drivingData.driveDirection;
  dataFrame["DrivingData"]["driveMode"] = carData.drivingData.driveMode;
  
  // --- SENSORS --- //

  dataFrame["Sensors"]["wheels"]["speed"]["FR"] = carData.sensors.wheelHeightFR;
  dataFrame["Sensors"]["wheels"]["speed"]["FL"] = carData.sensors.wheelHeightFL;
  dataFrame["Sensors"]["wheels"]["speed"]["BR"] = carData.sensors.wheelHeightBR;
  dataFrame["Sensors"]["wheels"]["speed"]["BL"] = carData.sensors.wheelHeightBL;

  dataFrame["Sensors"]["wheels"]["height"]["FR"] = carData.sensors.wheelHeightFR;
  dataFrame["Sensors"]["wheels"]["height"]["FL"] = carData.sensors.wheelHeightFL;
  dataFrame["Sensors"]["wheels"]["height"]["BR"] = carData.sensors.wheelHeightBR;
  dataFrame["Sensors"]["wheels"]["height"]["BL"] = carData.sensors.wheelHeightBL;

  dataFrame["Sensors"]["steeringWheelAngle"] = carData.sensors.steeringWheelAngle;

  // --- INPUTS --- //

  dataFrame["Inputs"]["pedal0"] = carData.inputs.pedal0;
  dataFrame["Inputs"]["pedal1"] = carData.inputs.pedal1;

  dataFrame["Inputs"]["brake0"] = carData.inputs.brake0;
  dataFrame["Inputs"]["brake1"] = carData.inputs.brake1;

  dataFrame["Inputs"]["brakeRegen"] = carData.inputs.brakeRegen;
  dataFrame["Inputs"]["coastRegen"] = carData.inputs.coastRegen;

  dataFrame["Inputs"]["vicoreTemp"] = carData.inputs.vicoreTemp;
  dataFrame["Inputs"]["pumpTempIn"] = carData.inputs.pumpTempIn;
  dataFrame["Inputs"]["pimpTempOut"] = carData.inputs.pimpTempOut;

  // --- OUTPUTS --- //

  dataFrame["Outputs"]["buzzerActive"] = carData.outputs.buzzerActive;
  dataFrame["Outputs"]["buzzerCounter"] = carData.outputs.buzzerCounter;
  dataFrame["Outputs"]["brakeLight"] = carData.outputs.brakeLight;


  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}
