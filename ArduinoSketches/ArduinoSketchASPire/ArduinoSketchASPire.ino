/*Purpose: Main Arduino file for the actuator unit. 
 *         Used to control the actuators and send feedback to the navigation unit. 
 *         Uses CAN-bus to receive and send data. 
 *         
 *         Notes:
 *                Messages can only be of 8 bytes!
 *                Little endian - big endian problems may arise if using non-standard int types
 *                Improper alimentation may lead to trasmission errors
 *                Configuration macros are to be stored in the config.h file for conveniency and maintainability
 *                Watchdog prevents uC halting problems due to possible invalid Can frames being received
 * 
 */
#include <SoftwareSerial.h>
#include <PololuMaestro.h>
#include <Canbus.h>
#include <MsgParsing.h>
#include <avr/wdt.h>
#include "config.h"

/* On boards with a hardware serial port available for use, use
that port to communicate with the Maestro. For other boards,
create a SoftwareSerial object using pin 10 to receive (RX) and
pin 11 to transmit (TX). */
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else

  SoftwareSerial maestroSerial(19, 18); //10, 11 for UNO ||19, 18 for MEGA
#endif

// Used for current sensors
enum SensedElement : uint8_t { SAILDRIVE, WINDVANE_SWITCH, WINDVANE_ANGLE, ACTUATOR_UNIT };
static int index = 0;

/* Next, create a Maestro object using the serial port.
Uncomment one of MicroMaestro or MiniMaestro below depending
on which one you have. */
MicroMaestro maestro(maestroSerial);
//MiniMaestro maestro(maestroSerial);

CanbusClass Canbus;

void setup()
{
  pinMode(RUDDER_FEEDBACK_PIN, INPUT);
  pinMode(WINGSAIL_FEEDBACK_PIN, INPUT);
  pinMode (RADIO_CONTROLL_OFF_PIN, INPUT);
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  Serial.begin(9600);

  if(Canbus.Init(0)) {
    Serial.println("CAN bus initialized.");
  }
  // Enable the watchdog
  wdt_enable(WDTO_4S);
 
  Serial.println("SETUP COMPLETE");
}

void loop()
{
  sendArduinoData ();
  checkCanbusFor (50);
  wdt_reset();
  
  sendCurrentSensorData ();
  checkCanbusFor (50);
  wdt_reset();
  
  sendFeedback ();
  checkCanbusFor (400);
  wdt_reset();
}

///// BUG: why not use build-in map() function?
float mapInterval(float val, float fromMin, float fromMax, float toMin, float toMax) {
  return (val - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin;
}

void sendFeedback (){
 CanMsg feedbackMsg;
 uint16_t rudderAngle16;
 uint16_t wingsailAngle16;
 
      
      rudderAngle16 = getRudderFeedback ();
      wingsailAngle16 = getWingsailFeedback();

      feedbackMsg.id = 701;
      feedbackMsg.header.ide = 0;
      feedbackMsg.header.length = 7;  
      
      feedbackMsg.data[0] = (rudderAngle16 & 0xff);
      feedbackMsg.data[1] = (rudderAngle16 >> 8);
      feedbackMsg.data[2] = (wingsailAngle16 & 0xff);
      feedbackMsg.data[3] = (wingsailAngle16 >> 8);
      feedbackMsg.data[4] = 0;
      feedbackMsg.data[5] = 0;
      feedbackMsg.data[6] = 0;
      
      Canbus.SendMessage(&feedbackMsg);
      
}

void sendArduinoData (){
  CanMsg arduinoData;
    arduinoData.id = 702;
    arduinoData.header.ide = 0;
    arduinoData.header.length = 7;
    uint16_t RCon16 = (uint16_t) isRadioControllerUsed ();
    arduinoData.data[0] = (RCon16 & 0xff);
    arduinoData.data[1] = (RCon16 >> 8);
    arduinoData.data[2] = 0;
    arduinoData.data[3] = 0;
    arduinoData.data[4] = 0;
    arduinoData.data[5] = 0;
    arduinoData.data[6] = 0;
    
    Canbus.SendMessage(&arduinoData);   

}

void sendCurrentSensorData(){ // in mV and mA stored in two 16 bit integer vars
  uint8_t unit_type;
  uint16_t voltage_raw = 0, current_raw = 0;
  uint16_t voltage_val = 0, current_val = 0;

  // TODO: refactor redundant code

  switch(index)
  {
  /* 
   *  Read AttoPilot sensors
   *  
   *  Developer notes: possible update using internal analog reference at 1.1V
   */

    case 0:
      // ------------------- Measurement for the saildrive -------------------    
      for(int i = 0; i < SAMPLES_N; i++)
      {
        voltage_raw += analogRead(SAILDRIVE_VOLTAGE_PIN);
        current_raw += analogRead(SAILDRIVE_CURRENT_PIN);
        delay(2);
      }
    
      voltage_raw /= SAMPLES_N;
      current_raw /= SAMPLES_N;
      
      // Conversion
      voltage_val = round((voltage_raw/49.44)*1000); //45 Amp board  
      current_val = round((current_raw/14.9)*1000); //45 Amp board
      unit_type = SAILDRIVE;
    break;

    case 1:
    
      // ------------------- Measurement for the actuator unit -------------------
      for(int i = 0; i < SAMPLES_N; i++)
      {
        voltage_raw += analogRead(ACTUATOR_UNIT_VOLTAGE_PIN);
        current_raw += analogRead(ACTUATOR_UNIT_CURRRENT_PIN);
        delay(2);
      }
    
      voltage_raw /= SAMPLES_N;
      current_raw /= SAMPLES_N;
      
      // Conversion
      voltage_val = round((voltage_raw/49.44)*1000);
      current_val = round((current_raw/14.9)*1000);
      unit_type = ACTUATOR_UNIT;
    break;

  /*
   * Read ACS712 sensors
   */

   case 2:
   
      // ------------------- Measurement for the windvane switch -------------------    
      for(int i = 0; i < SAMPLES_N; i++)
      {
        voltage_raw += analogRead(WINDVANE_SWITCH_VOLTAGE_PIN);
      }
    
      voltage_raw /= SAMPLES_N;
      
      // Conversion
      // The on-board ADC is 10-bits -> 2^10 = 1024 -> 5V / 1024 ~= 4.88mV
      voltage_val = round(voltage_raw * 4.88); 
      current_val = round((voltage_val - V_REF) / ACS712_MV_PER_AMP);
      unit_type = WINDVANE_SWITCH;
    break;

    case 3:
    
      // ------------------- Measurement for the windvane angle -------------------    
      for(int i = 0; i < SAMPLES_N; i++)
      {
        voltage_raw += analogRead(WINDVANE_ANGLE_VOLTAGE_PIN);
      }
    
      voltage_raw /= SAMPLES_N;
      
      voltage_val = round(voltage_raw * 4.88); 
      current_val = round((voltage_val - V_REF) / ACS712_MV_PER_AMP);
      unit_type = WINDVANE_ANGLE;
    break;
  }

  // Increment index
  if(index > 3)
    index = 0;
  else
    index++;

  // Craft CAN message
  CanMsg currentSensorData;
    currentSensorData.id = 705;
    currentSensorData.header.ide = 0;
    currentSensorData.header.length = 7;

    currentSensorData.data[0] = (voltage_val & 0xff);
    currentSensorData.data[1] = (voltage_val >> 8);
    currentSensorData.data[2] = (current_val & 0xff);
    currentSensorData.data[3] = (current_val >> 8);
    currentSensorData.data[4] = unit_type;
    currentSensorData.data[5] = 0;
    currentSensorData.data[6] = 0;

  Canbus.SendMessage(&currentSensorData);
}

void checkCanbusFor (int timeMs){
  int startTime= millis();
  int timer = 0;
  
  while (timer < timeMs){
    if (Canbus.CheckForMessages()) {
      CanMsg msg;
      Canbus.GetMessage(&msg);
      processCANMessage (msg);
    }
    timer = millis() - startTime;
  }
}

uint16_t getRudderFeedback() {
  int feedback = analogRead(RUDDER_FEEDBACK_PIN);

//Variabales c, b1 and b2 come from formula to map to a squareroot function. Reference in Hardware documatation
  float c = -361.0000;
  float b1 =  1.8823;
  float b2 = -1.8473;
  float angle;
  if (feedback < -c){
    
    angle = b1* sqrt (-(feedback+c));
  } else {
    angle = b2* sqrt (feedback+c);
  }

  uint16_t canbusAngle = mapInterval (angle, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 0, INT16_SIZE);
  return canbusAngle;
  
}

float getWingsailFeedback() {
  int feedback = analogRead(WINGSAIL_FEEDBACK_PIN);

  //Variabales c, b1 and b2 come from formula to map to a squareroot function. Reference in Hardware documatation
  float c = -450;
  float b1 = 0.7098;
  float b2 = -0.6455;
  float angle;
  if (feedback < -c){
    
    angle = b1* sqrt (-(feedback+c));
  } else {
    angle = b2* sqrt (feedback+c);
  }
  uint16_t canbusAngle = mapInterval (angle, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE, 0, INT16_SIZE);
  return canbusAngle; 
}

int isRadioControllerUsed (){
  
  if (analogRead (RADIO_CONTROLL_OFF_PIN) > 250) { //Value comes from the multiplexer indicator led
    return 0;
  }
  else {
    return  INT16_SIZE/2;
  }
}
   

void processCANMessage (CanMsg& msg){
    
        if(msg.id == 700) {
          uint16_t rawCanData = (msg.data[1]<<8 | msg.data[0]);
          double rudderAngel = mapInterval (rawCanData, 0, INT16_SIZE, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
          //Serial.print("Received rudder angle: "); Serial.println(rudderAngel);
          rawCanData = (msg.data[3]<<8 | msg.data[2]);
          double wingsailAngle = mapInterval (rawCanData, 0, INT16_SIZE, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);
          //Serial.print("Received wingsail angle: "); Serial.println(wingsailAngle);
          
          moveRudder(rudderAngel);
          
          moveWingsail(wingsailAngle);      
             
      }
}

void moveRudder(double angleToSet) {
  float target = mapInterval(angleToSet, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 
                      RUDDER_MAESTRO_MIN_TARGET, RUDDER_MAESTRO_MAX_TARGET);
  
  maestro.setTarget(RUDDER_MAESTRO_CHANNEL, target*MAESTRO_SIGNAL_MULTIPLIER);
  maestro.getErrors(); //Used to clear any errors on the maestro inorder for it not to lock up
}

void moveWingsail(double angleToSet) {
  float target = mapInterval(angleToSet, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE,
                  WINGSAIL_MAESTRO_MIN_TARGET, WINGSAIL_MAESTRO_MAX_TARGET);
  maestro.setTarget(WINGSAIL_MAESTRO_CHANNEL, target*MAESTRO_SIGNAL_MULTIPLIER);  
  maestro.getErrors(); //Used to clear any errors on the maestro inorder for it not to lock up
}



 


