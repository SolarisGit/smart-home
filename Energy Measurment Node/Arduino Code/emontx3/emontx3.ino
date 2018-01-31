
/*Recommended node ID allocation
------------------------------------------------------------------------------------------------------------
-ID-	-Node Type-
0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
1-4     - Control nodes
5-10	- Energy monitoring nodes
11-14	--Un-assigned --
15-16	- Base Station & logging nodes
17-30	- Environmental sensing nodes (temperature humidity etc.)
31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
-------------------------------------------------------------------------------------------------------------

emonhub.conf node decoder (nodeid is 8 when switch is off, 7 when switch is on)
See: https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md

[[8]]
    nodename = emonTx_3
    firmware =V2_3_emonTxV3_4_DiscreteSampling
    hardware = emonTx_(NodeID_DIP_Switch1:OFF)
    [[[rx]]]
       names = power1, power2, power3, power4, Vrms, temp1, temp2, temp3, temp4, temp5, temp6, pulse
       datacodes = h,h,h,h,h,h,h,h,h,h,h,L
       scales = 1,1,1,1,0.01,0.1,0.1, 0.1,0.1,0.1,0.1,1
       units =W,W,W,W,V,C,C,C,C,C,C,p

*/

#define emonTxV3                                                                          // Tell emonLib this is the emonTx V3 - don't read Vcc assume Vcc = 3.3V as is always the case on emonTx V3 eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/
#define RF69_COMPAT 0                                                              // Set to 1 if using RFM69CW or 0 if using RFM12B
#include <JeeLib.h>  //https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14
#include "EmonLib.h"                                                                    // Include EmonLib energy monitoring library https://github.com/openenergymonitor/EmonLib

EnergyMonitor ct1, ct2, ct3, ct4, ct5, ct6, ct7;

const byte version = 29;      
boolean DEBUG = 1;                       // Print serial debug

//----------------------------emonTx V3 Settings---------------------------------------------------------------------------------------------------------------
byte Vrms=                        230;            // Vrms for apparent power readings (when no AC-AC voltage sample is present)
const byte Vrms_USA=              120;            // VRMS for USA apparent power
const byte TIME_BETWEEN_READINGS = 10;            //Time between readings

//http://openenergymonitor.org/emon/buildingblocks/calibration

const float Ical1=                15;                                 // 15A/1V
const float Ical2=                15;                                 // 15A/1V
const float Ical3=                15;                                 // 15A/1V
const float Ical4=                15;                                 // 15A/1V
const float Ical5=                15;                                 // 15A/1V
const float Ical6=                15;                                 // 15A/1V
const float Ical7=                15;                                 // 15A/1V

float Vcal=                       238;                             // (231V/12.4V) * (34,2Vpp ACAC / 2,68Vpp en uC) = 237,7 Calibration for EU AC-AC adapter Model DE-06-09

const float phase_shift=          1.7;
const int no_of_samples=          1662;
const int no_of_half_wavelengths= 30;
const int timeout=                2000;                               //emonLib timeout
const int ACAC_DETECTION_LEVEL=   3000;
const byte min_pulsewidth= 110;                                // minimum width of interrupt pulse (default pulse output meters = 100ms)


//----------------------------emonTx V3 hard-wired connections---------------------------------------------------------------------------------------------------------------
const byte LEDpin=                   8;                              // emonTx V3 LED
const byte RFM12pin_enable=          10;                             // enable RFM12

//-----------------------RFM12B / RFM69CW SETTINGS----------------------------------------------------------------------------------------------------
byte RF_freq=RF12_868MHZ;                                           // Frequency of RF69CW module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
byte nodeID = 3;                                                    // emonTx RFM12B node ID
int networkGroup = 7;
boolean RF_STATUS = 1;                                              // Enable RF

// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
typedef struct {
  int power1, power2, power3, power4, power5, power6, power7, Vrms;
} PayloadTX;     

PayloadTX emontx;

//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------

//Random Variables
boolean CT1, CT2, CT3, CT4,CT5, CT6, CT7, ACAC;
byte CT_count=0;
unsigned long start=0;

const char helpText1[] PROGMEM =                                 // Available Serial Commands
"\n"
"Available commands:\n"
"  <nn> i     - set node IDs (standard node ids are 1..30)\n"
"  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
"  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)\n"
"  s          - save config to EEPROM\n"
"  v          - Show firmware version\n"
;


#ifndef UNIT_TEST  // IMPORTANT LINE!
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
//SETUP
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  pinMode(LEDpin, OUTPUT);

  // Enable the RF module
  pinMode(RFM12pin_enable,OUTPUT);
  digitalWrite(RFM12pin_enable,LOW);

  digitalWrite(LEDpin,HIGH);

  Serial.begin(115200);
  Serial.print("emonTx V3.4 Discrete Sampling V"); Serial.println(version*0.1);
  Serial.println(" ");
   
  if (RF_STATUS==1){
    load_config();                                                        // Load RF config from EEPROM (if any exists)
    #if (RF69_COMPAT)
       Serial.print("RFM69CW");
    #else
      Serial.print("RFM12B");
    #endif
    Serial.print(" Node: "); Serial.print(nodeID);
    Serial.print(" Freq: ");
    if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
    if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
    if (RF_freq == RF12_915MHZ) Serial.print("915Mhz");
    Serial.print(" Group: "); Serial.println(networkGroup);
    Serial.println(" ");
  }
  
  Serial.println("POST.....wait 10s");
  Serial.println("'+++' then [Enter] for RF config mode");
  
  delay(20);
  
  if (RF_STATUS==1){
    Serial.println("r12_initialization");
    rf12_initialize(nodeID, RF_freq, networkGroup);                 
    Serial.println(">>1");
/* 
    for (int i=10; i>=0; i--)                                               // Send RF test sequence (for factory testing)
    {
      emontx.power1=i;
      rf12_sendNow(0, &emontx, sizeof emontx);
      delay(100);
    }
    rf12_sendWait(2);
    emontx.power1=0;
*/  
  }

  Serial.println("Cheking CT sensors");  
  if (analogRead(1) > 0) {CT1 = 1; CT_count++;} else CT1=0;              // check to see if CT is connected to CT1 input, if so enable that channel
  if (analogRead(2) > 0) {CT2 = 1; CT_count++;} else CT2=0;              // check to see if CT is connected to CT2 input, if so enable that channel
  if (analogRead(3) > 0) {CT3 = 1; CT_count++;} else CT3=0;              // check to see if CT is connected to CT3 input, if so enable that channel
  if (analogRead(4) > 0) {CT4 = 1; CT_count++;} else CT4=0;              // check to see if CT is connected to CT4 input, if so enable that channel
  if (analogRead(5) > 0) {CT5 = 1; CT_count++;} else CT5=0;              // check to see if CT is connected to CT5 input, if so enable that channel
  if (analogRead(6) > 0) {CT6 = 1; CT_count++;} else CT6=0;              // check to see if CT is connected to CT6 input, if so enable that channel
  if (analogRead(7) > 0) {CT7 = 1; CT_count++;} else CT7=0;              // check to see if CT is connected to CT7 input, if so enable that channel

  if ( CT_count == 0) CT1=1;                                             // If no CT's are connect ed CT1-7 then by default read from CT1

  // Quick check to see if there is a voltage waveform present on the ACAC Voltage input
  // Check consists of calculating the RMS from 100 samples of the voltage input.
  start = millis();
  while (millis() < (start + 10000)){
    // If serial input of keyword string '+++' is entered during 10s POST then enter config mode
    if (Serial.available()){
      if ( Serial.readString() == "+++\r\n"){
        Serial.println("Entering config mode...");
        showString(helpText1);
        // char c[]="v"
        config(char('v'));
        while(1){
          if (Serial.available()){
            config(Serial.read());
          }
        }
      }
    }
  }

  Serial.println("r12_config time out");
  digitalWrite(LEDpin,LOW);

  // Calculate if there is an ACAC adapter on analog input 0
  double vrms = calc_rms(0,1780) * (Vcal * (3.3/1024) );
  if (vrms>90) ACAC = 1; else ACAC=0;

  if (ACAC)
  {
    for (int i=0; i<10; i++)                                              // indicate AC has been detected by flashing LED 10 times
    {
      digitalWrite(LEDpin, HIGH); delay(200);
      digitalWrite(LEDpin, LOW); delay(300);
    }
  }
  else
  {
    delay(1000);
    digitalWrite(LEDpin, HIGH); delay(2000); digitalWrite(LEDpin, LOW);   // indicate DC power has been detected by turing LED on then off
  }

  if (DEBUG==1)
  {
    Serial.print("CT 1 Cal "); Serial.println(Ical1);
    Serial.print("CT 2 Cal "); Serial.println(Ical2);
    Serial.print("CT 3 Cal "); Serial.println(Ical3);
    Serial.print("CT 4 Cal "); Serial.println(Ical4);
    Serial.print("CT 5 Cal "); Serial.println(Ical5);
    Serial.print("CT 6 Cal "); Serial.println(Ical6);
    Serial.print("CT 7 Cal "); Serial.println(Ical7);
    delay(1000);

    Serial.print("RMS Voltage on AC-AC  is: ~");
    Serial.print(vrms,0); Serial.println("V");

    if (ACAC) {
      Serial.println("AC-AC detected - Real Power measure enabled");
      Serial.println("assuming pwr from AC-AC (jumper closed)");
      Serial.print("Vcal: "); Serial.println(Vcal);
      Serial.print("Phase Shift: "); Serial.println(phase_shift);
    } else {
      Serial.println("AC-AC NOT detected - Apparent Pwr measure enabled");
      Serial.print("Assuming VRMS: "); Serial.print(Vrms); Serial.println("V");
      Serial.println("Assuming power from batt / 5V USB - power save enabled");
    }

    if (CT_count==0) {
      Serial.println("NO CT's detected");
    } else {
      if (CT1) Serial.println("CT 1 detected");
      if (CT2) Serial.println("CT 2 detected");
      if (CT3) Serial.println("CT 3 detected");
      if (CT4) Serial.println("CT 4 detected");
      if (CT5) Serial.println("CT 5 detected");
      if (CT6) Serial.println("CT 6 detected");
      if (CT7) Serial.println("CT 7 detected");
    }
    
    delay(500);

  }
  else
  {
    Serial.end();
  }

  if (CT1) ct1.current(1, Ical1);             // CT ADC channel 1, calibration.  
  if (CT2) ct2.current(2, Ical2);             // CT ADC channel 2, calibration.
  if (CT3) ct3.current(3, Ical3);             // CT ADC channel 3, calibration.
  if (CT4) ct4.current(4, Ical4);             // CT ADC channel 4, calibration.
  if (CT5) ct5.current(5, Ical5);             // CT ADC channel 5, calibration.
  if (CT6) ct6.current(6, Ical6);             // CT ADC channel 6, calibration.
  if (CT7) ct7.current(7, Ical7);             // CT ADC channel 7, calibration.  

  if (ACAC)
  {
    if (CT1) ct1.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT2) ct2.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT3) ct3.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT4) ct4.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT5) ct5.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT6) ct6.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT7) ct7.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
  }

} //end SETUP

//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
// LOOP
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
  start = millis();

  if (ACAC) {
    delay(200);                         //if powering from AC-AC allow time for power supply to settle
    emontx.Vrms=0;                      //Set Vrms to zero, this will be overwirtten by CT 1-4
  }

  // emontx.power1 = 1;
  // emontx.power2 = 1;
  // emontx.power3 = 1;
  // emontx.power4 = 1;

  if (CT1) {
    if (ACAC) {
      ct1.calcVI(no_of_half_wavelengths,timeout); emontx.power1=ct1.realPower;
      emontx.Vrms=ct1.Vrms*100;
    } else {
      emontx.power1 = ct1.calcIrms(no_of_samples)*Vrms;
    }
  }

  if (CT2) {
    if (ACAC) {
      ct2.calcVI(no_of_half_wavelengths,timeout); emontx.power2=ct2.realPower;
      emontx.Vrms=ct2.Vrms*100;
    } else {
      emontx.power2 = ct2.calcIrms(no_of_samples)*Vrms;
    }
  }

  if (CT3) {
    if (ACAC) {
      ct3.calcVI(no_of_half_wavelengths,timeout); emontx.power3=ct3.realPower;
      emontx.Vrms=ct3.Vrms*100;
    } else {
      emontx.power3 = ct3.calcIrms(no_of_samples)*Vrms;
    }
  }

  if (CT4) {
    if (ACAC) {
      ct4.calcVI(no_of_half_wavelengths,timeout); emontx.power4=ct4.realPower;
      emontx.Vrms=ct4.Vrms*100;
    } else {
      emontx.power4 = ct4.calcIrms(no_of_samples)*Vrms;
    }
  }

  if (CT5) {
    if (ACAC) {
      ct5.calcVI(no_of_half_wavelengths,timeout); emontx.power5=ct5.realPower;
      emontx.Vrms=ct5.Vrms*100;
    } else {
      emontx.power5 = ct5.calcIrms(no_of_samples)*Vrms;
    }
  }

  if (CT6) {
    if (ACAC) {
      ct6.calcVI(no_of_half_wavelengths,timeout); emontx.power6=ct6.realPower;
      emontx.Vrms=ct6.Vrms*100;
    } else {
      emontx.power6 = ct6.calcIrms(no_of_samples)*Vrms;
    }
  }

  if (CT7) {
    if (ACAC) {
      ct7.calcVI(no_of_half_wavelengths,timeout); emontx.power7=ct7.realPower;
      emontx.Vrms=ct7.Vrms*100;
    } else {
      emontx.power7 = ct7.calcIrms(no_of_samples)*Vrms;
    }
  }

  if(DEBUG==1){
    Serial.print("ct1:");  Serial.print(emontx.power1); 
    Serial.print(",ct2:"); Serial.print(emontx.power2);
    Serial.print(",ct3:"); Serial.print(emontx.power3);
    Serial.print(",ct4:"); Serial.print(emontx.power4);
    Serial.print(",ct5:"); Serial.print(emontx.power5);
    Serial.print(",ct6:"); Serial.print(emontx.power6);
    Serial.print(",ct7:"); Serial.print(emontx.power7);
    Serial.print(",vrms:"); Serial.print(emontx.Vrms);
  }
  Serial.println();
  delay(50);
  

  if (ACAC) {digitalWrite(LEDpin, HIGH); delay(200); digitalWrite(LEDpin, LOW);}    // flash LED if powered by AC

  if (RF_STATUS==1){
    send_rf_data();                                                           // *SEND RF DATA* - see emontx_lib
  }

  unsigned long runtime = millis() - start;
  unsigned long sleeptime = (TIME_BETWEEN_READINGS*1000) - runtime - 100;

  delay(sleeptime);
 
} // end loop
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
// SEND RF
//-------------------------------------------------------------------------------------------------------------------------------------------
void send_rf_data()
{
  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &emontx, sizeof emontx);                           //send temperature data via RFM12B using new rf12_sendNow wrapper
  rf12_sendWait(2);
  //if (!ACAC) rf12_sleep(RF12_SLEEP);                             //if powred by battery then put the RF module into sleep inbetween readings
}

double calc_rms(int pin, int samples)
{
  unsigned long sum = 0;
  for (int i=0; i<samples; i++) // 178 samples takes about 20ms
  {
    int raw = (analogRead(0)-512);
    sum += (unsigned long)raw * raw;
  }
  double rms = sqrt((double)sum / samples);
  return rms;
}
//-------------------------------------------------------------------------------------------------------------------------------------------

#endif    // IMPORTANT LINE!
