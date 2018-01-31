
#include <JeeLib.h>  
#include <VoltageReference.h>

// ----------------- Hardware setup  ----------------------
#define INT_PIN 3                         // INT1 --> digital pin 3 for interruption (hall sensor)
#define BATT_ADC 0                        // Battery voltage measurement in ADC0
#define BATT_ADC_ENABLE_PIN 7             // PD7 enables the mosfet to measure in the voltage divider 

// ------------------- Battery definition -----------------
VoltageReference vRef = VoltageReference();
int vcc = 0;
uint16_t batVoltage = 0; 
uint8_t batLevel = 0;
uint16_t maxVoltage = 4150;
uint16_t minVoltage = 3400;
float dividerRatio = 1.305;

// ------------------- RF setup  --------------------------
#define freq RF12_868MHZ       // emonTx RFM12B working frequency
const int nodeID = 18;         // emonTx RFM12B node ID
const int networkGroup = 7;    // emonTx RFM12B wireless network group 
bool send_data = 0;


// ----------------- Data setup  --------------------------                                              
typedef struct { 
   uint16_t batery_voltage;  // in mV
   uint8_t batery_level;    // in %
} Tx_struct;            // data for RF comms
Tx_struct tx_data;


// ----------------- Interrupt setup  ---------------------  
ISR(WDT_vect) { Sleepy::watchdogEvent(); }      // Setup the watchdog


void setup() {
  
  // RF12 radio setup
  Sleepy::loseSomeTime(50);
  rf12_initialize(nodeID, freq, networkGroup);      // initialize RF
  rf12_sleep(RF12_SLEEP);

  // Com setup
  Serial.begin(57600);                              // initialize Serial interface
  Serial.println();
  Serial.println("---------------------------------------");
  Serial.println("Nodo de detección de apertura de puerta");
  Serial.println("Nodo 18 en la red 7 a 868Mhz");
  Serial.println("---------------------------------------");
  Serial.flush();
  Sleepy::loseSomeTime(100);

  //Pins setup
  pinMode(BATT_ADC_ENABLE_PIN,OUTPUT); digitalWrite(BATT_ADC_ENABLE_PIN,LOW);     // Disable mosfet
  
  pinMode(INT_PIN ,INPUT);                     // hall sensor wakes up the microcontroller

  // Hall Sensor Interrupt  http://web.ics.purdue.edu/~jricha14/Interrupts/MCUCR.htm 
  //                        https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
  //MCUCR |= (1<<ISC11);
  //EICRA |= (1 << ISC11);          // Set INT1 to trigger on FALLING logic change
  //EIMSK |= (1 << INT1);           // Turns on INT1
  // No se usa la interrupcón externa INT0 o INT1, sino que una interrupción por cambio de estado de un 
  // pin digital: Pin Change Interrupts. Se emplean estas instrucciones:
  
  PCICR |= (1<<PCIE2);              // Pin Change Interrupt Control Register
  PCMSK2 |= (1<<INT_PIN);           // The same pin as PCINT19 - INT1 - PD3
  sei();                            // Turn on interrupts
  
  // Internal ADC calibration for Battery measurement
  vRef.begin(1042930);

}

void loop() {
  
   if(send_data){

      // Measure batery voltaje
      vcc = vRef.readVcc();                       // Power supply measured from the uC internal and calibrated reference.
      digitalWrite(BATT_ADC_ENABLE_PIN,HIGH);     // Enable mosfet
      delayMicroseconds(20);
      analogRead(BATT_ADC);
      delay(2);
      batVoltage = analogRead(BATT_ADC) * dividerRatio * vcc / 1024;
      digitalWrite(BATT_ADC_ENABLE_PIN,LOW);      // Disable mosfet
      
      // Calculate batery remaining level 
      if (batVoltage <= minVoltage) {
        batLevel = 0;
      } else if (batVoltage >= maxVoltage) {
        batLevel = 100;
      } else {
        batLevel = (unsigned long)(batVoltage - minVoltage) * 100 / (maxVoltage - minVoltage);
      }

      tx_data.batery_voltage = batVoltage;
      tx_data.batery_level = batLevel;
      
      //Debug
      Serial.print("Battery Voltage is ");
      Serial.print(batVoltage);
      Serial.print(" mV (");
      Serial.print(batLevel);
      Serial.println("%)");
      Serial.flush();
    
      // Sending RF data
      send_rf_data();  
      
      send_data = false;     
     }
    
    Sleepy::powerDown();
    
     //delay(1500);
    
}

//wake up with the interrupt
ISR(PCINT2_vect) { 
   
  if(digitalRead(INT_PIN)){
        /* LOW to HIGH pin change: Do nothing */
  }else{
        /* HIGH to LOW pin change: Execute function */
        door_open_int();
  }
}

void door_open_int()
{ 
  send_data = true;     //activate the sending message flag 
}

void send_rf_data()
{
  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit route if it gets stuck 
  int i = 0; 
  while (!rf12_canSend() && i<10)
  { 
    rf12_recvDone(); 
    i++;
  }
  rf12_sendStart(0, &tx_data, sizeof tx_data);   // 0 --> Broadcast message
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
}
