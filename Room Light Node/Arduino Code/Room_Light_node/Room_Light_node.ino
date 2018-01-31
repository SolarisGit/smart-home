
#include <JeeLib.h>             // RF library
#include <IRremote.h>           // Remote IRD
#include <util/crc16.h>         // CRC check library
#include "DHT.h"

// ------------------  Program variables ---------------------
#define LIGTHTIME 60000         // 60 seconds of light time when arriving home
#define SENDTIME 60000          // Send information to the central node every min
#define MIN_LIGHT 0             // 1-7KOhm light sensor --> min light:7K --> 0,4V --> 120 of 1024
#define MAX_LIGHT 970           // 1-7KOhm light sensor --> max light:1K --> 1,5V --> 500 of 1024
#define LIGHT_RANGE (MAX_LIGHT) - (MIN_LIGHT)

// ------------------  Pinout setup --------------------------
#define PWM_CHANNEL 8           // Mosfet attached to the pin 0 to regulate the leds
#define RECV_PIN  3             // IRD sensor connected to digital pin 3
#define LIGHT_ADC_PIN 5         // Light sensor connected to the ADC pin 5
#define LED_PIN 6               // Activity led to have RF communication feedback
#define _receivePin 8           // RF Data from  RFM12B module ??
#define DHTPIN 9                // Humidity aand temperature sensor

// ----------------- Temperature sensor setup  ---------------
//#define DHTTYPE DHT11         // DHT 11
#define DHTTYPE DHT22           // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21         // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

// ------------------- RF setup  -----------------------------
#define freq RF12_868MHZ        // emonTx RFM12B working frequency
const int sendAddress = 15;      // emonTx RFM12B sending data to this address node
const int nodeID = 17;           // emonTx RFM12B node ID
const int networkGroup = 7;     // emonTx RFM12B wireless network group 
bool send_data = 0;

// ----------------- Data setup  -----------------------------                                              
typedef struct { 
   int temperature;         // in Celsius * 10
   int humidity;            // in %
   int heatindex;           // in Celsius * 10
   int luminosity;          // in %
} Tx_struct;                    // data for RF comms
Tx_struct tx_data;

static int _bitDelay;
static char _receive_buffer;
static byte _receive_buffer_index;

// RF12 configuration area
typedef struct {
    byte group;                 // used by rf12_config, offset 1
    byte hex_output   :2;       // 0 = dec, 1 = hex, 2 = hex+ascii
    byte quiet_mode   :1;       // 0 = show all, 1 = show only valid packets
} RF12Config;

static RF12Config config;
String msg;

// ----------------- Infra Red setup  --------------------------  
IRrecv irrecv(RECV_PIN);        // Interrupcion para recibir IRD
decode_results results;

// ----------------- Program variables setup  -------------------------- 
unsigned long detec_time;
unsigned long send_time;
unsigned long infraRed;
bool flag = 0;
bool masterkey = 1;

// ----------------- Function declaration ------------------------------
static void showString (PGM_P s); 
static void activityLed (byte on);
static void printOneChar (char c);
static void showNibble (byte nibble);
static byte showByte (byte value);
static word calcCrc (const void* ptr, byte len);
static void ledOn(void);
static void ledOff(void);
static float luminosity(void);

ISR (PCINT0_vect) {
    char i, d = 0;
    if (digitalRead(_receivePin))       // PA2 = Jeenode DIO2
        return;             // not ready!
    whackDelay(_bitDelay - 8);
    for (i=0; i<8; i++) {
        whackDelay(_bitDelay*2 - 6);    // digitalread takes some time
        if (digitalRead(_receivePin)) // PA2 = Jeenode DIO2
            d |= (1 << i);
    }
    whackDelay(_bitDelay*2);
    if (_receive_buffer_index)
        return;
    _receive_buffer = d;                // save data
    _receive_buffer_index = 1;  // got a byte
}


void whackDelay (word delay) {
    byte tmp=0;

    asm volatile("sbiw      %0, 0x01 \n\t"
                 "ldi %1, 0xFF \n\t"
                 "cpi %A0, 0xFF \n\t"
                 "cpc %B0, %1 \n\t"
                 "brne .-10 \n\t"
                 : "+r" (delay), "+a" (tmp)
                 : "0" (delay)
                 );
}

static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, on);
#endif
}

static void printOneChar (char c) {
    Serial.print(c);
}

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);
    }
}

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    Serial.print(c);
}

static byte showByte (byte value) {
    if (config.hex_output) {
        showNibble(value >> 4);
        showNibble(value);
    } else
        Serial.print((word) value);
        return ((word) value);
}

static word calcCrc (const void* ptr, byte len) {
    word crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
}

static void ledOff(void){
    Serial.println("Light Off!");
    for(byte i = 0; i < 255 ; i++){
          analogWrite(PWM_CHANNEL, i);   
          delay(5);
    }
}

static void ledOn(void){
    Serial.println("Light On!");
    for(byte i = 255; i > 0 ; i--){
          analogWrite(PWM_CHANNEL, i);  
          delay(5);
        }
}

static float luminosity(void){
      float light = analogRead(LIGHT_ADC_PIN);
      return 100*((light-MIN_LIGHT)/LIGHT_RANGE);
}

void setup() {
  
  delay(200);
  
  // Com setup
  Serial.begin(57600);
  Serial.println();
  Serial.println("Room Light Node");
  Serial.print("Node ID: "); Serial.println(nodeID); 
  Serial.print("Network Group: "); Serial.println(networkGroup);
  Serial.println("Working Freq: 868 MHz");
  Serial.print("Sending data to Node: "); Serial.println(sendAddress);
  Serial.println();
  
  // RF12 radio setup
  rf12_initialize(nodeID, freq, networkGroup);      // initialize RF
  config.quiet_mode = 1;                            // 1 -> not debuging RF frames
  
  // IRD setup
  irrecv.enableIRIn();           // Start the receiver

  // LED ARRAY PWM
  pinMode(PWM_CHANNEL, OUTPUT);  
  analogWrite(PWM_CHANNEL, 255);   // Light Off

  // Debug LED
  pinMode(LED_PIN, OUTPUT);   

  // Temp and hum sensor initialization
  dht.begin();
  
  // Enabe RF commands to the Leds
  masterkey = 1;

}

void loop() {

    // Infra Red communication
    if (irrecv.decode(&results)) {                     
      irrecv.resume();                        // Receive the next value
      infraRed = results.value;
      Serial.println(infraRed, HEX);
    }
    
    // Check Infra Red communication
    switch(infraRed){
      case 0xE0E036C9:                        // Light on  
        masterkey = 0;                        // Disable control on the Leds, stay on continuously
        infraRed = 0;
        ledOn();
      break;
      
      case 0xE0E028D7:                        // Light off 
        masterkey = 1;                        // Enable control on the Leds
        infraRed = 0;
        ledOff();
      break;
    }
    
    // RF communication
    if (rf12_recvDone()) {
        byte n = rf12_len;
        if (rf12_crc == 0){
            showString(PSTR("OK"));
            msg = "OK";
        }else {
            if (config.quiet_mode)
                return;
            showString(PSTR(" ?"));
            if (n > 20)                        // Print at most 20 bytes if crc is wrong
                n = 20;
        }
        if (config.group == 0) {
            showString(PSTR(" G"));
            byte g = showByte(rf12_grp);
            msg += " G"; msg += g; msg += " ";
             
        }
        printOneChar(' ');
        byte from_node = showByte(rf12_hdr);
        msg += from_node; 
        for (byte i = 0; i < n; ++i) {
            if (!config.hex_output)
                printOneChar(' ');
            showByte(rf12_data[i]);
        }
        Serial.println();
    }

    // Check if we have to light the leds
    if(luminosity()<70){
        if(masterkey){
            if(msg.equals("OK G7 18")){
                ledOn();
                detec_time = millis();               // Take time   
                msg = "";                            // Empty msg
                flag = 1;
            }
                
            if(flag){
                if((millis()-detec_time)>LIGTHTIME){
                    ledOff();
                    flag = 0;
                }
            }
        }
     }

    msg = "";      // Clear Message
    
    // Check if it's time to send the RF message
    if((millis()-send_time)>SENDTIME){     // Time to send RF info
      
      activityLed(1);
      
      float h = dht.readHumidity();
      float t = dht.readTemperature();                  // Read temperature as Celsius 
      float hic = dht.computeHeatIndex(t, h, false);    // Compute heat index in Celsius (isFahreheit = false)
      float lum = luminosity();
      
      Serial.print("H: ");
      Serial.print(h);
      Serial.print("% T: ");
      Serial.print(t);
      Serial.print("ºC Heat: ");
      Serial.print(hic);
      Serial.print("ºC L: ");  
      Serial.print(lum);
      Serial.print("% Mkey:");
      Serial.println(masterkey);
      Serial.flush();
      
      tx_data.temperature = int(t*100); 
      tx_data.humidity = int(h*100);
      tx_data.heatindex = int(hic*100);
      tx_data.luminosity = int(lum*100);
      
      send_rf_data();
      send_time = millis();

      activityLed(0);
    }


} // loop end

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
  rf12_sendStart(sendAddress, &tx_data, sizeof tx_data);   // 5 --> to the Raspberry.
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
}
