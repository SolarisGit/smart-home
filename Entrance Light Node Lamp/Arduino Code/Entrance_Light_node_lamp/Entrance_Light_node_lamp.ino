
#include <JeeLib.h>             // RF library
#include <util/crc16.h>         // CRC check library


// ------------------  Program variables ---------------------
#define LIGTHTIME 60000         // 60 seconds of light time when arriving home
#define SENDTIME 60000          // Send information to the central node every min
#define MIN_LIGHT 0             // 1-7KOhm light sensor --> min light:7K --> 0,4V --> 120 of 1024
#define MAX_LIGHT 970           // 1-7KOhm light sensor --> max light:1K --> 1,5V --> 500 of 1024
#define LIGHT_RANGE (MAX_LIGHT) - (MIN_LIGHT)

// ------------------  Pinout setup --------------------------
#define LAMP_PIN 8              // Relay attached to the pin 0 to turn On/Off the Lamp
#define LED_PIN 6               // Activity led to have RF communication feedback
#define _receivePin 8           // RF Data from  RFM12B module 

// ----------------- Temperature sensor setup  ---------------


// ------------------- RF setup  -----------------------------
#define freq RF12_868MHZ        // emonTx RFM12B working frequency
const int sendAddress = 15;      // emonTx RFM12B sending data to this address node
const int nodeID = 19;           // emonTx RFM12B node ID
const int networkGroup = 7;     // emonTx RFM12B wireless network group 
bool send_data = 0;

// ----------------- Data setup  -----------------------------                                              
typedef struct { 
   int temperature;         // in Celsius * 10
   int humidity;            // in %
   int heatindex;           // in Celsius * 10
   int pressure;            // in bar * 10
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


// ----------------- Program variables setup  -------------------------- 
unsigned long detec_time;
unsigned long send_time;
bool flag = 0;
bool masterkey = 1;

// ----------------- Function declaration ------------------------------
static void showString (PGM_P s); 
static void activityLed (byte on);
static void printOneChar (char c);
static void showNibble (byte nibble);
static byte showByte (byte value);
static word calcCrc (const void* ptr, byte len);
static void lampOn(void);
static void lampOff(void);


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

static void lampOff(void){
    Serial.println("Light Off!");
    digitalWrite(LAMP_PIN, LOW);
    delay(5);
    
}

static void lampOn(void){
    Serial.println("Light On!");
    digitalWrite(LAMP_PIN, HIGH);
    delay(5);
        
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
  
  // LAMP RELAY
  pinMode(LAMP_PIN, OUTPUT);  
  digitalWrite(LAMP_PIN, LOW);   // Light Off

  // Debug LED
  pinMode(LED_PIN, OUTPUT);   

  // Temp and hum sensor initialization

  
  // Enabe RF commands to the Lamp
  masterkey = 1;

}

void loop() {
  
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

        if(masterkey){
            if(msg.equals("OK G7 18")){
                lampOn();
                detec_time = millis();               // Take time   
                msg = "";                            // Empty msg
                flag = 1;
            }
                
            if(flag){
                if((millis()-detec_time)>LIGTHTIME){
                    lampOff();
                    flag = 0;
                }
            }
        }


    msg = "";      // Clear Message
    
    // Check if it's time to send the RF message
    if((millis()-send_time)>SENDTIME){     // Time to send RF info
      
      activityLed(1);
      
      
//      Serial.print("H: ");
//      Serial.print(h);
//      Serial.print("% T: ");
//      Serial.print(t);
//      Serial.print("ºC Heat: ");
//      Serial.print(hic);
//      Serial.print("ºC L: ");  
//      Serial.print(lum);
//      Serial.print("% Mkey:");
//      Serial.println(masterkey);
//      Serial.flush();
      
//      tx_data.temperature = int(t*100); 
//      tx_data.humidity = int(h*100);
//      tx_data.heatindex = int(hic*100);
//      tx_data.luminosity = int(lum*100);
      
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
