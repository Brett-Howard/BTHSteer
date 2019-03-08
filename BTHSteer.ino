#include <SPI.h>                        //SPI Support
#include <RH_RF95.h>                    //LoRa Radio support
#include <avr/dtostrf.h>                //helps deal with floats in strings

//#define debug                   //comment this out to not depend on USB uart.
#define showReceivedPackets     //show recived messages from the mast head unit as they come in
//#define ackReceivedPackets      //should we send acks or not?
//#define showPacketLoss          //print packet lost messages
//#define showRSSI                //print out RSSI and SNR information

#define radioTimeoutReset 5000      //reset radio reception variables if no message is received for this period.

#define maxNMEALength 64

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define RH_RF95_MAX_MESSAGE_LEN 32
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define RED_LED_PIN 13

#ifdef debug
    #include <SdFat.h>      //stupidly only have this here for cout. ;)  Judge me if you want its only for debugging.
    ArduinoOutStream cout(Serial);  //allows "cout <<" to go to the Serial port
#endif

RH_RF95 rf95(RFM95_CS, RFM95_INT);   //instantiate the radio object

void setup() {
  #ifdef debug
  	Serial.begin(115200);
	  while(!Serial);  //wait for USB port to be initialized
  #endif

  Serial1.begin(4800);   //configure NMEA output serial port to 4800 baud for the ST2000


  //setup radio pins
  pinMode(RFM95_CS, OUTPUT);
  digitalWrite(RFM95_CS, HIGH);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

 //////////////////////////////////////////////////////Setup LoRa Radio//////////////////////////////////////////////////////
    
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  #ifdef debug
    Serial.println("LoRa radio init OK!");
  #endif

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  #ifdef debug
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  #endif

  //power is adjustible from 5 to 23dBm
  rf95.setTxPower(5);  //leaving at the default power because this is plugged in (mashead is at 5dBm)
  rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128);
}  //setup

void loop() {
    uint16_t spd;
    int16_t dir;
    uint8_t messageCount;
    uint16_t battVoltage;

    if (rf95.available())
    {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      uint8_t data[RH_RF95_MAX_MESSAGE_LEN];

      if (rf95.recv(buf, &len))
      {
        if(buf[0] != 'A') {  //ignore the ACK messages from BTHWind
          #ifdef debug
            #ifdef showRSSI
              cout << "RSSI: " << rf95.lastRssi() << " SNR: " << rf95.lastSNR() << endl;
            #endif
          #endif
          strcpy((char*)data, "A");    //get ready to send back an "A" for ACK.
           
          //Grab the values from the recieved message
          memcpy(&spd, &buf, 2);
          memcpy(&dir, &buf[2], 2);
          memcpy(&battVoltage, &buf[4], 2);
          memcpy(&messageCount, &buf[6], 1);
            
          #ifdef debug
            #ifdef showPacketLoss
              static uint8_t lastMessage;
              static uint16_t packetsLost = 0;
              static uint32_t packetsReceived = 0;
              if(messageCount != uint8_t(lastMessage + 1)) {   //if the message count isn't sequential
                ++packetsLost;
                cout << "Packet lost!!!!!!" << endl << endl;
                cout << "Packet Loss: " << double(packetsLost) / double(packetsReceived) * 100.0 << "%" << endl;
                lastMessage = messageCount;
              }
              ++packetsReceived;
              lastMessage = messageCount;
            #endif
            #ifdef showReceivedPackets
              cout << "Time = " << millis() << " Speed =" << spd << " Direction =" << dir << " Voltage=" << battVoltage << " MessageCount="; Serial.println(messageCount, DEC);
            #endif          
          #endif

          #ifdef ackReceivedPackets
            rf95.send(data, sizeof(data));  //transmit ACK response
            rf95.waitPacketSent();
          #endif

          //transmit Wind NMEA data to autopilot
          //Example Message: $BTVWR,045.0,L,12.6,N,6.5,M,23.3,K*44
          //045.0 - Relative Wind Angle 0 to 180 degrees
          //L - Left or Right (really should be P or S but I digress)...
          //12.6,N - Wind speed in knots
          //6.5, M - Wind speed in meters per second
          //23.3, K - Wind speed in Km/Hr
          //*44 - Checksum
          char boatSide;
          if(dir < 180)
            boatSide = 'R';
          else
          {
            boatSide = 'L';
            dir = 360 - dir;
          }
          char AWSKts[7];
          dtostrf(float(spd/100.0),0,1, AWSKts);  //converts the float to a c_str with no padding and 1 digit of precision.
          char AWSMps[7];
          dtostrf(ktsToMps(float(spd/100.0)),0,1, AWSMps);
          char AWSKph[7];
          dtostrf(ktsToKph(float(spd/100.0)),0,1, AWSKph);
          char tmp[maxNMEALength];

          //create the NMEA0183 string to be sent.
          sprintf(tmp,"$BTVWR,%03d.0,%c,%s,N,%s,M,%s,K\0",dir,boatSide,AWSKts,AWSMps,AWSKph);

          //check for proper formatting, calculate the CRC and transmit string.
          nmeaSend(tmp);
        }
      }
      else {
        #ifdef debug
          cout << "Receive failed" << endl;
        #endif
      }
    }
  
    //cout << F("Free Mem: ") << freeRam() << endl;
}  //loop

//nmeaSend expects a null terminated char array.
//each message takes about 80mS to transmit at 4800 baud.
void nmeaSend(char* s)
{
  uint8_t checksum;
  char tmp[maxNMEALength];
  int i = 0;

  //strip off leading $ if we were given one.
  if('$' == s[0])
  {
    s++;
  }

  //calculate checksum
  i = 0;
  checksum = 0;
  while('\0' != s[i] && '*' != s[i])  //look for a null (but don't include any trailing *'s)
  {
    checksum ^= s[i];
    ++i;
  }

  //if someone sent us a string ending in * truncate it off.
  if('*' == s[i])
  {
    s[i] = '\0';
  }

  //format string
  snprintf(tmp, maxNMEALength, "$%s*%02X",s,checksum);
  
  #ifdef debug
    cout << tmp << endl;
  #endif
  Serial1.println(tmp);
}

float ktsToMps(float a)
{
  return a*0.514444;
}

float ktsToKph(float a)
{
  return a*1.852;
}

//A function that allows for finding out how much free memory is available.
extern "C" char *sbrk(int i); 
int freeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

void TC5_Handler (void) {   //This timer gets moved to the mast head in a wireless version of this project
    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}


//Just blinks an LED you can set the number of times and the duration of the blinks
//This only works for the LEDs on the GPIO pins directly on the Arduino board.
static void blip(int ledPin, int times, int dur) {
  pinMode(ledPin, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
  }
} //blip

//This is a way of saying we've died on the vine.  This function is meant to never return.
static void failBlink() {
  pinMode(RED_LED_PIN, OUTPUT );  //seems that I need this here because something is breaking this.
  #ifdef debug
    cout << "got to FailBlink" << endl;
  #endif
  while(1) blip(RED_LED_PIN, 1, 75);
} //failBlink

//////////////////////////////////////////////////////Timer Counter Configuration///////////////////////////////////////////////////
//configures the timer counter to allow for a slow tick to use if/when the wind gets so low the anemometer interrupts stop
void tcConfigure(int sampleRate)
{
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT32;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  delay(10);

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  delay(10); //wait until TC5 is done syncing 
} 
//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  delay(10);
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}
//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  delay(10);
}