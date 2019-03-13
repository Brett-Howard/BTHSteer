#include <SPI.h>                        //SPI Support
#include <RH_RF95.h>                    //LoRa Radio support
#include <avr/dtostrf.h>                //helps deal with floats in strings

//#define debug                   //comment this out to not depend on USB uart.
#define showReceivedPackets     //show recived messages from the mast head unit as they come in
//#define ackReceivedPackets      //should we send acks or not?
//#define showPacketLoss          //print packet lost messages
//#define showRSSI                //print out RSSI and SNR information

#define NMEABaud 4800

//turn on which NMEA0183 sentences to send
#define NMEA_VWR 1
#define NMEA_MWV 1

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
	  //while(!Serial);  //wait for USB port to be initialized
  #endif

  Serial1.begin(NMEABaud, SERIAL_8N1);   //configure NMEA output serial port to 4800 baud for the ST2000


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
  rf95.setTxPower(5);
  rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128);
}  //setup

void loop() {
    uint16_t spd;
    int16_t dir;
    uint8_t messageCount;
    uint16_t battVoltage;
    static uint32_t startTime;
    
    if (rf95.available())
    {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      uint8_t data[RH_RF95_MAX_MESSAGE_LEN];

      if (rf95.recv(buf, &len))
      {
        if(buf[0] != 'A') {  //ignore the ACK messages from BTHWind (wind data could start with decimal 65 "ascii A" but it would require over 80 knots of wind if my math is right.)
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
          
          //hard code test
          //dir = 175;
          //spd = 732;

          //send NMEA sentences that are enabled.
          if(NMEA_VWR) { sendVWR(spd, dir); }
          if(NMEA_MWV) { sendMWV(spd, dir); }
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

void sendVWR (uint16_t spd, int16_t dir)
{
  //Example Message: $BTVWR,045.0,L,12.6,N,6.5,M,23.3,K*44
  //       1   2 3   4 5   6 7   8 9
  //       |   | |   | |   | |   | |
  //$--VWR,x.x,a,x.x,N,x.x,M,x.x,K*hh<CR><LF>
  //Field Number:
  //1. Wind direction magnitude in degrees
  //2. Wind direction Left/Right of bow
  //3. Speed
  //4. N = Knots  
  //5. Speed
  //6. M = Meters Per Second
  //7. Speed  
  //8. K = Kilometers Per Hour
  //9. Checksum

  char boatSide;
  if(dir < 180)
    boatSide = 'R';
  else
  {
    boatSide = 'L';
    dir = 360 - dir;  //my anemometer object returns 0 to 360 rotating counter clockwise so this converts to bow referenced left or right.
  }
  char AWSKts[7];
  dtostrf(float(spd/100.0),0,1, AWSKts);  //converts the float to a c_str with no padding and 1 digit of precision.
  char AWSMps[7];
  dtostrf(ktsToMps(float(spd/100.0)),0,1, AWSMps);
  char AWSKph[7];
  dtostrf(ktsToKph(float(spd/100.0)),0,1, AWSKph);
  char tmp[maxNMEALength];

  //create the NMEA0183 string to be sent.
  sprintf(tmp,"BTVWR,%03d,%c,%s,N,%s,M,%s,K\0",dir,boatSide,AWSKts,AWSMps,AWSKph);
  
  //check for proper formatting, calculate the CRC and transmit string.
  nmeaSend(tmp);
}

void sendMWV(uint16_t spd, int16_t dir)
{
  //                1   2 3   4 5 6
  //                |   | |   | | |
  //Example: $--MWV,x.x,a,x.x,a,A*hh<CR><LF>
  //Field Number:
  //1. Wind Angle, 0 to 360 degrees
  //2. Reference, R = Relative, T = True
  //3. Wind Speed
  //4. Wind Speed Units, K/M/N (K=KPH, M=MPH, N=Knots)
  //5. Status, A = Data Valid
  //6. Checksum

  char tmp[maxNMEALength];

  char AWSKts[7];
  dtostrf(float(spd/100.0),0,1, AWSKts);  //converts the float to a c_str with no padding and 1 digit of precision.

  sprintf(tmp, "BTMWV,%03d,R,%s,N,A\0", dir, AWSKts);

  nmeaSend(tmp);
}

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
  snprintf(tmp, maxNMEALength, "$%s*%02X\r\n",s,checksum);
  
  #ifdef debug
    cout << tmp;
  #endif
  Serial1.print(tmp);
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