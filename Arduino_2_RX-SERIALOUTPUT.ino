// For LoRan
#include <SPI.h>
#include <RH_RF95.h>
// Time Library
#include <TimeLib.h>
// For OLED Firmware
// #include <Adafruit_SSD1331.h>
// For Displaying on OLED
// #include <Adafruit_GFX.h>

// Define LoRan pins
#define RFM95_CS    4  
#define RFM95_INT   3  
#define RFM95_RST   2
// Loran frequency
#define RF95_FREQ 434.0
// Dummy value for comparison
#define DUMMY 99
// Time zone definitions
#define MST 7
#define DAYLIGHT 0 

// // Define display pins
// #define sclk 13
// #define mosi 11
// #define cs   5
// #define rst  6
// #define dc   8

// // Color definitions (for display)
// #define	BLACK           0x0000
// #define	BLUE            0x001F
// #define	RED             0xF800
// #define	GREEN           0x07E0
// #define CYAN            0x07FF
// #define MAGENTA         0xF81F
// #define YELLOW          0xFFE0
// #define WHITE           0xFFFF

// For declaring the LoRan object
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Old code for display initialization
// Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);

// All time global variables
int Wyear, Wmonth, Wday, Whour, Wminute, Wsecond, Wsyncmin;
int Gyear, Gmonth, Gday, Ghour, Gminute, Gsecond;

// RX buffer
uint8_t buf[22];

// Display and BPSK sync timer
uint32_t dtimer = millis();
uint32_t BPTimer;

// BPSK data classification variable
int BPGood = 0;

// if the LoRan module has started
bool loranStart = false;
// ========================================================================= ALL FUNCTIONS ========================================================
// --------------------------------------------------------------- LORAN FUNCTIONS ---------------------------------------------------------------
void LoRanSetup() // Initializing the Loran Module
{
  // To ensure all pins are in needed modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Wait for the serial monitor to begin
  Serial.begin(9600);
  while (!Serial) delay(1);
  delay(100);

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // If the module cannot initialize, stop the program
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Module isn't transimitting so make sure the module knows it isn't being used
  rf95.setTxPower(23, false);
}

void LoRanRx() // For recieving data
{
  // If the packet is ready
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      // Change the lights on the arduino to show that its recieving time
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
      // Log the data into an array
      loranStart = true;
      LogData();
    } else {
      // Or print that it wasn't able to recieve the signal
      Serial.println("Receive failed");
    }
  }
}

// ----------------------------------------------------------------- DATA OPERATIONS -------------------------------------------------------------
void LogData()
{
  // If there has been a WWV AM Sync
  if (buf[0] == 70)
  {
    // Log all of the buffer variables into local variables
    Whour = buf[1];
    Wminute = buf[2];
    Wsecond = buf[3];
    Wmonth = buf[4];
    Wday = buf[5];
    Wyear = buf[6];
    Wsyncmin = buf[7];
  }
  else
  {
    // Make all of the local variables dummy ones to ignore them
    Whour = DUMMY;
    Wminute = DUMMY;
    Wsecond = DUMMY;
    Wmonth = DUMMY;
    Wday = DUMMY;
    Wyear = DUMMY;
    Wsyncmin = DUMMY;    
  }
  // Log all of the GPS data from the buffer into local variables
  Ghour = buf[9];
  Gminute = buf[10];
  Gsecond = buf[11];
  Gmonth = buf[12];
  Gday = buf[13];
  Gyear = buf[14];

  Serial.print(buf[15]);
  // If there is a good BPSK sync, then set the time accordingly
  if (buf[15] == 80 && millis() - BPTimer > 10000)
  {
    int Bhour = buf[16];
    int Bminute = buf[17];
    int Bsecond = buf[18];
    int Bday = buf[20];
    int Bmonth = buf[19];
    int Byear = (buf[21] + 2000);
    // Set the arduino local clock to this time
    setTime(Bhour, Bminute, Bsecond, Bday, Bmonth, Byear);

    // Classify this data as good
    BPGood++;
    // Set the last sync timer gap
    BPTimer = millis();
  }
}

// ---------------------------------------------------------------- DISPLAYING DATA -------------------------------------------------------------
void SerialDataDisplay()
{
  // Cycle through all of the differerent displaying functions
  WWVAMSerialDisp();
  GPSSerialDisp();
  WWVBPSKSerialDisp();
}

void WWVAMSerialDisp()
{
  // If there is good WWV AM data
  if (Whour != DUMMY)
  {
    // Print all of this data (this is the same process as is used on the TX arduino, reference this for a more detailed breakdown)
    Serial.print("WWV: ");
    if (Whour > 12)
    {
      Serial.print(Whour - 12);
    }
    else
    {
      Serial.print(Whour);
    }
    Serial.print(":");
    if (Wminute < 10)
    {
      Serial.print("0");
    }
    Serial.print(Wminute);
    Serial.print(":");
    if (Wsecond < 10)
    {
      Serial.print("0");
    }
    Serial.print(Wsecond);
    if (Whour > 11)
    {
      Serial.print(" PM ");
    }
    else
    {
      Serial.print(" AM ");
    }
    Serial.print(Wmonth);
    Serial.print("/");
    Serial.print(Wday);
    Serial.print("/");
    Serial.print(Wyear + 2000);
  }
}

void GPSSerialDisp()
{
  // Print GPS data, in a similar way as that of the WWV AM
  Serial.print(" GPS:");
  if (Ghour > 12)
  {
    Serial.print(Ghour - 12);
  }
  else
  {
    Serial.print(Ghour);
  }
  Serial.print(":");
  if (Gminute < 10)
  {
    Serial.print("0");
  }
  Serial.print(Gminute);
  Serial.print(":");
  if (Gsecond < 10)
  {
    Serial.print("0");
  }
  Serial.print(Gsecond);
  if (Ghour > 11)
  {
    Serial.print(" PM ");
  }
  else
  {
    Serial.print(" AM ");
  }
  Serial.print(Gmonth);
  Serial.print("/");
  Serial.print(Gday);
  Serial.print("/");
  Serial.println(Gyear + 2000);
}

void WWVBPSKSerialDisp()
{
  // If there is good data, then display this using the internal clock
  if (BPGood > 0)
  {
    int h = hour();
    int m = minute();
    int s = second();
    int mo = month();
    int d = day();
    int y = year();
    Serial.print(" BPSK: ");
    if (h > 12)
    {
      Serial.print(h - 12);
    }
    else
    {
      Serial.print(h);
    }
    Serial.print(":");
    if (m < 10)
    {
      Serial.print("0");
    }
    Serial.print(m);
    Serial.print(":");
    if (s < 10)
    {
      Serial.print("0");
    }
    Serial.print(s);
    if (h > 11)
    {
      Serial.print(" PM ");
    }
    else
    {
      Serial.print(" AM ");
    }
    Serial.print(mo);
    Serial.print("/");
    Serial.print(d);
    Serial.print("/");
    Serial.println(y);
  }
}

void setup() {
  LoRanSetup();
  dtimer = millis();
}

void loop() {
  // Recieve the LoRan Data
  LoRanRx();
  // If its been a second and the loran module has begun recieving, then display the text and reset the timer
  if (millis() - dtimer > 1000 && loranStart == true)
  { 
    SerialDataDisplay();
    dtimer = millis();
  }
}