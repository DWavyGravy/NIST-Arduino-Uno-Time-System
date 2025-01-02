// ============================================ HEADER FILES & SUCH ======================================================
// Time Library
#include <TimeLib.h>
// For LoRan Firmware
#include <SPI.h>
#include <RH_RF95.h>
// For GPS
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// All incoming bit codes
#define ERRBIT 4
#define NOBIT 3
#define MRKBIT 2
#define HIBIT 1
#define LOBIT 0
// Time Zone
#define MST 7
#define DAYLIGHT 0

// GPS Definition
#define GPSECHO true

// LoRan Pins
#define RFM95_CS    4 
#define RFM95_INT   3  
#define RFM95_RST   2 

// Loran Frequency
#define RF95_FREQ 434.0

// GPIO pins
#define GPIO_EN      9
#define GPIO_IRQ     10
// I2C slave address
#define ES100_SLAVE_ADDR 0x32

// enable delay time
//#define ENABLE_DELAY_US 1500
#define ENABLE_DELAY_US 100000       // 100ms: module version 1 only

// I2C clock high/low time
#define SCL_TIME_US 2

// ES100 API register addresses
#define ES100_CONTROL0_REG       0x00
#define ES100_CONTROL1_REG       0x01
#define ES100_IRQ_STATUS_REG     0x02
#define ES100_STATUS0_REG        0x03
#define ES100_YEAR_REG           0x04
#define ES100_MONTH_REG          0x05
#define ES100_DAY_REG            0x06
#define ES100_HOUR_REG           0x07
#define ES100_MINUTE_REG         0x08
#define ES100_SECOND_REG         0x09
#define ES100_NEXT_DST_MONTH_REG 0x0A
#define ES100_NEXT_DST_DAY_REG   0x0B
#define ES100_NEXT_DST_HOUR_REG  0x0C

#define DT_STATUS         0
#define DT_YEAR           1
#define DT_MONTH          2
#define DT_DAY            3
#define DT_HOUR           4
#define DT_MINUTE         5
#define DT_SECOND         6
#define DT_NEXT_DST_MONTH 7
#define DT_NEXT_DST_DAY   8
#define DT_NEXT_DST_HOUR  9

#define DT_LENGTH        10

// Dummy value for LoRan packet
#define DUMMY 99

// =============================================== GLOBAL VARIABLES ==========================================================

// LoRan Instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Begin GPS communication
SoftwareSerial mySerial(6, 5);
Adafruit_GPS GPS(&mySerial);

// Defines pin of the Radio
int pin = 7;

// Creates the timer to handle the data and interrupts
const uint16_t tl_load = 0;
const uint16_t tl_comp = 625;

// Keeps track of last sync 
time_t goodstuff = 0;

// Keeps track of pulses and samples for the interrupt
volatile byte pulseWidth = 0;
volatile byte samplecounter = 1000;

// Keeps track of the 2 most recent recieved bits
volatile byte newBit = NOBIT;
volatile byte oldBit = NOBIT;

// Frame and index for storing bits, as well as segment array for error checking
volatile byte frame[60];
int frameIndex = 0;
volatile byte seg[6];

// Stops the time from being displayed until after a full sync
int starttime = 0;
int BPSKstart = 0;

// For keeping track of all time variables
int yr,mo,dy,hr,mn,leap,dst;
int sec, min, hou, mon, days, yea;


// BPSK variables
int         current_timer_value_global;
int         dt_array_global[DT_LENGTH];

// LoRan Time Code Variables
uint8_t WWVAMTimeCode = 70;
uint8_t GPSTimeCode = 75;
uint8_t WWVBPSKTimeCode = 81; // Must start at 81 for the TX code to function

// For making sure the BPSK sync comes through LoRan
int firstSyncTimerAllowance = 0;

// For BPSK resync
long BPSKtimer;
// ===================================================================== ALL FUNCITONS ======================================================
// --------------------------------------------------------------------- BPSK FUNCTIONS ------------------------------------------------------
void mcu_init(void)
{
  // optional code to initialize GPIO, timer and other MCU functions
  pinMode(GPIO_EN, OUTPUT);
  pinMode(GPIO_IRQ, INPUT);
  Wire.begin();
}

void mcu_gpio_set_high(int bpin)
{
  // set pin to a logic one
  digitalWrite(bpin, HIGH);
}

void mcu_gpio_set_low(int bpin)
{
  // set pin to a logic zero
  digitalWrite(bpin, LOW);
}

int mcu_gpio_read(int bpin)
{
  // read current state of input pin
  return(digitalRead(bpin));
}

int mcu_timer_read(void)
{
  // read timer value and return as integer
  return(millis());
}

void mcu_timer_wait_us(int count)
{
  // wait 1 microsecond or greater for each count
  delayMicroseconds(count);
}

void i2c_write(uint8_t slave_addr, uint8_t num_bytes, uint8_t *ptr)
{

  int i;

  Wire.beginTransmission(slave_addr);

  for (i=0; i<num_bytes; i++)
  {
    Wire.write(ptr[i]);
  }

  Wire.endTransmission();

}

void i2c_read(uint8_t slave_addr, uint8_t num_bytes, uint8_t *ptr)
{

  int i;
  const uint8_t stop_flag = 1;

  Wire.requestFrom(slave_addr, num_bytes, stop_flag);
  for(i=0; (i<num_bytes && Wire.available()); i++)
  {
    ptr[i] = Wire.read();
  }
}

void es100_write_register(uint8_t addr, uint8_t data)
{
  uint8_t es100_write_array[2];

  es100_write_array[0]= addr;
  es100_write_array[1]= data;
  i2c_write(ES100_SLAVE_ADDR, 0x2, es100_write_array);
}

uint8_t es100_read_register(uint8_t addr)
{
  uint8_t   data;

  i2c_write(ES100_SLAVE_ADDR, 0x1, &addr);
  i2c_read(ES100_SLAVE_ADDR, 0x1, &data);

  return(data);
}

void es100_enable(void)
{
  mcu_gpio_set_high(GPIO_EN);
}

void es100_disable(void)
{
  mcu_gpio_set_low(GPIO_EN);
}

void es100_start_rx()
{
  es100_write_register(ES100_CONTROL0_REG, 0x01);

  // perform read of control register for i2c debug only
  es100_read_register(ES100_CONTROL0_REG);
}

void es100_wait_for_irq()
{
  while(mcu_gpio_read(GPIO_IRQ));   // wait until IRQ is low
}

uint8_t es100_get_irq_status()
{
  //Read IRQ status register
  return(es100_read_register(ES100_IRQ_STATUS_REG));
}

void es100_read_time(int dt_array[])
{
  dt_array[DT_STATUS]         = es100_read_register(ES100_STATUS0_REG);
  dt_array[DT_YEAR]           = es100_read_register(ES100_YEAR_REG);
  dt_array[DT_MONTH]          = es100_read_register(ES100_MONTH_REG);
  dt_array[DT_DAY]            = es100_read_register(ES100_DAY_REG);
  dt_array[DT_HOUR]           = es100_read_register(ES100_HOUR_REG);
  dt_array[DT_MINUTE]         = es100_read_register(ES100_MINUTE_REG);
  dt_array[DT_SECOND]         = es100_read_register(ES100_SECOND_REG);
  dt_array[DT_NEXT_DST_MONTH] = es100_read_register(ES100_NEXT_DST_MONTH_REG);
  dt_array[DT_NEXT_DST_DAY]   = es100_read_register(ES100_NEXT_DST_DAY_REG);
  dt_array[DT_NEXT_DST_HOUR]  = es100_read_register(ES100_NEXT_DST_HOUR_REG);
}

int es100_receive(int dt_array[])
{
  // local variables
  int         irq_status = 0;
  int         current_timer_value;
  int         count = 0;

  // enable and delay
  Serial.println(F("enabling es100, all functions stopped until sync has been finished"));
  es100_enable();
  mcu_timer_wait_us(ENABLE_DELAY_US);

  // start reception
  es100_start_rx();

  // loop until time received
  while (irq_status != 0x01)
  {
    // wait for interrupt
    es100_wait_for_irq();

    // interrupt defines second boundary, so save current timer value
    current_timer_value = mcu_timer_read();

    // read interrupt status
    irq_status = es100_get_irq_status();
  }

  // read date and time
  es100_read_time(dt_array);

  // disable ES100
  es100_disable();

  Serial.println(F("BPSK Sync Aquired"));
  // return timer value when interrupt occurred
  return(current_timer_value);
}
// -------------------------------------------------------------------- TIMER INTERRUPTS ----------------------------------------------------
void interruptSetup() // Sets Up Interrupt
{
  // For more information as to how these registers work, please reference this video by sparkfun: https://www.youtube.com/watch?v=2kr5A350H7E
  TCCR1A = 0;

  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);

  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TCNT1 = tl_load;
  OCR1A = tl_comp;

  TIMSK1 = (1 << OCIE1B);

  sei();
}

void timerHandler () // Handles timer interrupt
{
  // Adds the digital read to the pulse width value
  pulseWidth += !digitalRead(pin);
  // Counts down until next full sample
  samplecounter--;
  // If full sample, determine what bit the new one is
  if (samplecounter == 0)
  {
    // If the width is large, Marker
    if (pulseWidth > 63 && pulseWidth < 90)
    {
      newBit = MRKBIT;
    }
    // If the width is slightly smaller, highbit
    else if (pulseWidth > 33 && pulseWidth < 60)
    {
      newBit = HIBIT;
    }
    // If the width is even smaller, lowbit
    else if (pulseWidth > 5 && pulseWidth < 30)
    {
      newBit = LOBIT;
    }
    // If no other width has matched, bit must be an error
    else
    {
      newBit = ERRBIT;
    }
    // Reset counter and pulse width
    samplecounter = 100;
    pulseWidth = 0;
  }
}

// -------------------------------------------------------------------- DATA STORAGE AND ERROR CHECKING -----------------------------------------------
void startNewFrame() // Starts new frame for storing data
{
  // Resets the frame index
  frameIndex = 0;
}

bool validSegment() // Checks if a group of 9 bits doesn't contain errors
{
  // Assume the data is good
  bool goodData = true;
  // Determine where to start error check
  int start = frameIndex - 9;
  for (int j = 0; j < (start + 8); j++)
  {
    // If a frame has an error, data is bad
    if (frame[j] == ERRBIT)
    {
      goodData = false;
    }
  }
  // If the newest bit isn't a marker, segment is bad
  if (newBit != MRKBIT)
  {
    goodData = false;
  }
  // This segment of 9 bits must otherwise be good
  return goodData;
}

bool validFrame() // Checks if a frame has markers in the right spots
{
  // Iterate over all 6 bits where markers should be
  for (int i = 0; i < 6; i++)
  {
    // If a segment doesn't have a marker, false
    if (seg[i] < 1)
    {
      return false;
    }
  }
  // All segments must have markers, so data is good
  return true;
}

void clearSegment() // Clears all data in the Seg array
{
  // Iterates over segment and clears all data
  for (int i = 0; i < 6; i++)
  {
    seg[i] = 0;
  }
}

// ------------------------------------------------------------------ LORAN -------------------------------------------------------------------------
void LoRanSetup() // Sets up LoRan Module
{
  // Set the pin that the LoRan module is connected to to output, ready the SPI
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Make sure the sync works correctly
  while (!Serial) delay(1);
  delay(100);

  // Again, checking to make sure the LoRan module is synced up with the Uno, ready to send a signal
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // If the radio module isn't initializing, display error message, if this is the case, please check your wiring
  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }
  // If the initialization hasn't failed, then it is working
  Serial.println(F("LoRa radio init OK!"));

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    // If frequency wasn't able to be set, then display error message
    Serial.println(F("setFrequency failed"));
    while (1);
  }
  // Otherwise, show what frequency the radio will be set to
  Serial.print(F("Set Freq to: ")); Serial.println(RF95_FREQ);

  // Set the power going to the module, in TX mode
  rf95.setTxPower(23, false);
}

void LoRanTX() // Transimits the Data via the LoRan Module
{
  Serial.print(F("Transmitting LoRan "));
  // Increment the WWVAM indicator if there is no data
  if (starttime < 1)
  {
    if (WWVAMTimeCode > 71){}
    else
    {
      WWVAMTimeCode += 1;
    }
  }
  else if (WWVAMTimeCode > 70)
  {
    WWVAMTimeCode = 70;
  }
  // Get WWV AM Time Data in variables
  uint8_t Wsec;
  uint8_t Wmin;
  uint8_t Whou;
  uint8_t Wdays;
  uint8_t Wmon;
  uint8_t Wyea;
  // If a full sync hasn't been established, set all values to dummy (99)
  if (starttime < 1)
  {
    Wsec = DUMMY;
    Wmin = DUMMY;
    Whou = DUMMY;
    Wdays = DUMMY;
    Wmon = DUMMY;
    Wyea = DUMMY;
  }
  // If there has been a full sync, set values to the current Arduino's set internal clock
  else
  {
    Wsec = second();
    Wmin = minute();
    Whou = hour();
    Wdays = day();
    Wmon = month();
    Wyea = (year() - 2000);
  }
  // Set the last time a sync occured for WWV AM
  uint8_t Wsyncmin = (now() - goodstuff) / 60;
  // See if the BPSK module has been forced to sync yet
  if (BPSKstart == 1)
  {
    // If there has been 3 passes of the TX code, reset all BPSK Variables, there is no valid sync currently
    if (firstSyncTimerAllowance == 2)
    {
      firstSyncTimerAllowance = 0;
      BPSKstart = 0;
      WWVBPSKTimeCode = 81;
    }
    // Otherwise, make sure that the packet shows that the BPSK data is good
    else
    {
      WWVBPSKTimeCode = 80;
      // Increment the BPSK indication sync timer so only 3 passes of the code happen
      firstSyncTimerAllowance++;
    }
  }
  // Get GPS Time Data in local variables
  uint8_t Gsec = GPS.seconds;
  uint8_t Gmin = GPS.minute;
  int Ghouint = GPS.hour - MST;
  if (Ghouint < 0)
  {
    Ghouint += 24;
  }
  uint8_t Ghou = Ghouint;
  uint8_t Gdays = GPS.day;
  uint8_t Gmon = GPS.month;
  uint8_t Gyea = GPS.year;
  Ghou += DAYLIGHT;
  // Get BPSK Time in local variables
  uint8_t Bsec = tohex(dt_array_global[DT_SECOND]);
  uint8_t Bmin = tohex(dt_array_global[DT_MINUTE]);
  uint8_t Bhou = tohex(dt_array_global[DT_HOUR]);
  uint8_t Bdays = tohex(dt_array_global[DT_DAY]);
  uint8_t Bmon = tohex(dt_array_global[DT_MONTH]);
  uint8_t Byea = tohex(dt_array_global[DT_YEAR]);
  // If the hour of the BPSK transmission is less than 7 AM during standard time or 6 AM during daylight savings time
  if (Bhou < (MST - DAYLIGHT))
  {
    // Shift the BPSK hour variable to the appropriate time in 24h format
    Bhou = 24 - (MST - DAYLIGHT) + Bhou; 
    // if it says that this is the 1st of the month, it isn't because MST/MDT hasn't reached the new day yet, so change the day and month accordingly
    if (Bdays == 1)
    {
      int monthsOfThirtyOne[7] = {1, 3, 5, 7, 8, 10, 12};
      for (int i = 0; i < 6; i++)
      {
        if (monthsOfThirtyOne[i] == Bmon)
        {
          Bdays = 31;
        }
      }
      if (Bdays != 31)
      {
        Bdays = 30;
      }
    }
    // If it isn't the first, you can just make the day go back by one
    else
    {
      Bdays--;
    }
  }
  // If UTC is more than 7/6 AM (depending on MST or MDT), then adjust the time accordingly
  else
  {
    Bhou = Bhou - MST + DAYLIGHT;
  }
  // Establish the time packet to be sent via LoRan, order is very important for the correct functioning of the RX module
  uint8_t timepacket[] = {WWVAMTimeCode, Whou, Wmin, Wsec, Wmon, Wdays, Wyea, Wsyncmin, 
                          GPSTimeCode, Ghou, Gmin, Gsec, Gmon, Gdays, Gyea, WWVBPSKTimeCode, 
                          Bhou, Bmin, Bsec, Bmon, Bdays, Byea};

  // Sent the time packet
  rf95.send(timepacket, sizeof(timepacket));

  // Wait to make sure the packet has been sent
  rf95.waitPacketSent();
}

int tohex(int decimalnum) // Process to change a decimal to a hexadecimal (needed for BPSK time transmission)
{
  int quotient = decimalnum;

  int remainder = quotient % 16;
  quotient = quotient / 16;
  quotient = quotient * 10;

  int fullnum = remainder + quotient;
  return fullnum;
}

// ------------------------------------------------------------------- DATA AQUISITION ----------------------------------------------------------------
void checkGPSData() // Check if there is good GPS data, also regulates data aquisition to one ping per second
{
  char c = GPS.read();
  if ((c) && (GPSECHO))
  if (GPS.newNMEAreceived()) 
      {
        if (!GPS.parse(GPS.lastNMEA()))
      return;  
      }
}

void checkRadioData() // Adds data to the frame, if all data is valid, turns the frame into time
{
  // If the frame index is where a marker should be
  if (frameIndex % 10 == 9)
  {
    // If the newest bit is a marker
    if (newBit == MRKBIT)
    {
      // Segement value of the corresponding index is either true or false (no errors or errors)
      seg[frameIndex / 10] = validSegment();
    }
  }
  // If the frame index is too large, start a new frame
  if (frameIndex > 59)
  {
    startNewFrame();
  }
  // If there are two marker bits in a row
  if ((newBit == MRKBIT) && (oldBit == MRKBIT))
  {
    // If the last full frame didn't have errors and had markers in the right spots
    if (validFrame())
    {
      // Decode time 
      getRadioTime();
      // OK to start displaying time now
      starttime++;
    }
    // Start new frame of data
    startNewFrame();
  }
  // If at the beginning of a frame, clear the segment array
  if (frameIndex == 0)
  {
    clearSegment();
  }
  // Set the frame value at this index to what the bit value is
  frame[frameIndex] = newBit;
  // Increment the index
  frameIndex++;
  // Keep track of last bit
  oldBit = newBit;
  // Reset current bit
  newBit = NOBIT;
}

void sync() // First sync for the clock, waits until a marker bit is aquired
{
  int synctimer = millis();
  // Initializes pulsewidth variable
  int pw;
  // Checks the pulsewidth until it is a marker
  do
  {
    pw = pulseIn(pin, LOW, 2000000) / 1000;
  }
  while (pw < 650 || pw > 900 && (millis() - synctimer > 300000));

  Serial.println(F("Achieved initial sync, waiting for full atomic sync"));
  // Delays next sample and initializes sample counter
  delay(200);
  samplecounter = 100;
}

void getRadioTime() // Decodes time from current frame
{
  // Initializes time variables
  const byte daysInMonth[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
                             // JanFebMarAprMayJunJulAugSepOctNovDec
  const int century=2000;
  leap=dy=hr=dst=leap=mn=0;
  yr=century;

  // Decodes minutes
  if (frame[1]==HIBIT) mn+=40;
  if (frame[2]==HIBIT) mn+=20;
  if (frame[3]==HIBIT) mn+=10;
  if (frame[5]==HIBIT) mn+=8;
  if (frame[6]==HIBIT) mn+=4;
  if (frame[7]==HIBIT) mn+=2;
  if (frame[8]==HIBIT) mn+=1;

  // Decodes Hours
  if (frame[12]==HIBIT) hr+=20;
  if (frame[13]==HIBIT) hr+=10;
  if (frame[15]==HIBIT) hr+=8;
  if (frame[16]==HIBIT) hr+=4;
  if (frame[17]==HIBIT) hr+=2;
  if (frame[18]==HIBIT) hr+=1;

  // Decodes Days
  if (frame[22]==HIBIT) dy+=200;
  if (frame[23]==HIBIT) dy+=100;
  if (frame[25]==HIBIT) dy+=80;
  if (frame[26]==HIBIT) dy+=40;
  if (frame[27]==HIBIT) dy+=20;
  if (frame[28]==HIBIT) dy+=10;
  if (frame[30]==HIBIT) dy+=8;
  if (frame[31]==HIBIT) dy+=4;
  if (frame[32]==HIBIT) dy+=2;
  if (frame[33]==HIBIT) dy+=1;

  // Decodes Years
  if (frame[45]==HIBIT) yr+=80; 
  if (frame[46]==HIBIT) yr+=40;
  if (frame[47]==HIBIT) yr+=20;
  if (frame[48]==HIBIT) yr+=10;
  if (frame[50]==HIBIT) yr+=8;
  if (frame[51]==HIBIT) yr+=4;
  if (frame[52]==HIBIT) yr+=2;
  if (frame[53]==HIBIT) yr+=1;
  // Leapyear Indicator
  if (frame[55]==HIBIT) leap+=1;
  // DST indicator
  if (frame[58]==HIBIT) dst+=1;

  // Convert from day of year to day of month, and the month
  mo=1;
  // For each month, starting with January
  while (1) 
  {
    // Get the number of days in the current month
    byte dim = daysInMonth[mo];
    // Adjust for leapyear, if needed
    if (mo == 2 && leap == 1) dim += 1;
    // If the month is correct, break
    if (dy <= dim) break;
    // Not correct, subtract all days in the month and increment the month forward
    dy -= dim; mo += 1;
  }

  // Set the time of the Arduino internal clock
  setTime(hr, mn, 0, dy, mo, yr);
  // Adjust it just slightly cause it always is off
  adjustTime(61);
  // Adjust hours for the timezone
  adjustTime(-3600 * MST);
  // If daylight savings time is happening, adjust accordingly
  if (dst == 1)
  {
    adjustTime(3600);
  }
  goodstuff = now();
}
// ---------------------------------------------------------------- SERIAL DISPLAY OF TIME --------------------------------------------------------
void WWVTime() // For displaying the WWV AM time on the serial monitor
{
  // Initialize variables to be used
  sec = second();
  min = minute();
  hou = hour();
  days = day();
  mon = month();
  yea = year();
  // Prints Time Type
  Serial.print(F("WWV AM: "));
  // Prints current hour on screen
  if (hou > 12)
  {
    Serial.print(hou - 12);
  }
  else
  {
    Serial.print(hou);
  }
  Serial.print(F(":"));
  // Prints current minute on screen
  if (min < 10)
  {
    Serial.print(F("0"));
    Serial.print(min);
  }
  else
  {
    Serial.print(min);
  }
  Serial.print(F(":"));
  // Prints curent second on screen
  if (sec < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(sec);
  Serial.print(F(" "));
  // Handles AM vs PM
  if (hou < 12)
  {
    Serial.print(F("AM"));
  }
  else
  {
    Serial.print(F("PM"));
  }
  // Handles the date in mn/dy/yr format
  Serial.print(F(" "));
  Serial.print(mon);
  Serial.print(F("/"));
  Serial.print(days);
  Serial.print(F("/"));
  Serial.print(yea);
  Serial.print(F(", "));
}

void showTime()
{
  // Show GPS time because it is synced by now
  GPSTime();
  // If a full WWV AM Sync has been achieved, display the data
  if (starttime > 0)
  {
    WWVTime();
  }
  // Otherwise, show that a sync hasn't been attained
  else
  {
    Serial.print(F("WWV AM sync not yet available "));
  }
}

void GPSTime() // For displaying GPS time on the serial monitor
{
  // Change all of the recieved data to data that can be processed
  int ghour = (GPS.hour) - MST;
  ghour = ghour + DAYLIGHT;
  int gmin = GPS.minute;
  int gsec = GPS.seconds;
  // FOLLOWS THE SAME PROTOCOL AS THE WWV AM DISPLAYING, SEE IT FOR REFERENCE IF NEEDED
  Serial.print(F(" GPS: "));

  Serial.print(int(GPS.month));
  Serial.print("/");
  Serial.print(int(GPS.day));
  Serial.print("/");
  Serial.print(int(GPS.year));
  Serial.print(" ");

  if (ghour > 12)
  {
    Serial.print(ghour - 12);
  }
  // Sometimes the time comes out as negative, so must adjust for this
  else if (ghour < 0)
  {
    Serial.print(ghour + 12);
  }
  else
  {
    Serial.print(ghour);
  }
  Serial.print(F(":"));
  if (gmin < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(gmin);
  Serial.print(F(":"));
  if (gsec < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(gsec);
  // If the time is negative or it is midnight, then it is AM
  if (ghour > 23 || ghour < -11)
  {
    Serial.println(F(" AM"));
  }
  else if (ghour > 11)
  {
    Serial.println(F(" PM"));
  }
  else if (ghour < 0)
  {
    Serial.println(F(" PM"));
  }
  else
  {
    Serial.println(F(" AM"));
  }

}
// ---------------------------------------------------------------- SETUP AND LOOP -----------------------------------------------------------------
void setup() {
  // Initializes pin
  pinMode(pin, INPUT);
  Serial.begin(9600);
  // Syncs for the second
  sync();
  // Initializes GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   
  GPS.sendCommand(PGCMD_ANTENNA);
  // Sets up timer interrupt
  interruptSetup();
  // // Initialize LoRan radio
  LoRanSetup();
  BPSKtimer = millis();
}

void loop() {
  // If there is incoming data
  checkGPSData();
  if (newBit != NOBIT)
  {
    // Displays time on the Serial Monitor
    showTime();
    // Sends the Data via LoRan
    LoRanTX();
    // Check the atomic data
    checkRadioData();
  }
  // If it has been more than 30 minutes since the last WWV AM sync and BPSK sync
  if ((now() - goodstuff) / 60 > 30 && (millis() - BPSKtimer > 1800000))
  {
    // As long as there has been at least one WWV AM Sync
    if (starttime > 0)
    {
      // Go through BPSK sync and time aquisition process THIS WILL COMPLETELY STOP ALL OTHER OPERATIONS, SO IF YOU DON'T WISH TO HAVE THIS FUNCTIONALITY, COMMENT IT ALL OUT
      mcu_init();
      // initialize gpios
      mcu_gpio_set_low(GPIO_EN);
      // Get BPSK Time
      current_timer_value_global = es100_receive(dt_array_global);
      Serial.print(F("received UTC time = 20"));
      Serial.print(dt_array_global[DT_YEAR], HEX);
      Serial.print(F(":"));
      Serial.print(dt_array_global[DT_MONTH], HEX);
      Serial.print(F(":"));
      Serial.print(dt_array_global[DT_DAY], HEX);
      Serial.print(F(" "));
      Serial.print(dt_array_global[DT_HOUR], HEX);
      Serial.print(F(":"));
      Serial.print(dt_array_global[DT_MINUTE], HEX);
      Serial.print(F(":"));
      Serial.println(dt_array_global[DT_SECOND], HEX);
      BPSKstart = 1;
      BPSKtimer = millis();
     }
  }
}

// --------------------------------------------------------------- ISR FUNCTION -------------------------------------------------------------------
ISR(TIMER1_COMPB_vect) // Timer1 Interrupt routine for WWV AM data aquisition process
{
  timerHandler();
}