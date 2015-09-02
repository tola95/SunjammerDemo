#include <SPI.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//Functions on the ADC and the corresponding pins they are connected to
#define SPI_SS     8 //Slave select 
#define SPI_CLK    9 //Serial clock
#define SPI_MOSI   10 //Master Out Slave In
#define SPI_MISO   11 //Master In Slave Out

#define VCC5V A3 //VCC Voltage
#define REF5V A2 //Ref Voltage

#define OB_RANGE A0  //Outboard Range
#define IB_RANGE A1  //Inboard Range

#define RED 28
#define ORANGE 27
#define YELLOW 26

//Constants
const int baudRate = 9600;
const long clockSpeed = 3200000; //ADC Maximum speed 
const int bufferCapacity = 128;

//Global Variables
char input; //Variable to receive inputs during main loop
bool telementary = true; 
bool graphicalDisplay = false; 
bool voltageDisplay = false;
unsigned int mode; //This variable is used to select which ISR to run 
int valid; //Used to count how many interrupts to skip before one is valid
uint8_t packetcount = 0;
uint8_t commandcount = 0;
uint8_t stat = 0;

//Struct representing a vector
struct vector
{
  unsigned int obx; //Outboard x coordinate
  unsigned int oby; //Outboard y coordinate
  unsigned int obz; //Outboard z coordinate
  unsigned int obt; //Outboard temperature
  unsigned int ibx; //Inboard x coordinate
  unsigned int iby; //Inboard y coordinate
  unsigned int ibz; //Inboard z coordinate
  unsigned int ibt; //Inboard temperature
};

//Struct representing circular buffer
struct circular_buffer 
{
    //the storage of the buffer is an array of vectors
    struct vector *storage = new struct vector [bufferCapacity];  
    volatile int count;  //number of items in the buffer
    volatile int write_ptr; //current write position in the buffer
    volatile int read_ptr; //current read position in the buffer
};

//Circular buffer that will be used in the loop
struct circular_buffer buf;

void setup() {
  
  Serial.begin(baudRate);
  
    Serial.println("  -------------------------------------------");
    Serial.println("  |  Sunjammer MAGIC Flight Software Demo   |");
    Serial.println("  |  Version 1.0.0                          |");
    Serial.println("  |  2015-08-03                             |");
    Serial.println("  |  (C) Space Magnetometer Laboratory      |");
    Serial.println("  |  Imperial College London                |");
    Serial.println("  -------------------------------------------");
    Serial.println("...........Select a mode to start...............");
    Serial.println("           0 - Normal Data Mode");
    Serial.println("           1 - Raw Data Mode");
    Serial.println("           2 - Telementery Mode 16Hz");
    Serial.println("           3 - Telementery Mode 0Hz");
    Serial.println("  -------------------------------------------");

  //Initialise the necessary pins
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_CLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(IB_RANGE, OUTPUT);
  pinMode(OB_RANGE, OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(ORANGE,OUTPUT);
  pinMode(YELLOW,OUTPUT);

  digitalWrite (IB_RANGE, HIGH);
  stat |= (1 << 5); 
  digitalWrite (OB_RANGE, HIGH);
  stat |= (1 << 6);
  digitalWrite(RED,HIGH);

  //Start the SPI library
  SPI.begin(); 
}

void loop() {
  input = Serial.read();
  
  switch (input) {
    case ('3') :  disable_sampling_interrupt();
                  interupt_configure_16Hz(); 
                  stat |= (1 << 4);
                  mode = 2; 
                  commandcount++;
                  digitalWrite(ORANGE,HIGH); 
                  digitalWrite(YELLOW,LOW);
                  enable_sampling_interrupt();
                  break;
    case ('2') :  disable_sampling_interrupt();
                  interupt_configure_16Hz(); 
                  stat |= (1 << 4);
                  mode = 1;
                  digitalWrite(YELLOW,HIGH);
                  digitalWrite(ORANGE,LOW);
                  commandcount++;
                  enable_sampling_interrupt();
                  break;
    case ('1') :  disable_sampling_interrupt();
                  interupt_configure_16Hz(); 
                  stat |= (1 << 4);
                  mode = 16;
                  digitalWrite(YELLOW,HIGH);
                  digitalWrite(ORANGE,LOW);
                  commandcount++;
                  enable_sampling_interrupt();
                  break;
    case ('0') :  disable_sampling_interrupt();
                  interupt_configure_0Hz();
                  stat &= ~(1 << 4);
                  mode = 0; //(1/60)
                  digitalWrite(ORANGE,HIGH); 
                  digitalWrite(YELLOW,LOW);
                  commandcount++;
                  enable_sampling_interrupt(); 
                  break;
    case ('r') :  //buffer (R)ead
                  buffer_read();
                  telementary = false;
                  Serial.println(" Press o to turn telementary back on ");
                  commandcount++;
                  break;
    case ('k') :  //house(K)eeping
                  showHousekeeping();
                  telementary = false;
                  Serial.println(" Press o to turn telementary back on ");
                  commandcount++;
                  break;
    case ('h') :  //(H)elp
                  help();
                  telementary = false;
                  Serial.println(" Press o to turn telementary back on ");
                  commandcount++;
                  break;
    case ('o') :  //telementary (O)n
                  telementary = true;
                  commandcount++;
                  break;
    case ('f') :  //telementary o(F)f
                  telementary = false;
                  commandcount++;
                  break;
    case ('g') :  //(G)raphical display on
                  graphicalDisplay = true;
                  voltageDisplay = false;
                  commandcount++;
                  break;
    case ('n') :  //(N)ormal display on
                  graphicalDisplay = false;
                  voltageDisplay = false;
                  commandcount++;
                  break;
    case ('v') :  //(N)ormal display on
                  voltageDisplay = true;
                  graphicalDisplay = false;
                  commandcount++;
                  break;
    case ('d') :  //(N)ormal display on
                  dump();
                  telementary = false;
                  Serial.println(" Press o to turn telementary back on ");
                  commandcount++;
                  break; 
    case ('4') :  //Switch inboard range
                  pin_switch(IB_RANGE);
                  if (digitalRead(IB_RANGE)==0) {stat &= ~(1 << 5);} 
                  else {stat |= (1 << 5);}
                  commandcount++;
                  break;
    case ('5') :  //Switch outboard range
                  pin_switch(OB_RANGE);
                  if (digitalRead(OB_RANGE)==0) {stat &= ~(1 << 6);} 
                  else {stat |= (1 << 6);}
                  commandcount++;
                  break;

    default    :  break; 
  }
}

//Interrupt service routine attached to the Timer Compare A vector
ISR(TIMER1_COMPA_vect){
  switch (mode) {
    case (0)  : ISR_0(); break;
    case (1)  : ISR_1(); break;
    case (2)  : ISR_2(); break;
    case (16) : ISR_16(); break;
    default   : break;
  }
}

//ISR for mode 0 (Normal Data Mode)
void ISR_0 () {
  //This ISR is valid in normal data mode every 15 interrupts to achieve the 1/60 Hz rate
  if (valid == 15 ) { 
    SPIRead();
    valid = 0;
  }
  valid++;  
}

//ISR for mode 1 (Packet Telementery mode)
void ISR_1 () {
  //This ISR is valid in normal data mode every 16 interrupts to achieve the 1 Hz rate
  if (valid == 16) {
    printPacket();
    valid = 0;
  }
  SPIRead();
  valid++;
}

void ISR_2 () {
  //This ISR is valid in normal data mode every 960 interrupts to achieve the 1/60 Hz rate
  if (valid == 960) {
    printPacket();
    valid = 0;
  }
  SPIRead();
  valid++;
}

//ISR for mode 16 (Raw Data Mode)
void ISR_16() {
  SPIRead();
}

void SPIRead() {
  struct vector vec;

    //Enable SPI reading
    SPI.beginTransaction(
      SPISettings(clockSpeed, MSBFIRST, SPI_MODE3)
    );  
    digitalWrite(SPI_SS,LOW);

    vec.obx = getSPIValue(1);
    vec.oby = getSPIValue(2);
    vec.obz = getSPIValue(3);
    vec.obt = getSPIValue(4);
    vec.ibx = getSPIValue(5);
    vec.iby = getSPIValue(6);
    vec.ibz = getSPIValue(7);
    vec.ibt = getSPIValue(0);
  
    //Disable SPI reading
    digitalWrite(SPI_SS,HIGH);
    SPI.endTransaction();

    //Print the readings from the vector if in the right mode
    switch (mode) {
      case (1) :  break;
      case (2) :  break;
      default  :  if (telementary) {
                    if (graphicalDisplay) {
                      printGraphicalVector(vec);
                    } else if (voltageDisplay) {
                      printVoltages(vec);
                    } else {
                      printVector(vec);
                    }
                  }
    }

    buffer_write(vec);
}
//Read value at a particular input channel on ADC
unsigned int getSPIValue(int inputChannel) {
  
  /* The ADC 128S022 uses bits 5,4 and 3 to select channels hence the '<<3' convention
   * Transfer twice as each transfer reads 8 bits. First transfer collects the msb, 
   * second collects the lsb.
   */
   
  uint16_t lsb,msb;
  unsigned int coordinate;
  
  msb = SPI.transfer(inputChannel << 3); 
  lsb = SPI.transfer(inputChannel << 3);
  coordinate = (msb << 8) | lsb;
  
  return coordinate;
}

//Housekeeping data
void showHousekeeping() {
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("             STATUS");
  Serial.print("    Time (ms)      : "); Serial.println(millis());
  Serial.print("    Mode           : "); Serial.println(mode);
  Serial.print("    Frequency      : "); Serial.println(mode);
  Serial.print("    Outboard Range : "); 
  if (digitalRead(OB_RANGE)==0) {Serial.println("LOW");} else {Serial.println("HIGH");}
  Serial.print("    Inboard Range  : "); 
  if (digitalRead(IB_RANGE)==0) {Serial.println("LOW");} else {Serial.println("HIGH");}
  Serial.print("    VCC 5V         : "); Serial.println(analogRead(VCC5V));
  Serial.print("    REF 5V         : "); Serial.println(analogRead(REF5V));
  Serial.print("    Read Position  : "); Serial.println(buf.read_ptr);
  Serial.print("    Write Position : "); Serial.println(buf.write_ptr);
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");

}

void help() {
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("                Commands");
  Serial.println("           r - Read from buffer");
  Serial.println("           k - Read hosekeeping data");
  Serial.println("           h - Help");
  Serial.println("           o - Telementary on");
  Serial.println("           f - Telementary off");
  Serial.println("           g - Graphical Display on");
  Serial.println("           n - Normal Telementary on ");
  Serial.println("           d - Data dump");
  Serial.println("           v - Voltage Display on");
  Serial.println("           0 - Normal Data Mode");
  Serial.println("           1 - Raw Data Mode");
  Serial.println("           2 - Telementery Mode 16Hz");
  Serial.println("           3 - Telementery Mode 0Hz");
  Serial.println("           4 - Switch Inboard Range");
  Serial.println("           5 - Switch Outboard Range");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
}

///////////////////////////////////////
/////////// BUFFER METHODS ////////////
///////////////////////////////////////

void buffer_write(struct vector vec) {
  if (buf.count < 0) {
    Serial.println("Buffer error: Negative count"); 
  } else if (buf.count > bufferCapacity) {
    Serial.println("Buffer error: Count over capacity");
  } else {
    buf.storage[buf.write_ptr] = vec;  //Write vector to buffer
    buf.write_ptr = (buf.write_ptr + 1) % bufferCapacity; //Increase write pointer value
  }
  
  //Increase the number of items in buffer if under capacity
  if (buf.count < bufferCapacity) { 
    buf.count++;
  }
  //Set read pointer to write pointer location as that is now the least recent value
  if (buf.count == bufferCapacity) {
    buf.read_ptr = buf.write_ptr;
  }
}

struct vector buffer_read () {
  struct vector readvec;
  
  if (buf.count < 0) {
    Serial.println("Buffer error: Negative count"); 
  } else if (buf.count > bufferCapacity) {
    Serial.println("Buffer error: Count over capacity");
  } else if (buf.count == 0) {
    Serial.println("Buffer currently empty");
  } else {
    readvec = buf.storage[buf.read_ptr]; //Read value from buffer
    buf.read_ptr = (buf.read_ptr + 1) % bufferCapacity; //Increase pointer
    buf.count--; //Reduce number of items in buffer
    printVector(readvec); //Print the vector
  } 

  return readvec;
}

//Same as buffer_read but without the printing. Necessary in packet telementery modes
struct vector buffer_read_v2 () {
  struct vector readvec;
  
  if (buf.count < 0) {
    Serial.println("Buffer error: Negative count"); 
  } else if (buf.count > bufferCapacity) {
    Serial.println("Buffer error: Count over capacity");
  } else if (buf.count == 0) {
    Serial.println("Buffer currently empty");
  } else {
    readvec = buf.storage[buf.read_ptr];
    buf.read_ptr = (buf.read_ptr + 1) % bufferCapacity;
    buf.count--;
  } 

  return readvec;
}

/////////////////////////////////////////////////
////////////// DATA DEBUG MODES /////////////////
/////////////////////////////////////////////////

//Graphical
void printGraphicalVector(struct vector vec) {
  graphical(vec.obx);
  Serial.print(" ");
  graphical(vec.oby);
  Serial.print(" ");
  graphical(vec.obz);
  Serial.print(" ");
  graphical(vec.obt);
  Serial.print("  ");
  graphical(vec.ibx);
  Serial.print(" ");
  graphical(vec.iby);
  Serial.print(" ");
  graphical(vec.ibz);
  Serial.print(" ");
  graphical(vec.ibt);
  Serial.println();
}

//Helper method for graphical
//Prints the graphical representation of a coordinate
void graphical(unsigned int coord) {
  // 586 is the lowest divisor that will give a short enough for loop to fit within ISR
  int x = coord/586; 
  for (int i=0; i<x; i++) {
    Serial.print(".");
  }
  Serial.print("x");
  for (int i=x+1; i<(0xFFF/586); i++) {
    Serial.print(".");
  }
}

//Raw bits
void printVector(struct vector vec) {
  char tbs[64];
  sprintf(tbs, "%04i %04i %04i %04i   %04i %04i %04i %04i", 
          vec.obx,vec.oby,vec.obz,vec.obt,vec.ibx,vec.iby,vec.ibz,vec.ibt);
  Serial.println(tbs);
}

//Packet format for vectors
void packetFormat(struct vector vec) {
  char tbs[64];
  sprintf(tbs, "%03x%03x%03x%03x%03x%03x%03x%03x", 
          vec.obx,vec.oby,vec.obz,vec.obt,vec.ibx,vec.iby,vec.ibz,vec.ibt);
  Serial.print(tbs);
}

//Packet format for 8 bit values
void packetFormat(uint8_t coord) {
  char tbs[4];
  sprintf(tbs, "%02x", coord);
  Serial.print(tbs);
}

//Packet format for voltages
void packetFormat_Voltages(uint16_t coord) {
  char tbs[8];
  sprintf(tbs, "%04x", coord);
  Serial.print(tbs);
}

//Format in which the packets are printed
void printPacket() {
  packetFormat(packetcount);
  packetFormat(stat);
  packetFormat(commandcount);
  Serial.print(" ");
  for (int i=0; i<16; i++) {
    packetFormat(buffer_read_v2());
  }
  Serial.print(" ");
  packetFormat_Voltages(analogRead(VCC5V));
  packetFormat_Voltages(analogRead(REF5V));
  Serial.println();
  packetcount++;
}

//Display Voltages
void printVoltages(struct vector vec) {
  Serial.print(voltage(vec.obx), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.oby), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.obz), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.obt), 4);
  Serial.print("  ");
  Serial.print(voltage(vec.ibx), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.iby), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.ibz), 4);
  Serial.print(" ");
  Serial.println(voltage(vec.ibt), 4);
}

double voltage (unsigned int coord) {
  return (((float) coord / (float) 4096 ) * 5);
}

//////////////////////////////////////////////////
/////////// INTERRUPT CONFIGURATIONS /////////////
//////////////////////////////////////////////////

//Useful information about AVR timers can be found here
//http://www.avrbeginners.net/architecture/timers/timers.html

//Configure interrupts to occur at 16 Hz
void interupt_configure_16Hz() {

  TCCR1A=0;
  //Set clock prescaler to 1024 and use CTC Mode
  TCCR1B=(1<<CS12) | (1<<CS10) | (1<<WGM12);
  
  // Set the OCR A to 0x03D4 
  OCR1AH=0x03;
  OCR1AL=0xD4;
}

//Configure interrupts to occur at 1/60 Hz
void interupt_configure_0Hz() {
  
  TCCR1A=0;
  //Set clock prescaler to 1024 and use CTC Mode
  TCCR1B=(1<<CS12) | (1<<CS10) | (1<<WGM12);
  
  // Set the OCR A to 0xF428 
  OCR1AH=0xF4;
  OCR1AL=0x28;
}
 
void enable_sampling_interrupt() {
  // Enable the Output Compare A match interrupt
  TIMSK |=(1<<OCIE1A);
}

void disable_sampling_interrupt() {
  // Disable the Output Compare A match interrupt
  TIMSK &=~(1<<OCIE1A);
}

//Dump a whole seconds worth of data at once
void dump() {
  for (int i = 0; i < 16; i++) {
    buffer_read();
  }
}

///////////////// SWITCHING RANGES ///////////////////

void pin_switch(int pin) {
  int val = digitalRead(pin);
  digitalWrite(pin, (1-val));
}

