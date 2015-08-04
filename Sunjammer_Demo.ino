//SPI is an arduino library that lets you communicate withh SPI devices`
#include <SPI.h>
#include <stdio.h>
//#include <pthread.h>

//C:\WinAVR-20100110\bin\avrdude.exe -P COM -c arduino -p at90usb1286 -U flash:w:Sunjammer_FAS_0_2_2.cpp.hex:i

//Functions on the ADC and the corresponding piins they are wired to
#define SPI_SS     PIN_B0 //Slave select 
#define SPI_CLK    PIN_B1 //Serial clock
#define SPI_MOSI   PIN_B2 //Master Out Slave In
#define SPI_MISO   PIN_B3 //Master In Slave Out

#define VCC5V 2
#define REF5V 3

#define OB_RANGE PIN_F0
#define IB_RANGE PIN_F1

//Constants
const int baudRate = 9600;
const long clockSpeed = 3200000;
const int NO_OF_INPUT_CHANNELS = 8;
const int bufferCapacity = 256;

//Global Variables
int counter = 0; //Number times loop is executed
int input;
bool telementary = true;
bool graphicalDisplay = false;
int vectorCount = 0;
//pthread_mutex_t buffer_lock =PTHREAD_MUTEX_INITIALIZER;

// Status values
int mode = 1;
int packet_count;
byte command_count;
unsigned int vcc_5v;
unsigned int ref_5v;

//Struct representing a vector
struct vector
{
  int id;           //Id of this vector
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
    struct vector *storage = new struct vector [bufferCapacity];  // the storage of the buffer is an array of 16 vectors
    int count;  // number of items in the buffer
    int lock; // To prevent race conditions when reading or writing from the buffer
};

//Circular buffer that will be used
struct circular_buffer buf;

void setup() {
  
  Serial.begin(baudRate);
  Serial.flush();
  Serial.println("  -------------------------------------- ");
  Serial.println("  |  Sunjammer MAGIC Flight Software    |");
  Serial.println("  |  Version 0.0.1                      |");
  Serial.println("  |  2015-08-03                         |");
  Serial.println("  |  (C) Space Magnetometer Laboratory  |");
  Serial.println("  |  Imperial College London            |");
  Serial.println("  ---------------------------------------");
  Serial.println("                Commands              ");
  Serial.println("           r - Read from buffer       ");
  Serial.println("           k - Read hosekeeping data  ");
  Serial.println("           h - Help  ");
  Serial.println("           o - Telementary on  ");
  Serial.println("           f - Telementary off ");
  Serial.println("           g - Graphical Telementary on  ");
  Serial.println("           n - Graphical Telementary off ");

  //Initialise the necessary pins
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_CLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);

  //Start the SPI library
  SPI.begin();
}

void loop() {
  buffer_write(readVector());
  
  input = Serial.read();
  switch (input) {
    case ((int) 'r') :
      buffer_read();
      telementary = false;
      Serial.println(" Press o to turn telementary back on ");
      break;
    case ((int) 'k') :
      showHousekeeping();
      telementary = false;
      Serial.println(" Press o to turn telementary back on ");
      break;
    case ((int) 'h') :
      help();
      telementary = false;
      Serial.println(" Press o to turn telementary back on ");
      break;
    case ((int) 'o') :
      telementary = true;
      break;
    case ((int) 'f') :
      telementary = false;
      break;
    case ((int) 'g') :
      graphicalDisplay = true;
      break;
    case ((int) 'n') :
      graphicalDisplay = false;
      break;
    default : ;
  }
  
  delay(1000/16); //Dalay for 1/16 of a second
}

struct vector readVector(){
  struct vector vec;

  //Enable SPI reading
  SPI.beginTransaction(
    SPISettings(clockSpeed, MSBFIRST, SPI_MODE3)
  );  
  digitalWrite(SPI_SS,LOW);

  //Populate the vector member variables
  vec.obx = getSPIValue(1);
  vec.oby = getSPIValue(2);
  vec.obz = getSPIValue(3);
  vec.obt = getSPIValue(4);
  vec.ibx = getSPIValue(5);
  vec.iby = getSPIValue(6);
  vec.ibz = getSPIValue(7);
  vec.ibt = getSPIValue(0);
  vec.id = vectorCount;

  //Disable SPI reading
  digitalWrite(SPI_SS,HIGH);
  SPI.endTransaction();

  //Print the readings from the vector
  if (telementary) {
    if (graphicalDisplay) {
      printGraphicalVector(vec);
    } else {
      printVector(vec);
    }
  }

  vectorCount++;
  return vec;
}

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

void showHousekeeping() {

  Serial.println("         STATUS");
  Serial.print("    Time (ms)      : "); Serial.println(millis());
  Serial.print("    Mode           : "); Serial.println(mode);
  Serial.print("    Outboard Range : "); 
  if (analogRead(OB_RANGE)==0) {Serial.println("LOW");} else {Serial.println("HIGH");}
  Serial.print("    Inboard Range  : "); 
  if (analogRead(IB_RANGE)==0) {Serial.println("LOW");} else {Serial.println("HIGH");}
  Serial.print("    Packet Count   : "); Serial.println(packet_count);
  Serial.print("    VCC 5V         : "); Serial.println(analogRead(VCC5V));
  Serial.print("    REF 5V         : "); Serial.println(analogRead(REF5V));

}

void help() {
  Serial.println("                Commands              ");
  Serial.println("           r - Read from buffer       ");
  Serial.println("           k - Read hosekeeping data  ");
  Serial.println("           h - Help  ");
  Serial.println("           o - Telementary on  ");
  Serial.println("           f - Telementary off ");
  Serial.println("           g - Graphical Telementary on  ");
  Serial.println("           n - Graphical Telementary off ");
}

///////////////////////////////////////
/////////// BUFFER METHODS ////////////
///////////////////////////////////////

void buffer_write(struct vector vec) {

  //pthread_mutex_lock(&buffer_lock);
  acquire_buffer_lock();
  if (buf.count < bufferCapacity) {
    for (int i = buf.count -1; i>=0; i--) {
      buf.storage[i+1] = buf.storage[i];
    }
    buf.count++;
    buf.storage[0] = vec;
  } else if (buf.count == bufferCapacity) {
    for (int i = (bufferCapacity - 2); i>=0; i--) {
      buf.storage[i+1] = buf.storage[i];
    }
    buf.storage[0] = vec;
  } else {
    //This should never be reached
    Serial.println("Buffer count error. Capacity Exceeded");
  }
  release_buffer_lock();
  //pthread_mutex_unlock(&buffer_lock);
}

struct vector buffer_read () {
  //pthread_mutex_lock(&buffer_lock);
  acquire_buffer_lock();
  struct vector vec = buf.storage[buf.count-1];
  buf.count--;
  release_buffer_lock();
  //pthread_mutex_unlock(&buffer_lock);
  printVector(vec);
  return vec;
}

//Spin locks protecting buffer
void acquire_buffer_lock() {
  while (buf.lock == 1) {
  }
  buf.lock = 1;
}

void release_buffer_lock() {
  buf.lock = 0;
}

////////////////////////////////////////////////
////////////// PRINTING METHODS ////////////////
////////////////////////////////////////////////

void printGraphicalVector(struct vector vec) {
  Serial.print(vec.id);
  Serial.print("   ");
  Serial.print(graphical(vec.obx));
  Serial.print(" ");
  Serial.print(graphical(vec.oby));
  Serial.print(" ");
  Serial.print(graphical(vec.obz));
  Serial.print(" ");
  Serial.print(graphical(vec.obt));
  Serial.print(" ");
  Serial.print(graphical(vec.ibx));
  Serial.print(" ");
  Serial.print(graphical(vec.iby));
  Serial.print(" ");
  Serial.print(graphical(vec.ibz));
  Serial.print(" ");
  Serial.println(graphical(vec.ibt));
}

String graphical(unsigned int coord) {
  int x = coord/400;
  String str = "";
  for (int i=0; i<x; i++) {
    str.concat(".");
  }
  str.concat("x");
  for (int i=x+1; i<(0xFFF/400); i++) {
    str.concat(".");
  }
  return str;
}

//Print the vector to Serial Monitor
void printVector(struct vector vec) {
  char tbs[64];
  sprintf(tbs, "%X)  %03X %03X %03X %03X   %03X %03X %03X %03X", 
          vec.id,vec.obx,vec.oby,vec.obz,vec.obt,vec.ibx,vec.iby,vec.ibz,vec.ibt);
  Serial.println(tbs);
}
