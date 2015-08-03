//SPI is an arduino library that lets you communicate withh SPI devices`
#include <SPI.h>
#include <stdio.h>

//C:\WinAVR-20100110\bin\avrdude.exe -P COM -c arduino -p at90usb1286 -U flash:w:Sunjammer_FAS_0_2_2.cpp.hex:i

//Functions on the ADC and the corresponding piins they are wired to
#define SPI_SS     PIN_B0 //Slave select 
#define SPI_CLK    PIN_B1 //Serial clock
#define SPI_MOSI   PIN_B2 //Master Out Slave In
#define SPI_MISO   PIN_B3 //Master In Slave Out

//Constants
const int baudRate = 9600;
const long clockSpeed = 3200000;
const int NO_OF_INPUT_CHANNELS = 8;
const int bufferCapacity = 256;

//Global Variables
int counter = 0; //Number times loop is executed
int input;

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
    struct vector *storage = new struct vector [bufferCapacity];  // the storage of the buffer is an array of 16 vectors
    int count;  // number of items in the buffer
    int lock; // To prevent race conditions when reading or writing from the buffer
};

//Circular buffer that will be used
struct circular_buffer buf;

void setup() {
  
  Serial.begin(baudRate);
  Serial.flush();
  Serial.println("Readings from the Magnetometer");

  //Initialise the necessary pins
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_CLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);

  //Start the SPI library
  SPI.begin();
}

void loop() {
  buffer_write(readRawValues());
  
  input = Serial.read();
  if (input == (int) 'r') {
    buffer_read ();
  }
  
  delay(1000/16);
}

struct vector readRawValues(){
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

  //Disable SPI reading
  digitalWrite(SPI_SS,HIGH);
  SPI.endTransaction();

  //Print the readings from the vector
  //printVector(vec);

  return vec;
}

//Print the vector to Serial Monitor
void printVector(struct vector vec) {
  char tbs[64];
  sprintf(tbs, "%03X %03X %03X %03X   %03X %03X %03X %03X", 
          vec.obx,vec.oby,vec.obz,vec.obt,vec.ibx,vec.iby,vec.ibz,vec.ibt);
  Serial.println(tbs);
  
}

//Read the 
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

void showAverage() {
  struct vector vec;
  
  vec.obx = buffer_average(1);
  vec.oby = buffer_average(2);
  vec.obz = buffer_average(3);
  vec.obt = buffer_average(4);
  vec.ibx = buffer_average(5);
  vec.iby = buffer_average(6);
  vec.ibz = buffer_average(7);
  vec.ibt = buffer_average(0);

  //printVector(vec);
  printGraphicalVector(vec);
}

uint16_t buffer_average(int selector) {
  uint16_t sum = 0;
  
  switch (selector) {
    case 1:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].obx;
      }
      break;
    case 2:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].oby;
      }
      break;
    case 3:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].obz;
      }
      break;
    case 4:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].obt;
      }
      break;
    case 5:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].ibx;
      }
      break;
    case 6:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].iby;
      }
      break;
    case 7:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].ibz;
      }
      break;
    case 0:
      for (int i=0; i<bufferCapacity; i++) {
        sum += buf.storage[i].ibt;
      }
      break;
    default: ;
  }
    return sum/bufferCapacity;
}

///////////////////////////////////////
/////////// BUFFER METHODS ////////////
///////////////////////////////////////

void buffer_write(struct vector vec) {

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
}

struct vector buffer_read () {
  acquire_buffer_lock();
  struct vector vec = buf.storage[buf.count-1];
  buf.count--;
  release_buffer_lock();
  printGraphicalVector(vec);
  return vec;
}

void acquire_buffer_lock() {
  while (buf.lock == 1) {
    
  }
  buf.lock = 1;
}

void release_buffer_lock() {
  buf.lock = 0;
}

void printGraphicalVector(struct vector vec) {
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

