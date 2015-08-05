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

#define VCC5V 2 //VCC Voltage
#define REF5V 3 //Ref Voltage

#define OB_RANGE PIN_F0  //Outboard Range
#define IB_RANGE PIN_F1  //Inboard Range

//Constants
const int baudRate = 9600;
const long clockSpeed = 3200000; //ADC Maximum speed
const int NO_OF_INPUT_CHANNELS = 8; 
const int bufferCapacity = 256;

//Global Variables
int input; //Variable to receive inputs during main loop
bool telementary = true; //Telementary display in main loop
bool graphicalDisplay = false; //Graphical display in main loop
bool voltageDisplay = false;
long int vectorCount = 0; //Number of vectors produced from start
//pthread_mutex_t buffer_lock =PTHREAD_MUTEX_INITIALIZER;
String mode; // Data mode 
long previousMillis = 0;
static int topicCount = 1;
unsigned long samplingInterval; 

//Struct representing a vector
struct vector
{
  int id; //Id of this vector
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

//Circular buffer that will be used in the loop
struct circular_buffer buf;

void setup() {
  
  Serial.begin(baudRate);
  Serial.flush();
  Serial.println("  -------------------------------------------");
  Serial.println("  |  Sunjammer MAGIC Flight Software Demo   |");
  Serial.println("  |  Version 0.0.1                          |");
  Serial.println("  |  2015-08-03                             |");
  Serial.println("  |  (C) Space Magnetometer Laboratory      |");
  Serial.println("  |  Omotola Babasola                       |");
  Serial.println("  |  Imperial College London                |");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  |              Commands                   |");
  Serial.println("  |         r - Read from buffer            |");
  Serial.println("  |         k - Read hosekeeping data       |");
  Serial.println("  |         h - Help                        |");
  Serial.println("  |         o - Telementary on              |");
  Serial.println("  |         f - Telementary off             |");
  Serial.println("  |         g - Graphical Telementary on    |");
  Serial.println("  |         n - Normal Telementary on       |");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");

  //Initialise the necessary pins
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_CLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);

  //Start the SPI library
  SPI.begin();
}

void loop() { 
  if (topicCount > 0) {
    Serial.println(".............Select a mode to start...............");
    Serial.println("    0 - Normal Data Mode    1 - Raw Data Mode     ");
    topicCount--;
    
  } 

  if (samplingInterval == 0) {
  input = Serial.read();
      switch (input) {
        case ((int) '1') : samplingInterval = 1000/16; mode = "Raw Data Mode"; break;
        case ((int) '0') : samplingInterval = 60000; mode = "Normal Data Mode"; break;
        default: ;
      }
  }
   
  if (samplingInterval != 0) { 

    unsigned long currentMillis = millis();
  
    if (currentMillis - previousMillis > samplingInterval) {
      buffer_write(readVector()); //Write a new vector to the buffer
      previousMillis = currentMillis;  
    }
  
    input = Serial.read(); 
  
    switch (input) {
      case ((int) 'r') : //buffer (R)ead
        buffer_read();
        telementary = false;
        Serial.println(" Press o to turn telementary back on ");
        break;
      case ((int) 'k') : //house(K)eeping
        showHousekeeping();
        telementary = false;
        Serial.println(" Press o to turn telementary back on ");
        break;
      case ((int) 'h') : //(H)elp
        help();
        telementary = false;
        Serial.println(" Press o to turn telementary back on ");
        break;
      case ((int) 'o') : //telementary (O)n
        telementary = true;
        break;
      case ((int) 'f') : //telementary o(F)f
        telementary = false;
        break;
      case ((int) 'g') : //(G)raphical display on
        graphicalDisplay = true;
        voltageDisplay = false;
        break;
      case ((int) 'n') : //(N)ormal display on
        graphicalDisplay = false;
        voltageDisplay = false;
        break;
      case ((int) 'v') : //(N)ormal display on
        voltageDisplay = true;
        graphicalDisplay = false;
        break;
      default : ;
    } 
  } 
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
    } else if (voltageDisplay) {
      printVoltages(vec);
    } else {
      printVector(vec);
    }
  }

  vectorCount++;
  return vec;
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
  Serial.print("    Outboard Range : "); 
  if (analogRead(OB_RANGE)==0) {Serial.println("LOW");} else {Serial.println("HIGH");}
  Serial.print("    Inboard Range  : "); 
  if (analogRead(IB_RANGE)==0) {Serial.println("LOW");} else {Serial.println("HIGH");}
  Serial.print("    Packet Count   : "); Serial.println(vectorCount);
  Serial.print("    VCC 5V         : "); Serial.println(analogRead(VCC5V));
  Serial.print("    REF 5V         : "); Serial.println(analogRead(REF5V));
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");

}

void help() {
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("                Commands              ");
  Serial.println("           r - Read from buffer       ");
  Serial.println("           k - Read hosekeeping data  ");
  Serial.println("           h - Help  ");
  Serial.println("           o - Telementary on  ");
  Serial.println("           f - Telementary off ");
  Serial.println("           g - Graphical Telementary on  ");
  Serial.println("           n - Normal Telementary on ");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
  Serial.println("  -------------------------------------------");
}

///////////////////////////////////////
/////////// BUFFER METHODS ////////////
///////////////////////////////////////

void buffer_write(struct vector vec) {

  //pthread_mutex_lock(&buffer_lock);
  acquire_buffer_lock();
  if (buf.count >=0 && buf.count < bufferCapacity) {
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
    Serial.println("Buffer write error: Buffer over capacity or negative valued");
  }
  release_buffer_lock();
  //pthread_mutex_unlock(&buffer_lock);
}

void buffer_read () {
  //pthread_mutex_lock(&buffer_lock);
  if (buf.count == 0) {
    Serial.println("Buffer currently empty");
  } else if (buf.count > 0 && buf.count <= bufferCapacity) {
    acquire_buffer_lock();
    struct vector vec = buf.storage[buf.count-1];
    buf.count--;
    release_buffer_lock();
    //pthread_mutex_unlock(&buffer_lock);
    printVector(vec);
    //return vec;
  } else {
    Serial.println("Buffer read error: Buffer over capacity or negative valued");
  }
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

/////////////////////////////////////////////////
////////////// DATA DEBUG MODES /////////////////
/////////////////////////////////////////////////

//Graphical
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

//Raw bits
void printVector(struct vector vec) {
  char tbs[64];
  sprintf(tbs, "%i)  %03X %03X %03X %03X   %03X %03X %03X %03X", 
          vec.id,vec.obx,vec.oby,vec.obz,vec.obt,vec.ibx,vec.iby,vec.ibz,vec.ibt);
  Serial.println(tbs);
}

//Display Voltages
void printVoltages(struct vector vec) {
  Serial.print(vec.id);
  Serial.print("   ");
  Serial.print(voltage(vec.obx), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.oby), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.obz), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.obt), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.ibx), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.iby), 4);
  Serial.print(" ");
  Serial.print(voltage(vec.ibz), 4);
  Serial.print(" ");
  Serial.println(voltage(vec.ibt), 4);
}

double voltage (unsigned int coord) {
  return ((5 / 4096) * (int) coord);
}

