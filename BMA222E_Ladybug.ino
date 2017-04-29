#include <Wire.h>

// Register Map BMA222E
#define BMA222E_BGW_CHIPID      0x00
#define BMA222E_ACCD_X_LSB      0x02
#define BMA222E_ACCD_X_MSB      0x03
#define BMA222E_ACCD_Y_LSB      0x04
#define BMA222E_ACCD_Y_MSB      0x05
#define BMA222E_ACCD_Z_LSB      0x06
#define BMA222E_ACCD_Z_MSB      0x07
#define BMA222E_ACCD_TEMP       0x08
#define BMA222E_INT_STATUS_0    0x09
#define BMA222E_INT_STATUS_1    0x0A
#define BMA222E_INT_STATUS_2    0x0B
#define BMA222E_INT_STATUS_3    0x0C
#define BMA222E_FIFO_STATUS     0x0E
#define BMA222E_PMU_RANGE       0x0F
#define BMA222E_PMU_BW          0x10
#define BMA222E_PMU_LPW         0x11
#define BMA222E_PMU_LOW_NOISE   0x12
#define BMA222E_ACCD_HBW        0x13
#define BMA222E_BGW_SOFTRESET   0x14
#define BMA222E_INT_EN_0        0x16
#define BMA222E_INT_EN_1        0x17
#define BMA222E_INT_EN_2        0x18
#define BMA222E_INT_MAP_0       0x19
#define BMA222E_INT_MAP_1       0x1A
#define BMA222E_INT_MAP_2       0x1B
#define BMA222E_INT_SRC         0x1E
#define BMA222E_INT_OUT_CTRL    0x20
#define BMA222E_INT_RST_LATCH   0x21
#define BMA222E_INT_0           0x22
#define BMA222E_INT_1           0x23
#define BMA222E_INT_2           0x24
#define BMA222E_INT_3           0x25
#define BMA222E_INT_4           0x26
#define BMA222E_INT_5           0x27
#define BMA222E_INT_6           0x28
#define BMA222E_INT_7           0x29
#define BMA222E_INT_8           0x2A
#define BMA222E_INT_9           0x2B
#define BMA222E_INT_A           0x2C
#define BMA222E_INT_B           0x2D
#define BMA222E_INT_C           0x2E
#define BMA222E_INT_D           0x2F
#define BMA222E_FIFO_CONFIG_0   0x30
#define BMA222E_PMU_SELF_TEST   0x32
#define BMA222E_TRIM_NVM_CTRL   0x33
#define BMA222E_BGW_SPI3_WDT    0x34
#define BMA222E_OFC_CTRL        0x36
#define BMA222E_OFC_SETTING     0x37
#define BMA222E_OFC_OFFSET_X    0x38
#define BMA222E_OFC_OFFSET_Y    0x39
#define BMA222E_OFC_OFFSET_Z    0x3A
#define BMA222E_TRIM_GP0        0x3B
#define BMA222E_TRIM_GP1        0x3C
#define BMA222E_FIFO_CONFIG_1   0x3E
#define BMA222E_FIFO_DATA       0x3F

#define BMA222E_ADDRESS  0x18  // if ADO is 0 (default)

#define myLed 13
#define intPin1 5
#define intPin2 3

bool newData = false;
bool newTap = false;

#define AFS_2G  0x02
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0C

#define BW_7_81Hz  0x08
#define BW_15_63Hz 0x09
#define BW_31_25Hz 0x0A
#define BW_62_5Hz  0x0B
#define BW_125Hz   0x0C
#define BW_250Hz   0x0D
#define BW_500Hz   0x0E
#define BW_1000Hz  0x0F

// Specify sensor full scale
uint8_t Ascale = AFS_2G, BW = BW_125Hz, tapStatus, tapType;

float aRes;      // scale resolutions per LSB for the sensor

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float pi = 3.141592653589793238462643383279502884f;
float ax, ay, az; // variables to hold latest sensor data values 
uint32_t delt_t = 0, count = 0; // used to control display output rate

  
void setup() {
  
  // Set up the INPUT/OUTPUT pins
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // active HIGH

  pinMode(intPin1, INPUT);
  pinMode(intPin2, INPUT);
  
  Serial.begin(115200);
  delay(4000);
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
 
  I2Cscan();

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("BMA222E accelerometer...");
  byte c = readByte(BMA222E_ADDRESS, BMA222E_BGW_CHIPID);  // Read CHIP_ID register for BMA222E
  Serial.print("BMA222E "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xF8, HEX);
 
  if (c == 0xF8) // CHIP_ID should always be 0xF8
  {  
    Serial.println("BMA222E is online...");
   
   getAres();                  // get sensor resolutions, only need to do this once
   selfTestBMA222E();          // perform sensor self test
   delay(1000);                // give some time to read the screen
   initBMA222E();              // initialize sensor for data acquisition
   fastCompensationBMA222E();  // calculate offset bias

   attachInterrupt(intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA222E
   attachInterrupt(intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA222E
  }
  
}


void loop() {
 
    if(newData == true) {  // On interrupt, read data
     newData = false;  // reset newData flag

     readBMA222EAccelData(accelCount);
     
    // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes/256.0f;  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes/256.0f;   
     az = (float)accelCount[2]*aRes/256.0f;  
    }

    if(newTap == true) {  // On interrupt, identify tap
      newTap = false;
 
      tapType = readByte(BMA222E_ADDRESS, BMA222E_INT_STATUS_0);

      if(tapType & 0x20) Serial.println("Single tap detected!");
      if(tapType & 0x10) Serial.println("Double tap detected!"); 
      
      tapStatus = readByte(BMA222E_ADDRESS, BMA222E_INT_STATUS_2);  // Read tap status register

      if(tapStatus & 0x80) {
        Serial.println("Tap is negative");
        }
      else {
        Serial.println("Tap is positive");
      }

       if(tapStatus & 0x40) {
        Serial.println("Tap is on x");
       }
       if(tapStatus & 0x20) {
        Serial.println("Tap is on y");
       }
       if(tapStatus & 0x10) {
        Serial.println("Tap is on z");
       }      
       
    }
  
    // Serial print and/or display at 1 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update serial once per half-second independent of read rate
   
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");

    tempCount = readBMA222EGyroTempData();  // Read the gyro adc values
    temperature = 0.5f * ((float) tempCount) / 256.0f + 23.0f; // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

    digitalWrite(myLed, !digitalRead(myLed));
    count = millis();  
    }

}

/* end of main program*/


/* Useful functions*/

void myinthandler1()
{
  newData = true;
}

void myinthandler2()
{
  newTap = true;
}

void getAres() {
  switch (Ascale)
  {
   // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 0.01563f;// per data sheet
          break;
    case AFS_4G:
          aRes = 0.03125f;
          break;
    case AFS_8G:
          aRes = 0.06250f;
          break;
    case AFS_16G:
          aRes = 0.125f;
          break;
  }
}

void initBMA222E()
{
   writeByte(BMA222E_ADDRESS,BMA222E_PMU_RANGE, AFS_2G);         // set full-scale range
   writeByte(BMA222E_ADDRESS,BMA222E_PMU_BW, BW);                // set bandwidth (and thereby sample rate) 
   writeByte(BMA222E_ADDRESS,BMA222E_INT_EN_1,  0x10);           // set data ready interrupt (bit 4) 
   writeByte(BMA222E_ADDRESS,BMA222E_INT_MAP_1, 0x01);           // map data ready interrupt to INT1 (bit 0) 
   writeByte(BMA222E_ADDRESS,BMA222E_INT_EN_0,  0x20 | 0x10);    // set single tap interrupt (bit 5) and double tap interrupt (bit 4)
   writeByte(BMA222E_ADDRESS,BMA222E_INT_MAP_2, 0x20 | 0x10);    // map single and double tap interrupts to INT2 (bits 4 and 5)
   writeByte(BMA222E_ADDRESS,BMA222E_INT_9, 0x0A);               // set tap threshold to 10 x 3.125% of full range
   writeByte(BMA222E_ADDRESS,BMA222E_INT_OUT_CTRL, 0x04 | 0x01); // interrupts push-pull, active HIGH (bits 0:3) 
}


void fastCompensationBMA222E()
{
     Serial.println("hold flat and motionless for bias calibration");
     delay(5000);
     
     uint8_t rawData[2];  // x/y/z accel register data stored here
     
     writeByte(BMA222E_ADDRESS,BMA222E_OFC_SETTING,0x20 | 0x01); // set target data to 0g, 0g, and +1 g, cutoff at 1% of bandwidth
     writeByte(BMA222E_ADDRESS,BMA222E_OFC_CTRL,0x20); // x-axis calibration
     while(!(0x10 & readByte(BMA222E_ADDRESS,BMA222E_OFC_CTRL))); // wait for calibration completion
     writeByte(BMA222E_ADDRESS,BMA222E_OFC_CTRL,0x40); // x-axis calibration
     while(!(0x10 & readByte(BMA222E_ADDRESS,BMA222E_OFC_CTRL))); // wait for calibration completion
     writeByte(BMA222E_ADDRESS,BMA222E_OFC_CTRL,0x60); // x-axis calibration
     while(!(0x10 & readByte(BMA222E_ADDRESS,BMA222E_OFC_CTRL))); // wait for calibration completion

     readBytes(BMA222E_ADDRESS, BMA222E_OFC_OFFSET_X, 2, &rawData[0]);
     int16_t offsetX = ((int16_t)rawData[1] << 8) | 0x00 ;
     Serial.print("x-axis offset = "); Serial.print((float)(offsetX)*1000.0f*aRes/256.0f, 1); Serial.println("mg");
     readBytes(BMA222E_ADDRESS, BMA222E_OFC_OFFSET_Y, 2, &rawData[0]);
     int16_t offsetY = ((int16_t)rawData[1] << 8) | 0x00 ;
     Serial.print("y-axis offset = "); Serial.print((float)(offsetY)*1000.0f*aRes/256.0f, 1); Serial.println("mg");
     readBytes(BMA222E_ADDRESS, BMA222E_OFC_OFFSET_Z, 2, &rawData[0]);
     int16_t offsetZ = ((int16_t)rawData[1] << 8) | 0x00 ;
     Serial.print("z-axis offset = "); Serial.print((float)(offsetZ)*1000.0f*aRes/256.0f, 1); Serial.println("mg");
}


void selfTestBMA222E()
{
    uint8_t rawData[2];  // x/y/z accel register data stored here

   writeByte(BMA222E_ADDRESS,BMA222E_PMU_RANGE, AFS_8G); // set full-scale range to 8G

  // x-axis test
   writeByte(BMA222E_ADDRESS,BMA222E_PMU_SELF_TEST, 0x10 | 0x04 | 0x01); // positive x-axis
   delay(100);
   readBytes(BMA222E_ADDRESS, BMA222E_ACCD_X_LSB, 2, &rawData[0]);
   int16_t posX = ((int16_t)rawData[1] << 8) | 0x00 ;

   writeByte(BMA222E_ADDRESS,BMA222E_PMU_SELF_TEST, 0x10 | 0x00 | 0x01); // negative x-axis
   delay(100);
   readBytes(BMA222E_ADDRESS, BMA222E_ACCD_X_LSB, 2, &rawData[0]);
   int16_t negX = ((int16_t)rawData[1] << 8) | 0x00;

   Serial.print("x-axis self test = "); Serial.print((float)(posX - negX)*1000.0f*aRes/256.0f, 1); Serial.println("mg, should be > 800 mg");
 
   // y-axis test
   writeByte(BMA222E_ADDRESS,BMA222E_PMU_SELF_TEST, 0x10 | 0x04 | 0x02); // positive y-axis
   delay(100);
   readBytes(BMA222E_ADDRESS, BMA222E_ACCD_Y_LSB, 2, &rawData[0]);
   int16_t posY = ((int16_t)rawData[1] << 8) | 0x00 ;

   writeByte(BMA222E_ADDRESS,BMA222E_PMU_SELF_TEST, 0x10 | 0x00 | 0x02); // negative y-axis
   delay(100);
   readBytes(BMA222E_ADDRESS, BMA222E_ACCD_Y_LSB, 2, &rawData[0]);
   int16_t negY = ((int16_t)rawData[1] << 8) | 0x00;
 
   Serial.print("y-axis self test = "); Serial.print((float)(posY - negY)*1000.0f*aRes/256.0f, 1); Serial.println("mg, should be > 800 mg");

   // z-axis test
   writeByte(BMA222E_ADDRESS,BMA222E_PMU_SELF_TEST, 0x10 | 0x04 | 0x03); // positive z-axis
   delay(100);
   readBytes(BMA222E_ADDRESS, BMA222E_ACCD_Z_LSB, 2, &rawData[0]);
   int16_t posZ = ((int16_t)rawData[1] << 8) | 0x00 ;

   writeByte(BMA222E_ADDRESS,BMA222E_PMU_SELF_TEST, 0x10 | 0x00 | 0x03); // negative z-axis
   delay(100);
   readBytes(BMA222E_ADDRESS, BMA222E_ACCD_Z_LSB, 2, &rawData[0]);
   int16_t negZ = ((int16_t)rawData[1] << 8) | 0x00;
 
   Serial.print("z-axis self test = "); Serial.print((float)(posZ - negZ)*1000.0f*aRes/256.0f, 1); Serial.println("mg, should be > 400 mg");

   writeByte(BMA222E_ADDRESS,BMA222E_PMU_SELF_TEST, 0x00); // disable self test
/* end of self test*/
}


void readBMA222EAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(BMA222E_ADDRESS, BMA222E_ACCD_X_LSB, 6, &rawData[0]);  // Read the 7 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | 0x00 ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | 0x00 ;  
  destination[2] = ((int16_t)rawData[5] << 8) | 0x00 ; 
}


int16_t readBMA222EGyroTempData()
{
  uint8_t temp = readByte(BMA222E_ADDRESS, BMA222E_ACCD_TEMP);  // Read the raw data register  
  return ((int16_t)temp << 8) | 0x00;  // Turn the 16-bit value signed temperature value
}


// I2C scan function
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
      error = Wire.transfer(address, NULL, 0, NULL, 0);
      
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

// I2C read/write functions for the BMA222E

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }
