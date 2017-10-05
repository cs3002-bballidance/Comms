#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define STACK_SIZE          100
#define N                   11
#define VOLT_REF            5
#define SAMPLE_RATE         50  // 50hz sampling rate
#define DLPF_MODE           0   // MPU6050 on-board digital low pass filter
#define ACC1                8   // AD0 pin connect for accelerometer 1
#define ACC2                9   // AD0 pin connect for accelerometer 2
#define GYRO                10  // AD0 pin connect for gyro
#define INITIAL_NUM_SAMPLE  100
#define ACCEL_SENSITIVITY   8192
#define GYRO_SENSITIVITY    250
#define GRAVITY             9.81

/*** Constant Values ***/
//Comms Protocol Variables
const byte START_FLAG = 0xDD;   //start of every packet
const byte CONT_COMMS = 0xCC;   //ACK flag
const byte INBD_DATA = 0x1D;    //flag for Incoming data
const byte ERR_FLAG = 0xFD;     //error flag
const byte DESCRIPTOR_HANDSHAKE[] = {0xDD, 0x1C};
const byte DESCRIPTOR_ACK[] = {0xDD, 0xCC};

// pwr measurement variables
const int INA169_OUT = A0;    // Input pin for measuring Vout
const int VOLT_PIN = A1;      // Input pin for measuring Vin
const float RS = 0.09;        // Shunt resistor value (in ohms, calibrated)
const int RL = 10;            // Load resistor value (in ohms)

//RTOS Variables
int buffer[N];
int in, out;
int itemsInBuffer = 0;

// sensor variable
MPU6050 mpu; // default 0x68 i2c address on AD0 low 

long currmillis = 0;
long startmillis = 0;

int avgAcc1X = 0;
int avgAcc1Y = 0;
int avgAcc1Z = 0;
int avgAcc2X = 0;
int avgAcc2Y = 0;
int avgAcc2Z = 0;

SemaphoreHandle_t xSemaphoreProducerA1 = NULL;
SemaphoreHandle_t xSemaphoreProducerA2 = NULL;
SemaphoreHandle_t xSemaphoreProducerA3 = NULL;
SemaphoreHandle_t xSemaphoreProducerP = NULL;
SemaphoreHandle_t xSemaphoreBuffer = NULL;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);   // set I2C speed to 400kbps. Max speed 500kbps
  #endif

  pinMode(VOLT_PIN,INPUT);
  pinMode(INA169_OUT,INPUT);
  pinMode(ACC1, OUTPUT);
  pinMode(ACC2, OUTPUT);
  pinMode(GYRO, OUTPUT);
  digitalWrite(ACC1, HIGH);
  digitalWrite(ACC2, HIGH);
  digitalWrite(GYRO, HIGH);

  //setup serial
  Serial.begin(57600);
  
  // initialize device
  setSensors(ACC1, SAMPLE_RATE);
  setSensors(ACC2, SAMPLE_RATE);
  setSensors(GYRO, SAMPLE_RATE);
  
  setOffset(ACC1, -4516, 1386, 389, -67, -44, 258);   // calibrated offset for each sensors
  setOffset(ACC2, -1568, -590, 1163, -1257, -26, 15);
  setOffset(GYRO, -240, 2307, 2200, 89, 30, 45);

  calibrateInitial();
  
  //start comms
  bool isStarted;
  do{
    isStarted = startComms();
  }while(!isStarted); 
  
  //create semaphores
  xSemaphoreProducerA1 = xSemaphoreCreateBinary();
  xSemaphoreProducerA2 = xSemaphoreCreateBinary();
  xSemaphoreProducerA3 = xSemaphoreCreateBinary();
  xSemaphoreProducerP = xSemaphoreCreateBinary();
  xSemaphoreBuffer = xSemaphoreCreateMutex();

  //Give semaphores
  xSemaphoreGive((xSemaphoreBuffer));
  xSemaphoreGive((xSemaphoreProducerA1));
  xSemaphoreGive((xSemaphoreProducerA2));
  xSemaphoreGive((xSemaphoreProducerA3));
  xSemaphoreGive((xSemaphoreProducerP));
  
  //create tasks
  xTaskCreate(A1Task, "A1", 100, NULL, 3, NULL);
  xTaskCreate(A2Task, "A2", 100, NULL, 3, NULL);
  xTaskCreate(A3Task, "A3", 100, NULL, 3, NULL);
  xTaskCreate(PowTask, "P", 100, NULL, 3, NULL);
  xTaskCreate(CommTask, "C", 100, NULL, 2, NULL); 
}

void setSensors(int mpuNum, int sampleRate){
  mpuselect(mpuNum);
  mpu.initialize();
  mpu.setRate(sampleRate);                     //set rate to 50Hz for sampling
  mpu.setDLPFMode(DLPF_MODE);                   //set on-board digital low-pass filter configuration  
  mpu.setFullScaleAccelRange(1);
  /*
   * 0 = +/- 2g
   * 1 = +/- 4g
   * 2 = +/- 8g
   * 3 = +/- 16g
  */
  mpu.setFullScaleGyroRange(0);
  /*
   * FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
  */
}

void setOffset(int mpuNum, int accX, int  accY, int accZ, int gyroX, int gyroY, int gyroZ){
  mpuselect(mpuNum);
  mpu.setXAccelOffset(accX);
  mpu.setYAccelOffset(accY);
  mpu.setZAccelOffset(accZ);
  mpu.setXGyroOffset(gyroX);
  mpu.setYGyroOffset(gyroY);
  mpu.setZGyroOffset(gyroZ);
}

void calibrateInitial(){
  int16_t ax, ay, az;
  int16_t ax2, ay2, az2;

  float sumAcc1X = 0;
  float sumAcc1Y = 0;
  float sumAcc1Z = 0;
  float sumAcc2X = 0;
  float sumAcc2Y = 0;
  float sumAcc2Z = 0;

  for(int i = 0; i < INITIAL_NUM_SAMPLE; i++){
    mpuselect(ACC1);
    mpu.getAcceleration(&ax, &ay, &az);
    mpuselect(ACC2);
    mpu.getAcceleration(&ax2, &ay2, &az2);
    sumAcc1X += ax;
    sumAcc1Y += ay;
    sumAcc1Z += az;
    sumAcc2X += ax2;
    sumAcc2Y += ay2;
    sumAcc2Z += az2;
  }
  avgAcc1X = sumAcc1X / INITIAL_NUM_SAMPLE;
  avgAcc1Y = sumAcc1Y / INITIAL_NUM_SAMPLE;
  avgAcc1Z = sumAcc1Z / INITIAL_NUM_SAMPLE;
  avgAcc2X = sumAcc2X / INITIAL_NUM_SAMPLE;
  avgAcc2Y = sumAcc2Y / INITIAL_NUM_SAMPLE;
  avgAcc2Z = sumAcc2Z / INITIAL_NUM_SAMPLE;
}

void mpuselect(int numMpu){
  switch(numMpu){
    case ACC1:
      digitalWrite(ACC1, LOW);
      digitalWrite(ACC2, HIGH);
      digitalWrite(GYRO, HIGH);
      break;
    case ACC2:
      digitalWrite(ACC1, HIGH);
      digitalWrite(ACC2, LOW);
      digitalWrite(GYRO, HIGH);
      break;
    case GYRO:
      digitalWrite(ACC1, HIGH);
      digitalWrite(ACC2, HIGH);
      digitalWrite(GYRO, LOW);
      break;
  }
}

void loop() {
  //Do Nothing
}

// Task for accelerometer1
static void A1Task(void* pvParameters)
{
  int16_t ax, ay, az;
  while(1)
  {
    //take producer, buffer and empty
    if((xSemaphoreProducerA1 != NULL) && (xSemaphoreBuffer != NULL)){
      //P(empty); P(mutex);
      if((xSemaphoreTake(xSemaphoreProducerA1, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        mpuselect(ACC1);
        mpu.getAcceleration(&ax, &ay, &az);

        buffer[in] = (int)(((float)(ax-avgAcc1X)/ACCEL_SENSITIVITY)*GRAVITY);
//        Serial.print("in(acc1x): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;

        buffer[in] = (int)(((float)(ay-avgAcc1Y)/ACCEL_SENSITIVITY)*GRAVITY);
//        Serial.print("in(acc1y): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;

        buffer[in] = (int)(((float)(az-avgAcc1Z)/ACCEL_SENSITIVITY)*GRAVITY);
//        Serial.print("in(acc1z): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;
        itemsInBuffer += 3;
        xSemaphoreGive(xSemaphoreBuffer); //V(mutex);
      }else {
        //Serial.println("A1: Failed to grab semaphores and start task!");
      }
    } 
  }
}

// Task for accelerometer2
static void A2Task(void* pvParameters)
{
  int16_t ax, ay, az;
  while(1)
  {
    //take producer, buffer and empty
    if((xSemaphoreProducerA2 != NULL) && (xSemaphoreBuffer != NULL)){
      //P(empty); P(mutex);
      if((xSemaphoreTake(xSemaphoreProducerA2, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        mpuselect(ACC2);
        mpu.getAcceleration(&ax, &ay, &az);

        buffer[in] = (int)(((float)(ax-avgAcc2X)/ACCEL_SENSITIVITY)*GRAVITY);
//        Serial.print("in(acc2x): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;

        buffer[in] = (int)(((float)(ay-avgAcc2Y)/ACCEL_SENSITIVITY)*GRAVITY);
//        Serial.print("in(acc2y): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;

        buffer[in] = (int)(((float)(az-avgAcc2Z)/ACCEL_SENSITIVITY)*GRAVITY);
//        Serial.print("in(acc2z): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;
        itemsInBuffer += 3;
        xSemaphoreGive(xSemaphoreBuffer); //V(mutex);
      }else {
        //Serial.println("A2: Failed to grab semaphores and start task!");
      }
    }
  }
}

// Task for the gyrometer
static void A3Task(void* pvParameters)
{
  int16_t gx, gy, gz;
  while(1)
  {
    //take producer, buffer and empty
    if((xSemaphoreProducerA3 != NULL) && (xSemaphoreBuffer != NULL)){
      //P(empty); P(mutex);
      if((xSemaphoreTake(xSemaphoreProducerA3, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        mpuselect(GYRO);
        mpu.getRotation(&gx, &gy, &gz);

        buffer[in] = (int)((float)gx/GYRO_SENSITIVITY);
//        Serial.print("in(gyrox): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;

        buffer[in] = (int)((float)gy/GYRO_SENSITIVITY);
//        Serial.print("in(gyroy): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;

        buffer[in] = (int)((float)gz/GYRO_SENSITIVITY);
//        Serial.print("in(gyroz): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;
        itemsInBuffer += 3;
        xSemaphoreGive(xSemaphoreBuffer); //V(mutex);
      } else {
        //Serial.println("A3: Failed to grab semaphores and start task!");
      }
    }
  }
}

// power measurement task
static void PowTask(void* pvParameters)
{
  while(1)
  {
    //take producer, buffer and empty
    if((xSemaphoreProducerP != NULL) && (xSemaphoreBuffer != NULL)){
      //P(empty); P(mutex);
      if((xSemaphoreTake(xSemaphoreProducerP, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        int sumCount = 0;
        int voltIn = 0;     // Variable to store value from analog read
        int currIn = 0;   // Variable to store value from analog read
        
        float voltAvgVal = 0.0;
        float ina169AvgVal = 0.0;
        float current = 0.0;     // Calculated current value
        float voltage = 0.0;     // Calculated voltage value

        currIn = analogRead(INA169_OUT);
        voltIn = analogRead(VOLT_PIN);
        
        // Remap the ADC value into a voltage number (5V reference)
        ina169AvgVal = (currIn * VOLT_REF) / 1023.0;
        voltAvgVal = (voltIn * VOLT_REF) / 1023.0;

        current = ina169AvgVal / (RS * RL);
        voltage = voltAvgVal * 1.4706;
        
        buffer[in] = (int)(current * 1000);
//        Serial.print("in(A): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;

        buffer[in] = (int)(voltage*100);
//        Serial.print("in(V): ");
//        Serial.println(buffer[in]);
        in =(in+1) % N;
        itemsInBuffer += 2;
        xSemaphoreGive(xSemaphoreBuffer); //V(mutex);
      }else {
        //Serial.println("P: Failed to grab semaphores and start task!");
      }
    }
  }
}

static void CommTask(void* pvParameters)
{
  while(1)
  {
    if((xSemaphoreBuffer != NULL) && itemsInBuffer == 11){
      //P(full); P(mutex);
      if((xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        char i;
        //Start sending data
        //Serial.println("Data:");
        Serial.write(START_FLAG);
        Serial.write(INBD_DATA);
        Serial.write(0b11110001); //TODO: implement checksum
        for(i=0; i<N; i++){ //unload all 11 vars
          int val = buffer[out];
          out = (out+1) % N;
          Serial.write(highByte(val));
          Serial.write(lowByte(val)); 
//          Serial.print("out");
//          Serial.print(i);
//          Serial.println(val);
        }
        itemsInBuffer = 0;
        vTaskDelay(1); //give a delay to ensure things are sent
        xSemaphoreGive(xSemaphoreBuffer); //V(mutex);
        xSemaphoreGive((xSemaphoreProducerA1)); //give back binaries to restart cycle
        xSemaphoreGive((xSemaphoreProducerA2));
        xSemaphoreGive((xSemaphoreProducerA3));
        xSemaphoreGive((xSemaphoreProducerP));
      }else {
        //Serial.println("C: Failed to grab semaphores and start task!");
      }
    }
  }
}

/*** Utility Functions ***/
bool startComms(){
  //wait for handshake
  byte handshake[2];
  while(!Serial.available()); //wait
  if(Serial.available()){
    Serial.readBytes(handshake, sizeof(handshake));
  }

  if(buffersAreEqual(handshake, DESCRIPTOR_HANDSHAKE)){
    Serial.write(START_FLAG);
    Serial.write(CONT_COMMS);

    //wait for ACK
    byte ack[2];
    while(!Serial.available());
    if(Serial.available()){
      Serial.readBytes(ack, sizeof(ack));
    }
    return buffersAreEqual(ack, DESCRIPTOR_ACK);
    
  } else {
    Serial.write(START_FLAG);
    Serial.write(ERR_FLAG);
    return false;
  }
}

//function for checking messages
bool buffersAreEqual(byte buffer1[], const byte buffer2[])
{
  for (int i = 0; i < sizeof(buffer1); i++) {
    if (buffer1[i] != buffer2[i]) {
      return false;
    }
  }
  return true;
}
