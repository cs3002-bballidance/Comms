#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/*** Constant Values ***/

//Comms Protocol Variables
const byte START_FLAG = 0xDD; //start of every packet
const byte CONT_COMMS = 0xCC; //ACK flag
const byte INBD_DATA = 0x1D; //flag for Incoming data
const byte ERR_FLAG = 0xFD; //error flag

const byte DESCRIPTOR_HANDSHAKE[] = {0xDD, 0x1C};
const byte DESCRIPTOR_ACK[] = {0xDD, 0xCC};

//RTOS Variables
#define STACK_SIZE 100
#define N 11
int buffer[N];
int in, out;
int itemsInBuffer = 0;

// pwr measurement variables
#define VOLT_REF 5
#define NUM_SAMPLES 10

const int INA169_OUT = A0;    // Input pin for measuring Vout
const int VOLT_PIN = A1;      // Input pin for measuring Vin
const float RS = 0.09;        // Shunt resistor value (in ohms, calibrated)
const int RL = 10;            // Load resistor value (in ohms)

// ============== //
SemaphoreHandle_t xSemaphoreProducerA1 = NULL;
SemaphoreHandle_t xSemaphoreProducerA2 = NULL;
SemaphoreHandle_t xSemaphoreProducerA3 = NULL;
SemaphoreHandle_t xSemaphoreProducerP = NULL;
SemaphoreHandle_t xSemaphoreBuffer = NULL;

void setup() {
  //setup serial
  Serial.begin(9600);

  pinMode(VOLT_PIN,INPUT);
  pinMode(INA169_OUT,INPUT);
  
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

void loop() {
  //Do Nothing
}

static void A1Task(void* pvParameters)
{
  while(1)
  {
    //take producer, buffer and empty
    if((xSemaphoreProducerA1 != NULL) && (xSemaphoreBuffer != NULL)){
      //P(empty); P(mutex);
      if((xSemaphoreTake(xSemaphoreProducerA1, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        int val =  0xA101;
        int val2 = 0xA102;
        int val3 = 0xA103;
        //val1
        buffer[in] = val;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        //val2
        buffer[in] = val2;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        //val3
        buffer[in] = val3;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        itemsInBuffer += 3;
        xSemaphoreGive(xSemaphoreBuffer); //V(mutex);
      }else {
        //Serial.println("A1: Failed to grab semaphores and start task!");
      }
    } 
  }
}

static void A2Task(void* pvParameters)
{
  while(1)
  {
    //take producer, buffer and empty
    if((xSemaphoreProducerA2 != NULL) && (xSemaphoreBuffer != NULL)){
      //P(empty); P(mutex);
      if((xSemaphoreTake(xSemaphoreProducerA2, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        int val =  0xA204;
        int val2 = 0xA205;
        int val3 = 0xA206;
        //val1
        buffer[in] = val;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        //val2
        buffer[in] = val2;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        //val3
        buffer[in] = val3;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        itemsInBuffer += 3;
        xSemaphoreGive(xSemaphoreBuffer); //V(mutex);
      }else {
        //Serial.println("A2: Failed to grab semaphores and start task!");
      }
    }
  }
}

static void A3Task(void* pvParameters)
{
  while(1)
  {
    //take producer, buffer and empty
    if((xSemaphoreProducerA3 != NULL) && (xSemaphoreBuffer != NULL)){
      //P(empty); P(mutex);
      if((xSemaphoreTake(xSemaphoreProducerA3, portMAX_DELAY) == pdTRUE) && (xSemaphoreTake(xSemaphoreBuffer, 0) == pdTRUE)){
        int val =  0xA307;
        int val2 = 0xA308;
        int val3 = 0xA309;
        //val1
        buffer[in] = val;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        //val2
        buffer[in] = val2;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        //val3
        buffer[in] = val3;
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
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
        int val =  0xA40A;
        int val2 = 0xA40B;

        int sumCount = 0;
        int voltSumVal;     // Variable to store value from analog read
        int ina169SumVal;   // Variable to store value from analog read
        
        float voltAvgVal;
        float ina169AvgVal;
        float current;     // Calculated current value
        float voltage;     // Calculated voltage value

        bool continueSampling = true;
        long currMillis;
        long startMillis = millis();

        while(continueSampling){
          currMillis = millis();
          if ((currMillis - startMillis) >= 10){
            ina169SumVal += analogRead(INA169_OUT);
            voltSumVal += analogRead(VOLT_PIN);
            sumCount++;
            if (sumCount == NUM_SAMPLES){
              continueSampling = false;
            }
            startMillis = millis();
          }
        }

        // Remap the ADC value into a voltage number (5V reference)
        ina169AvgVal = (((float)ina169SumVal / NUM_SAMPLES) * VOLT_REF) / 1023.0;
        voltAvgVal = (((float)voltSumVal / NUM_SAMPLES) * VOLT_REF) / 1023.0;
        
        //val1
//        buffer[in] = val;
        buffer[in] = (int)(ina169AvgVal * 100);
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
        in =(in+1) % N;
        //val2
//        buffer[in] = val2;
        buffer[in] = (int)(voltAvgVal * 10);
        //Serial.print("in: ");
        //Serial.println(buffer[in]);
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
          Serial.write(val);
          //Serial.print("out:");
          //Serial.println(val);
        }
        itemsInBuffer = 0;
        //vTaskDelay(50); //give a delay to ensure things are sent
        vTaskDelay(1);
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
