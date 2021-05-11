#include <ArduinoBLE.h>                 // Include the Arduino BLE Library       
#include <Arduino_LSM9DS1.h>            // Using IMU Sensor <Accelerometer, Gyroscope and Magneto meter>
                                        // 

static const char* hello = "Hello World!";

BLEService arduinoService("1101");  // User defined service

BLEStringCharacteristic helloCharacteristic("00002A3D-0000-1000-8000-00805f9b34fb", BLERead | BLENotify | BLEBroadcast ,15);
BLEIntCharacteristic RSSICharacteristic( "0000A666-0000-1000-8000-00805f9b34fb", BLERead | BLENotify );
BLEUnsignedCharCharacteristic batteryLevelCharacteristic("00002101-0000-1000-8000-00805f9b34fb", BLERead | BLENotify);

unsigned batteryLevel = 0;
int rssiValue = 0;

// Sleep Mode variables
const long interval = 1000 * 20;         // interval at which to sleep (milliseconds)

// Update - Dhruv (17-03-2021)
// Structure to store two different types as return from the SleepMode and ActiveMode functions
struct ArduinoStatus{
  volatile int ArduinoMode;
  volatile int inMotion;  
};

// Variable definition for the logic to check if the arduino is moving or not
float x, y, z;
static float xPrev = 0.0;
static float yPrev = 0.0;
static float zPrev = 0.0;
bool dx = false;
bool dy = false;
bool dz = false;

struct ArduinoStatus status;

bool inMotion = false;
float sensitivity = 0.008;
int ArduinoMode = 0;

// Variable declarations for timer
unsigned long diffMillis = 0;
unsigned long currentMillis = 0;
unsigned long startMillis = 0;

/*
 * LEDS
 */
//void connectedLight() 
//{
//  digitalWrite(LEDR, HIGH);
//  digitalWrite(LEDG, LOW);
//}
//
//void disconnectedLight() 
//{
//  digitalWrite(LEDR, LOW);
//  digitalWrite(LEDG, HIGH);
//}
//
void initSerialCommunication()
{
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);
}

void setupBLEService()
{
  BLE.setLocalName("Nano33BLE");  // Set name for connection
  BLE.setAdvertisedService(arduinoService); // Advertise service
  arduinoService.addCharacteristic(helloCharacteristic); // Add characteristic to service
  arduinoService.addCharacteristic(RSSICharacteristic);
  arduinoService.addCharacteristic(batteryLevelCharacteristic);
  BLE.addService(arduinoService); // Add service
}

//---------------------------------------------
// Update - Dhruv (17-03-2021)
// Setting up service for the IMU
void setupIMUService() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

//---------------------------------------------

void setup() 
{
  initSerialCommunication();  

//  disconnectedLight();//pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin

  if (!BLE.begin()) {   // initialize BLE
    Serial.println("starting BLE failed!");
    while (1);
  }

  setupBLEService();
  setupIMUService();

  BLE.advertise();  // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
}

//---------------------------------------------

// Function definition of IMU reading (Accelerometer)

void readAccelerometerData(){
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    dx = (abs(xPrev - x) < sensitivity);
    dy = (abs(yPrev - y) < sensitivity);
    dz = (abs(zPrev - z) < sensitivity);
  }else{
    Serial.print("Accelerometer data unavailable");
  }
}

bool update_timer(){
  diffMillis = currentMillis - startMillis;
  if(diffMillis >= interval){
    return true;
  }else{
    return false;
  }
}

//---------------------------------------------

// Function returning structure - Active Mode functionality, everything active on Arduino
// Logic for arduino moving is also called in the function
struct ArduinoStatus ActiveMode() {

  // Return: ArduinoMode, inMotion
  //           Arduino Mode - To know the exact mode in which arduino is
  //           Arduino Mode 1 - means arduino is in active mode
  //           inMotion - To know if arduino moving or not based on IMU accelerometer readings 
  //           (if it is changing continuously it is in moving state (1))

  struct ArduinoStatus aStatus;
  
  digitalWrite(LED_PWR, HIGH); // turn power LED
//  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); // turn I²C pullup
  
  pinMode(PIN_A6, OUTPUT);
  digitalWrite(PIN_A6, HIGH);


  readAccelerometerData();

  if (dx || dy || dz){
    // Start timer when the arduino stops moving
    currentMillis = millis();
//    Serial.println(currentMillis);    
    inMotion = false;
    xPrev = x;
    yPrev = y;
    zPrev = z;
//    ArduinoMode = 0;
  } else {
    inMotion = true;
    xPrev = x;
    yPrev = y;
    zPrev = z;
//    ArduinoMode = 1;
  }
  
  Serial.print("Active Mode");
  Serial.print('\t');
  Serial.print(x);
  Serial.print('\t');
  Serial.print(y);
  Serial.print('\t');
  Serial.print(z);
  Serial.print('\t');
  Serial.print(inMotion);
  Serial.print('\t');
  Serial.println(ArduinoMode);
  delay(1000);                          // wait to send limited data                       

  ArduinoMode = 1;
  aStatus.ArduinoMode = ArduinoMode;
  aStatus.inMotion = inMotion;
 
 return aStatus;
}

// Update - Dhruv (24-03-2021)
// Setting up service for the Sleep Mode
struct ArduinoStatus SleepMode(){

  // Return: ArduinoMode, inMotion
  //           Arduino Mode - To know the exact mode in which arduino is
  //           Arduino Mode 0 - means arduino is in sleep mode
  //           inMotion - To know if arduino moving or not based on IMU accelerometer readings 
  //           (if it is not changing continuously it is in still state (0))
  
    struct ArduinoStatus aStatus;
    
    digitalWrite(LED_PWR, LOW); // turn off power LED
//    digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW); // turn off I²C pullup

    pinMode(PIN_A6, OUTPUT);
    digitalWrite(PIN_A6, LOW);
    
    readAccelerometerData();
  
    if (dx || dy || dz){
      inMotion = false;
      xPrev = x;
      yPrev = y;
      zPrev = z;
//      ArduinoMode = 0;
    } else {
      inMotion = true;
      xPrev = x;
      yPrev = y;
      zPrev = z;
//      ArduinoMode = 1;
      startMillis = millis();
      Serial.println(startMillis);
    }
     
    Serial.print("Sleep Mode");
    Serial.print('\t');
    Serial.print(inMotion);
    Serial.print('\t');
    Serial.println(ArduinoMode);
    delay(1000);                          // wait to send limited data 

    ArduinoMode = 0;
    aStatus.ArduinoMode = ArduinoMode;
    aStatus.inMotion = inMotion;
 
 return aStatus;
}

// ________________Tests_________________
// Test - 1 (Test for the timer)

void millis_test(){
  unsigned long start_time = millis();
  Serial.print("Start time for the timer test: ");
  Serial.print('\t');
  Serial.println(start_time);

  while(millis() - start_time < 10000){
    Serial.println(millis());
  }

  unsigned long end_time = millis();
  Serial.print("End of test - time duration: ");
  Serial.print('\t');
  Serial.println(end_time - start_time);

  if ((end_time - start_time) >= (10000 - 50) || (end_time - start_time) <= (10000 + 50)){
    Serial.println("Test Passed");
  }else{
    Serial.println("Test Failed");
  }
}


void run_test(){
  millis_test();
}
void loop() 
{
//  BLEDevice central = BLE.central();  // Wait for a BLE central to connect
//
//  // if a central is connected to the peripheral:
//  if (central) 
//  {
//    Serial.print("Connected to central MAC: ");
//    Serial.println(central.address());
//    connectedLight();
//    
//    while (central.connected())
//    {
//      helloCharacteristic.setValue(hello); // Set greeting string
//
//      if( rssiValue != BLE.rssi() )
//      {
//        rssiValue = BLE.rssi();
//        RSSICharacteristic.setValue( rssiValue );  
//      }
//
//      {//Battery level calculation
//        int battery = analogRead(A0);
//        unsigned tempBatteryLevel = map(battery, 0, 1023, 0, 100);
//        if( tempBatteryLevel != batteryLevel )
//        {
//          batteryLevel = tempBatteryLevel;
//          batteryLevelCharacteristic.writeValue(batteryLevel);
//          //Serial.print("Battery Level % is now: ");
//          //Serial.println(batteryLevel);
//        }
//      }
//    }
//
//    disconnectedLight();
//    Serial.print("Disconnected from central MAC: ");
//    Serial.println(central.address());
//  }


  //---------------------------------------------
  // Update - Dhruv (24/03/2021)

// Now if arduino is not moving for more than 20sec (lets say) and is in active mode, then put the arduino to sleep mode
   if  (status.inMotion == false){       
      if (update_timer()){
        status = SleepMode();
        }else {
        status = ActiveMode();
      }
   }else{
    status = ActiveMode(); 
   }

//   run_test();
}
