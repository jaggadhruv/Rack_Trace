#include <ArduinoBLE.h>                 // Include the Arduino BLE Library       
#include <Arduino_LSM9DS1.h>            // Using IMU Sensor <Accelerometer, Gyroscope and Magneto meter>                   

BLEService arduinoService("1101");  // User defined service

BLECharacteristic macCharacteristic("00002A3D-0000-1000-8000-00805f9b34fb", BLERead | BLENotify | BLEBroadcast ,256, false);
BLEByteCharacteristic RSSICharacteristic( "0000A666-0000-1000-8000-00805f9b34fb", BLERead | BLENotify );
BLEUnsignedCharCharacteristic batteryLevelCharacteristic("00002101-0000-1000-8000-00805f9b34fb", BLERead | BLENotify);
BLEBoolCharacteristic isMovingCharacteristic( "0000C666-0000-1000-8000-00805f9b34fb", BLERead | BLENotify );

#define NUMBER_OF_REFERENCE_POINTS 3
BLECharacteristic rssiDistancesCharacteristic( "0000B666-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, NUMBER_OF_REFERENCE_POINTS * sizeof( float ) );

unsigned batteryLevel = 0;
int rssiValue = 0;

String referencePointsNames[NUMBER_OF_REFERENCE_POINTS] = { "", "", "" };
float referencePointsDistances[NUMBER_OF_REFERENCE_POINTS] = { 0, 0, 0 };

// Sleep Mode variables
const long interval = 1000 * 20;         // interval at which to sleep (milliseconds)

// Variable definition for the logic to check if the arduino is moving or not
float x, y, z;
static float xPrev = 0.0;
static float yPrev = 0.0;
static float zPrev = 0.0;
bool dx = false;
bool dy = false;
bool dz = false;

bool inMotion = false;
float sensitivity = 0.008;
int ArduinoMode = 0;

// Variable declarations for timer
unsigned long diffMillis = 0;
unsigned long currentMillis = 0;
unsigned long startMillis = 0;

// Variable declaration for the Blink Timer in Sleep Mode
unsigned long startBlinkMillis = 0;
unsigned long currentBlinkMillis = 0;
const unsigned long blinkPeriod = 1000 * 20;

unsigned long diffMillis2 = 0;
unsigned long currentMillis2 = 0;
unsigned long currentMillis3 = 0;

//-----------------------------------------------

/**
* @brief Structure to store two different types as return from the SleepMode and ActiveMode functions
* @author Dhruv Jagga
*/
struct ArduinoStatus{
  volatile int ArduinoMode;
  volatile int inMotion;  
};

struct ArduinoStatus status;

//-----------------------------------------------

/**
* @brief This method set the LED to green
* @author Ákos Gréczy
*/
void connectedLight() 
{
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
}

//-----------------------------------------------

/**
* @brief This method set the LED to red
* @author Ákos Gréczy
*/
void disconnectedLight() 
{
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
}

//-----------------------------------------------

/**
* @brief This method initialize the serial communication
* @author Ákos Gréczy
*/
void initSerialCommunication()
{
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);
}

//-----------------------------------------------

/**
* @brief This method setup the arduino service
* @author Ákos Gréczy
*/
void setupBLEService()
{
  BLE.setLocalName("Nano33BLE");  // Set name for connection
  BLE.setAdvertisedService(arduinoService); // Advertise service
  
  arduinoService.addCharacteristic(macCharacteristic);
  arduinoService.addCharacteristic(RSSICharacteristic);
  arduinoService.addCharacteristic(batteryLevelCharacteristic);
  arduinoService.addCharacteristic(rssiDistancesCharacteristic);
  arduinoService.addCharacteristic(isMovingCharacteristic);
  
  BLE.addService(arduinoService); // Add service
}

//-----------------------------------------------

/**
* @brief This method setup the IMU service
* @author Dhruv Jagga
*/
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

/**
* @brief This method calculates the RSSI distance from the reference points. 
* The function search for "ReferencePoint" string. 
* If the nearby devices has this name, the function calculate the distance and print it to the serial console
* @author Ákos Gréczy
*/
void calculateRSSIBasedOnFixPoints()
{
  BLEDevice peripheral = BLE.available();
  
  if (peripheral) 
  {
    if (peripheral.hasLocalName())
    {
      //if( peripheral.localName() == "ReferencePoint" )
      if( peripheral.localName() == "JBL Charge 4" )
      {
        Serial.println();
        Serial.print( "Local name: " );
        Serial.print( peripheral.localName() );
        Serial.print( "\t rssi:" );
        Serial.println( abs( peripheral.rssi() - BLE.rssi() ) );
      }
    }
  }
}

//-----------------------------------------------

/**
* @brief This method calculates the battery level of the arduino
* @author Ákos Gréczy
*/
void batteryLevelCalculation()
{
  int battery = analogRead(A0);
  unsigned tempBatteryLevel = map(battery, 0, 1023, 0, 100);
  if( tempBatteryLevel != batteryLevel )
  {
    batteryLevel = tempBatteryLevel;
    batteryLevelCharacteristic.writeValue(batteryLevel);
  }
}

//-----------------------------------------------

void setup() 
{
  initSerialCommunication();  

  disconnectedLight();//pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin

  if (!BLE.begin()) {   // initialize BLE
    Serial.println("starting BLE failed!");
    while (1);
  }
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  setupBLEService();

  BLE.advertise();  // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
}


//---------------------------------------------

/**
* @brief Function definition of IMU reading (Accelerometer)
* @author Dhruv Jagga
*/
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

//---------------------------------------------

/**
* @brief Timer to switch to other state
* @author Dhruv Jagga
*/
bool update_timer(){
  diffMillis = currentMillis - startMillis;
  if(diffMillis >= interval){
    return true;
  }else{
    return false;
  }
}

bool update_timer2(){
  diffMillis2 = currentBlinkMillis - startBlinkMillis;
  if(diffMillis2 >= blinkPeriod){
    return true;
  }else{
    return false;
  }
}

//---------------------------------------------

/**
* @brief Function returning structure - Active Mode functionality, everything active on Arduino
*        Logic for arduino moving is also called in the function
* @author Dhruv Jagga
*/
struct ArduinoStatus ActiveMode() {

  /** Return: ArduinoMode, inMotion
  *           Arduino Mode - To know the exact mode in which arduino is
  *           Arduino Mode 1 - means arduino is in active mode
  *           inMotion - To know if arduino moving or not based on IMU accelerometer readings 
  *           (if it is changing continuously it is in moving state (1))
  */
  
  struct ArduinoStatus aStatus;
  
  digitalWrite(LED_PWR, HIGH); // turn power LED
//  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); // turn I²C pullup
  
  pinMode(PIN_A6, OUTPUT);
  digitalWrite(PIN_A6, HIGH);


  readAccelerometerData();

  if (dx || dy || dz){
    // Start timer when the arduino stops moving
    currentMillis = millis();
    inMotion = false;
    xPrev = x;
    yPrev = y;
    zPrev = z;
  } else {
    inMotion = true;
    xPrev = x;
    yPrev = y;
    zPrev = z;
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

  //This part just simply advertise the RSSI of arduino
  if( rssiValue != BLE.rssi() )
  {
    rssiValue = BLE.rssi();
    RSSICharacteristic.setValue( rssiValue );  
  }

  batteryLevelCalculation();

  ArduinoMode = 1;
  aStatus.ArduinoMode = ArduinoMode;
  aStatus.inMotion = inMotion;
 
 return aStatus;
}

//---------------------------------------------

/**
* @brief Setting up service for the Sleep Mode
* @author Dhruv Jagga
*/

struct ArduinoStatus SleepMode(){

  /** Return: ArduinoMode, inMotion
  *           Arduino Mode - To know the exact mode in which arduino is
  *           Arduino Mode 0 - means arduino is in sleep mode
  *           inMotion - To know if arduino moving or not based on IMU accelerometer readings 
  *           (if it is not changing continuously it is in still state (0))
  */
    
  
    currentBlinkMillis = millis();    
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
    } else {
      inMotion = true;
      xPrev = x;
      yPrev = y;
      zPrev = z;
      startMillis = millis();
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

//---------------------------------------------

/**
* @brief Setting up service for the Blink Mode
* @author Dhruv Jagga
*/

struct ArduinoStatus BlinkMode(){
  startBlinkMillis = millis();
  
  struct ArduinoStatus aStatusSleep;
  
  aStatusSleep = SleepMode();
  ArduinoMode = 2;
  inMotion = aStatusSleep.inMotion;

  struct ArduinoStatus aStatus;

  aStatus.ArduinoMode = ArduinoMode;
  aStatus.inMotion = inMotion;
    
  Serial.println("Blink and Publish Data");

  //This part just simply advertise the RSSI of arduino
  if( rssiValue != BLE.rssi() )
  {
    rssiValue = BLE.rssi();
    RSSICharacteristic.setValue( rssiValue );  
  }

  batteryLevelCalculation();
  
  return aStatus;
}

//---------------------------------------------

// ________________Tests_________________

/**
* @brief Test - 1 (Test for the timer)
* @author Dhruv Jagga
*/

void millis_test(){
  unsigned long start_time = millis();
//  Serial.print("Start time for the timer test: ");
//  Serial.print('\t');
//  Serial.println(start_time);

  while(millis() - start_time < 10000){
    Serial.println(millis());
  }

  unsigned long end_time = millis();
//  Serial.print("End of test - time duration: ");
//  Serial.print('\t');
//  Serial.println(end_time - start_time);

  if ((end_time - start_time) >= (10000 - 50) || (end_time - start_time) <= (10000 + 50)){
    Serial.println("Test-1 Passed");
  }else{
    Serial.println("Test-1 Failed");
  }
}

//---------------------------------------------

/**
* @brief Test - 2 (Test for the Arduino Mode)
* @author Dhruv Jagga
*/

void arduinoStatus_test(){
  struct ArduinoStatus aStatusActive;
  struct ArduinoStatus aStatusSleep;
  struct ArduinoStatus aStatusBlink;

  aStatusActive = ActiveMode();
  aStatusSleep = SleepMode();
  aStatusBlink = BlinkMode();

  if (aStatusActive.ArduinoMode == 1 && aStatusSleep.ArduinoMode == 0 && aStatusBlink.ArduinoMode == 2){
    Serial.println("Test-2 Passed");
  }else{
    Serial.println("Test-2 Failed");
  }
}

//---------------------------------------------

/**
* @brief Test - 3 (Test for the Arduino Motion Detection)
* @author Dhruv Jagga
*/

void arduinoMotion_test(){
  struct ArduinoStatus aStatusActive;
  struct ArduinoStatus aStatusSleep;
  struct ArduinoStatus aStatusBlink;

  aStatusActive = ActiveMode();
  aStatusSleep = SleepMode();
  aStatusBlink = BlinkMode();

  if (aStatusActive.inMotion == 1 || aStatusActive.inMotion == 0 &&
      aStatusSleep.inMotion == 1 || aStatusSleep.inMotion == 0 &&
      aStatusBlink.inMotion == 1 || aStatusBlink.inMotion == 0){
    Serial.println("Test-3 Passed");
  }else{
    Serial.println("Test-3 Failed");
  }
}

//---------------------------------------------

/**
* @brief Routine to run the tests
* @author Dhruv Jagga
*/

void run_test(){
  millis_test();
  arduinoStatus_test();
  arduinoMotion_test();  
}

//---------------------------------------------

void loop() 
{
  BLEDevice central = BLE.central();  // Wait for a BLE central to connect
  
  // if a central is connected to the peripheral:
  if (central) 
  {
    Serial.print("Connected to central MAC: ");
    Serial.println(central.address());
    Serial.println(central.deviceName());
    //macCharacteristic.writeValue(BLE.address());
    connectedLight();
    
    while (central.connected())
    {
      // Update - Dhruv (20/05/2021)
      //calculateRSSIBasedOnFixPoints();

      // Now if arduino is not moving for more than 20sec (lets say) and is in active mode, then put the arduino to sleep mode
      if  (status.inMotion == false){       
        if (update_timer()){
          status = SleepMode();

            if (update_timer2()){
              status = BlinkMode();
            }else{
              status = SleepMode();
            }          
          }else {
          status = ActiveMode();
        }
    }else{
      status = ActiveMode(); 
    }
    }

    disconnectedLight();
    Serial.print("Disconnected from central MAC: ");
    Serial.println(central.address());
  }



  //---------------------------------------------
  // Update - Dhruv (27/05/2021)
    

    
//   run_test();
}
