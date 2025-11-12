#include <Arduino.h>

// FDCAN Library for STM32G4
#include <ACANFD_STM32.h>                                 

#include <SPI.h>

// SimpleFOC and SimpleFOC Drivers Library
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

// Include AS5048A encoder support from SimpleFOCDrivers
#include "encoders/as5048a/MagneticSensorAS5048A.h"
// Include DRV8316 support from SimpleFOCDrivers
#include "drivers/drv8316/drv8316.h"
// Include CORDIC support from SimpleFOCDrivers (the STM32
// has special hardware acceleration available to work out
// trig functions faster)
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#define LED_BUILTIN PC6

#define SPI1_SCK   PA5
#define SPI1_MISO  PA6   
#define SPI1_MOSI  PA7
#define DRV_CS     PA4 
#define ENC_CS     PC4

#define CAN_EN  PC10

#define PHA_H   PA10
#define PHA_L   PB15

#define PHB_H   PA9
#define PHB_L   PB14

#define PHC_H   PA8
#define PHC_L   PB13

#define SO_A    PA0
#define SO_B    PA1
#define SO_C    PA2

// SPI Object for communication with DRV8316
SPIClass SPI_1(SPI1_MOSI, SPI1_MISO, SPI1_SCK);

// Motor object for motor with 11 pole pairs
BLDCMotor motor = BLDCMotor(11);

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair 
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

// Low side current sensing with 600mV/A gain (~ max 5A measurement for 3.3V tolerant pins)
LowsideCurrentSense current_sense  = LowsideCurrentSense(600.0,  SO_A, SO_B, SO_C);

// Run AS5048A at 5 MHz SPI in mode 1 with ENC_CS pin
MagneticSensorAS5048A sensor(ENC_CS, false, SPISettings(5'000'000, MSBFIRST, SPI_MODE1));

float target_current = 0;

float voltage_limit = 10;  // Driver voltage limit

// Function prints the current status of the DRV8316
void printDRV8316Status() {
  DRV8316Status status = driver.getStatus();
	Serial.println("DRV8316 Status:");
	Serial.print("Fault: ");
	Serial.println(status.isFault());
	Serial.print("Buck Error: ");
	Serial.print(status.isBuckError());
	Serial.print("  Undervoltage: ");
	Serial.print(status.isBuckUnderVoltage());
	Serial.print("  OverCurrent: ");
	Serial.println(status.isBuckOverCurrent());
	Serial.print("Charge Pump UnderVoltage: ");
	Serial.println(status.isChargePumpUnderVoltage());
	Serial.print("OTP Error: ");
	Serial.println(status.isOneTimeProgrammingError());
	Serial.print("OverCurrent: ");
	Serial.print(status.isOverCurrent());
	Serial.print("  Ah: ");
	Serial.print(status.isOverCurrent_Ah());
	Serial.print("  Al: ");
	Serial.print(status.isOverCurrent_Al());
	Serial.print("  Bh: ");
	Serial.print(status.isOverCurrent_Bh());
	Serial.print("  Bl: ");
	Serial.print(status.isOverCurrent_Bl());
	Serial.print("  Ch: ");
	Serial.print(status.isOverCurrent_Ch());
	Serial.print("  Cl: ");
	Serial.println(status.isOverCurrent_Cl());
	Serial.print("OverTemperature: ");
	Serial.print(status.isOverTemperature());
	Serial.print("  Shutdown: ");
	Serial.print(status.isOverTemperatureShutdown());
	Serial.print("  Warning: ");
	Serial.println(status.isOverTemperatureWarning());
	Serial.print("OverVoltage: ");
	Serial.println(status.isOverVoltage());
	Serial.print("PowerOnReset: ");
	Serial.println(status.isPowerOnReset());
	Serial.print("SPI Error: ");
	Serial.print(status.isSPIError());
	Serial.print("  Address: ");
	Serial.print(status.isSPIAddressError());
	Serial.print("  Clock: ");
	Serial.print(status.isSPIClockFramingError());
	Serial.print("  Parity: ");
	Serial.println(status.isSPIParityError());
	if (status.isFault())
		driver.clearFault();
	delayMicroseconds(1); // ensure 400ns delay
	DRV8316_PWMMode val = driver.getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(1); // ensure 400ns delay
	bool lock = driver.isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);
}

// Setup various things for FOC
void setupFOC() {
    // Initialise magnetic sensor hardware
    sensor.init(&SPI_1);
    // Link the motor to the sensor
    motor.linkSensor(&sensor);

    // Set voltages and limit 
    driver.voltage_power_supply = 12.0;
    driver.voltage_limit = voltage_limit;

    // Connect to the DRV8316 over SPI_1
    driver.init(&SPI_1);

    // Small delay to allow SPI response
    delay(10);

    // Set the buck output to 5V
    driver.setBuckVoltage(VB_5V);
    // Wait some time for everything to stabilise
    delay(500);

    // It's advised to have at least 400ns between read and write calls
    driver.setOCPLevel(Curr_16A);
    delayMicroseconds(1);
    driver.setOCPMode(AutoRetry_Fault); 
    delayMicroseconds(1);
    driver.setOCPRetryTime(Retry500ms);
    delayMicroseconds(1);
    driver.setSlew(Slew_200Vus);
    delayMicroseconds(1);
    driver.setPWM100Frequency(FREQ_20KHz);
    delayMicroseconds(1);

    // Set the current sense gain. NOTE THAT THE DRV8316C DATASHEET
    // DIFFERS FROM THE VALUES SUGGGESTED BY SIMPLEFOC

    //driver.setCurrentSenseGain(Gain_0V15); // Set the current sense gain to 150 mV/A
    driver.setCurrentSenseGain(Gain_0V25); // Set the current sense gain to 600 mV/A
    //driver.setCurrentSenseGain(Gain_0V375); // Set the current sense gain to 1200 mV/A
    delayMicroseconds(1);

    // Set into 6 wire PWM mode
    driver.setPWMMode(PWM6_Mode);
    delayMicroseconds(1);

    Serial.println("Init complete...");

    delay(100);
    printDRV8316Status();

    // Set motor current to 2A 
    motor.current_limit = 2.0;   // [Amps]

    // Link current sensor with the driver
    current_sense.linkDriver(&driver);

    // Link driver to the motor
    motor.linkDriver(&driver);

    // Aligning voltage 
    motor.voltage_sensor_align = 3;
    // Choose FOC modulation (I believe low-side current sense only
    // supports SinePWM or SpaceVectorPWM)
    motor.foc_modulation = FOCModulationType::SinePWM;

    // Use the most accurate current control mode (using current sense)
    motor.torque_controller = TorqueControlType::foc_current;
    //motor.torque_controller = TorqueControlType::voltage;
    
    // Set motion control loop to be used
    motor.controller = MotionControlType::torque;

    
    // Enable debuggin to the serial monitor
    motor.useMonitoring(Serial);
}

// Start FOC
void startFOC() {
    // Initialize motor
    motor.init();

    if (current_sense.init())  Serial.println("Current sense init success!");
    else{
        Serial.println("Current sense init failed!");
        return;
    }
      // Link the current sense to the motor
    motor.linkCurrentSense(&current_sense);


     // Align encoder and start FOC
    motor.initFOC();

    Serial.println(F("Motor ready."));

    // _delay is a special non-blocking delay function implemented by SimpleFOC
    _delay(1000);
}


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // CAN Transceiver must be enabled by pulling CAN_EN (pin PC10) high.
    pinMode(CAN_EN, OUTPUT);
    digitalWrite(CAN_EN, HIGH);

    // Set SPI CS pins as outputs
    pinMode(DRV_CS, OUTPUT);
    pinMode(ENC_CS, OUTPUT);

    // Set current phase signals as inputs
    pinMode(SO_A, INPUT);
    pinMode(SO_B, INPUT);
    pinMode(SO_C, INPUT);

    // Enable SimpleFOC serial debugging
    SimpleFOCDebug::enable(&Serial);

    // Enable CORDIC
    SimpleFOC_CORDIC_Config();

    Serial.begin(115200);

    delay(2000);
  
  // Start FDCAN with an atrbitration rate of 1Mbps (80% sample point) and data rate of 5Mbps (80% sample point)
  // CAN 2.0B will only support up to atrbitration rate of 1Mbps and data rate of 1Mbps.
  ACANFD_STM32_Settings settings (1000 * 1000, 80, DataBitRateFactor::x5, 80) ;

  //ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x5) ;
  //ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x2) ;
  //ACANFD_STM32_Settings settings (1000 * 1000, DataBitRateFactor::x3) ;
  //ACANFD_STM32_Settings settings (1000 * 1000, 87, DataBitRateFactor::x5, 75) ;

  // CAN debugging
  Serial.print ("Bit Rate prescaler: ") ;
  Serial.println (settings.mBitRatePrescaler) ;
  Serial.print ("Arbitration Phase segment 1: ") ;
  Serial.println (settings.mArbitrationPhaseSegment1) ;
  Serial.print ("Arbitration Phase segment 2: ") ;
  Serial.println (settings.mArbitrationPhaseSegment2) ;
  Serial.print ("Arbitration SJW: ") ;
  Serial.println (settings.mArbitrationSJW) ;
  Serial.print ("Actual Arbitration Bit Rate: ") ;
  Serial.print (settings.actualArbitrationBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Arbitration sample point: ") ;
  Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
  Serial.println ("%") ;
  Serial.print ("Exact Arbitration Bit Rate ? ") ;
  Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
  Serial.print ("Data Phase segment 1: ") ;
  Serial.println (settings.mDataPhaseSegment1) ;
  Serial.print ("Data Phase segment 2: ") ;
  Serial.println (settings.mDataPhaseSegment2) ;
  Serial.print ("Data SJW: ") ;
  Serial.println (settings.mDataSJW) ;
  Serial.print ("Actual Data Bit Rate: ") ;
  Serial.print (settings.actualDataBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Data sample point: ") ;
  Serial.print (settings.dataSamplePointFromBitStart ()) ;
  Serial.println ("%") ;
  Serial.print ("Exact Data Bit Rate ? ") ;
  Serial.println (settings.exactDataBitRate () ? "yes" : "no") ;

  // The CAN has different modes where it can receive its own messages (LOOPBACK)
  // Internal loopback doesn't send the message to the transceiver, external does.

  //settings.mModuleMode = ACANFD_STM32_Settings::INTERNAL_LOOP_BACK ;
  //settings.mModuleMode = ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK ;
  settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD ;

  // Set CAN to use the correct pins
  settings.mTxPin = PB_9 ; 
  settings.mRxPin = PB_8 ; 

  // Start CAN. Blink LED if error.
  uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan1 ok") ;
  }else{
    Serial.print ("Error can: 0x") ;
    Serial.println (errorCode, HEX) ;
    while (1) {
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
      delay (50) ;
    }
  }
  // Serial.print ("Systick ") ;
  // Serial.println (SysTick->LOAD + 1) ;

  // Setup FOC
  setupFOC();

  // Start FOC (runs alignment procedure)
  startFOC();
}


static uint32_t gSendDate = 0 ; // Variable used to send message very 1 second
static uint32_t gSentCount = 0 ; // Variable to count number of messages sent
static uint32_t gReceivedCount = 0 ; // Variable to count number of messages received

static bool ledState = false; // Boolean to turn LED on and off

void loop() {

    
  motor.loopFOC(); // Must run as frequently as possible for maximum FOC performance

  motor.move(target_current); // Set the current target

  // Sent a message every 1 second
  if (gSendDate < millis ()) {

    // Create a new message object
    CANFDMessage message;
    message.id = 0x7FF; // CAN ID
    message.len = 8; // How many bytes your message is (CAN 2.0B is max 8)
    message.data [0] = 0; // Byte 0 data
    message.data [1] = 1; // Byte 1 data
    message.data [2] = 2; // Byte 2 data
    message.data [3] = 3; // Byte 3 data
    message.data [4] = 4; // Byte 4 data
    message.data [5] = 5; // Byte 5 data
    message.data [6] = 6; // Byte 6 data
    message.data [7] = 7; // Byte 7 data

    const uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {

      gSendDate += 1000 ;
      gSentCount += 1 ;

      Serial.print ("fdcan1 sent: ") ;
      Serial.println (gSentCount) ;
    }
  }

  // Create a new message object
  CANFDMessage messageFD ;
  // If there is a message waiting in the buffer, read it
  if (fdcan1.receiveFD0 (messageFD)) {
    gReceivedCount += 1 ;

    // Blink the LED
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    
    // Very basic filtering, the library has more advanced options
    if (messageFD.id == 0x001) {

        Serial.printf("Data ID: %x, Length: %u", messageFD.id, messageFD.len);

        // Data structure to store the data you are expecting
        uint8_t received_data[4];

        // Copy the message data into the data structure
        memcpy(received_data, messageFD.data, 4);

        // Need to implement code here to decode the data and set the target_current.
    }

    Serial.print ("fdcan1 received: ") ;
    Serial.println (gReceivedCount) ;
  }
}
