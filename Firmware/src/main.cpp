#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCCAN.h>
#include <StealthController.h>

// Configuration for the magnetic sensor - SPI
MagneticSensorSPIConfig_s MA702_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 20000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15, 
  .command_rw_bit = 0,
  .command_parity_bit = 0
};

// Magnetic sensor instance
MagneticSensorSPI sensor = MagneticSensorSPI(MA702_SPI, ENC_SS);

// BLDC motor instance with 7 pole pairs
BLDCMotor motor = BLDCMotor(7);

// DRV8316 motor driver instance
DRV8316Driver6PWM driver = DRV8316Driver6PWM(DRV_A_H, DRV_A_L, DRV_B_H, DRV_B_L, DRV_C_H, DRV_C_L, DRV_SS, false);

// Stealth controller instance
StealthController stealth = StealthController(driver);

// CAN driver and commander instances
CANDriver can = CANDriver(CAN_TX, CAN_RX);
CANCommander canCommand = CANCommander(can);

// Commander instance for receiving commands via Serial
Commander command = Commander(Serial);

// Function to process received motor commands
void doCommander(char* cmd) { command.motor(&motor, cmd); }

// Function to process received motor commands via CAN
void doCommanderCAN(char* cmd) { canCommand.motor(&motor, cmd); }

// Function to process custom commands
void doCustom(char* cmd) { 
  printf("Custom Command: %s\n", cmd);
  switch(cmd[0]) {
    case 'S':
      driver.setSlew((DRV8316_Slew)atoi(&cmd[1])); 
      break;
    case 'A':
      driver.setActiveSynchronousRectificationEnabled(atoi(&cmd[1]) != 0);
      break;
    case 'B':
      driver.setActiveAsynchronousRectificationEnabled(atoi(&cmd[1]) != 0);
      break;  
    case 'C':
      setCpuFrequencyMhz(atoi(&cmd[1]));
      break;
  }
}

void setup() {  
  // Initialize serial communication
  Serial.begin(115200);
  delay(100);
  Serial.println("Initializing...");

  // Add commands to the Commander
  command.add('M', doCommander, (char*)"motor");
  command.add('Z', doCustom, (char*)"Custom");
  canCommand.add('M', doCommanderCAN, (char*)"motor");
  
  // Enable monitoring via Serial
  motor.useMonitoring(Serial);

  // Setup Stealth Controller
  stealth.setup();

  // Configure and initialize motor driver
  driver.pwm_frequency = 50000;
  driver.voltage_power_supply = 12;
  driver.init(stealth.enc_spi);
  motor.linkDriver(&driver);

  // Initialize magnetic sensor and link it to the motor
  sensor.init(stealth.drv_spi);
  motor.linkSensor(&sensor);

  Serial.println("Driver Init complete...");

  // Configure motor control and modulation
  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = 4.0;
  motor.voltage_sensor_align = 4.0;  
  motor.velocity_limit = 100;

  // Configure motor control parameters
   motor.LPF_velocity.Tf = 0.03;
  motor.PID_velocity.output_ramp = 500;

  // Configure velocity PI controller parameters
  motor.PID_velocity.P = 0.0;
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.0;

  delay(100);
  stealth.printStatus();

  // Disable motor monitoring at first
  motor.monitor_downsample = 0;
  motor.target = motor.shaftAngle();
  motor.init();
  motor.initFOC();
  motor.target = motor.shaftAngle();

  Serial.println("Motor Init complete...");
}

void loop() {
  // Run the motor control loop
  motor.loopFOC();
  motor.move();
  motor.monitor();

  // Run the CAN and Serial Commanders
  canCommand.runWithCAN();
  command.run();

  // Run the Stealth Controller loop step
  stealth.loopStep(motor, 0.09, 2.0, 0.0, true);
}
