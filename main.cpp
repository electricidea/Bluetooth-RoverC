
/******************************************************************************
 * Bluetooth RoverC command control
 * 
 * This code starts a Bluetooth serial server on the M5StickC and waits for
 * angular values a command to programm the RoverC movement.
 * A single value of 0 will let the Rover move forward for 500ms
 * A set of the following values: 180 270 0 90 will let the Rover move
 * backwards, then to the left, forward and finally to the right.
 * In fact: The Rover will move a square path.
 * 
 * One can use a Mobile phone or a PC with a Bluetooth Serial APP to
 * send the commands to the M5StickC. A '*' will finish the commans, or
 * a timout of 5 seconds.
 * 
 * Hague Nusseck @ electricidea
 * v1.4 28.Apr.2020
 * https://github.com/electricidea/Bluetooth-RoverC
 * 
 * 
 * 
 * Changelog:
 * v1.4 = initial version
 * 
 * Distributed as-is; no warranty is given.
 ******************************************************************************/
#include <Arduino.h>

#include <M5StickC.h>
// M5StickC Library:
// Install for PlatformIO:
// pio lib install "M5StickC"

// I2C address of the RoverC Hat
#define ROVERC_I2C_Address 0x38

#include <TimeLib.h>
// pio lib install "Time"
// lib_deps = Time

#include <BluetoothSerial.h>
// the Bluetooth serial object
BluetoothSerial SerialBT;
uint64_t chipid;
char chipname[256];
// the string to build the received command
String read_String = "";
// timout counter for Receiving Bluetooth data 
uint32_t last_BT_cmd;

// a simple queue to store the commands for the RoverC
#include "command_queue.h"

// Library for a simple text buffer scrolling display on the M5StickC.
#include "tb_display.h"

// Display brightness level
// possible values: 7 - 15
uint8_t screen_brightness = 10; 

// screen Rotation values:
// 1 = Button right
// 2 = Button above
// 3 = Button left
// 4 = Button below
int screen_orientation = 3;

// state machine definitions
#define ps_stop 0
#define ps_wait_for_BT 10
#define ps_recv_BT_codes 20
#define ps_run_program 30
int process_state = ps_stop;

// Variable to trigger the state macine every second
uint8_t last_second;

// container for the actual motor speed
int8_t motor_speeds[4] = {0,0,0,0};
// motor numbers:
// 1 ------ 2
//   |    |
//   |    |
//   | M5 |
// 3 ------ 4


//==============================================================
// speed_ramp is limiting the acceleration to a maximum of
// 5 steps per call.
// higher accelerations make the rover behaves "jumpy"
int8_t speed_ramp(int8_t actual_speed, int8_t target_speed){
  // acceleration not larger than 5 can be applied directly
  // so init calc_speed with target_speed and check if the 
  // acceleration is higher than 5
  int8_t calc_speed = target_speed;
  // acceleration larger than 5?
  if(abs(target_speed - actual_speed) > 5){
    if( target_speed > actual_speed) 
      calc_speed = actual_speed + 5;
    else
      calc_speed = actual_speed - 5;
  }
  // ensure that the result is between -100 and +100
  calc_speed = (calc_speed > 100) ? 100 : calc_speed;
  calc_speed = (calc_speed < -100) ? -100 : calc_speed;
  return calc_speed;
}


//==============================================================
// setting the motor values to new speed under consideration 
// of the maximum acceleration.
void rover_set_motors(int8_t M1, int8_t M2, int8_t M3, int8_t M4) {
  motor_speeds[0] = speed_ramp(motor_speeds[0],M1); 
  motor_speeds[1] = speed_ramp(motor_speeds[1],M2);
  motor_speeds[2] = speed_ramp(motor_speeds[2],M3);
  motor_speeds[3] = speed_ramp(motor_speeds[3],M4);
  //Serial.print(motor_speeds[0]);Serial.print(" ");Serial.print(motor_speeds[1]);Serial.print(" ");
  //Serial.print(motor_speeds[2]);Serial.print(" ");Serial.println(motor_speeds[3]);
  // send the Motor speed to the Rover board via I2C
  Wire.beginTransmission(ROVERC_I2C_Address);
  short address = 0x00;
  Wire.write(address);
  Wire.write(motor_speeds[0]);
  Wire.write(motor_speeds[1]);
  Wire.write(motor_speeds[2]);
  Wire.write(motor_speeds[3]);
  Wire.endTransmission();
}


//==============================================================
// calculate the speed for the four motors to let the rover
// move into a specific direction (angle) at a given speed
// angle is: 
//    0deg = forward
//   90deg = right
//  180deg = backwards
//   45deg = diagonal right forward
// speed is:
//    +100 = maximum speed forward
//    -100 = maximum speed backwards
// NOTE:
// this function returns after the desired speed is reached
// Due to the acceleration and deceleration, the process may 
// take different amounts of time for different calls. 
void move_rover(double angle, int speed) {
  // ensure that speed is between -100 and 100
  speed = (speed > 100) ? 100 : speed;
  speed = (speed < -100) ? -100 : speed;
  double vx = sin((angle * PI) / 180.0) * speed;
  double vy = cos((angle * PI) / 180.0) * speed;
  // calculate the target speed for each motor
  int8_t M1 = (int8_t) round(vy + vx);
  int8_t M2 = (int8_t) round(vy - vx);
  int8_t M3 = (int8_t) round(vy - vx);
  int8_t M4 = (int8_t) round(vy + vx);
  // repeat until desired speed is reached
  while(motor_speeds[0] != M1 || motor_speeds[1] != M2 || motor_speeds[2] != M3 || motor_speeds[3] != M4) {
    rover_set_motors(M1, M2, M3, M4);
    delay(20);
  }
}


//==============================================================
// stop the rover under consideration of the maximum acceleration.
// you have to call this function once. The function returns, after 
// the rover stops (motor values == 0)
void rover_stop(){
  while(motor_speeds[0] != 0 || motor_speeds[1] != 0 || motor_speeds[2] != 0 || motor_speeds[3] != 0) {
    rover_set_motors(0, 0, 0, 0);
    delay(20);
  }
}

void setup() {
  // initialize the M5Stack object
  m5.begin();
  // initialize I2C 
  // extended IO port: Pin 0 and 26
  Wire.begin(0, 26);
  // Grove-Connector: Pin 32 and 33
  //Wire.begin(32, 33);
  // set screen brightness
  M5.Axp.ScreenBreath(screen_brightness);

  // print a welcome message over serial porta
	Serial.println("===================");
	Serial.println("     RoverC");
	Serial.println("Bluetooth control");
	Serial.println(" v1.4 28.04.2020");
	Serial.println("===================");

  // init the text buffer display and print welcome text on the display
  tb_display_init(screen_orientation);
  tb_display_print_String("      RoverC\nBluetooth control\n\n");
  delay(2000);    
  
  // with additional battery, we need to increase charge current
  M5.Axp.SetChargeCurrent(CURRENT_360MA);
  tb_display_print_String("[OK] Charge Current\n");
  delay(2000);   
  
  // character array for formatted text output
  // big enough for complete string, including a null terminator
  char String_buffer[128]; 
  // print the Battery status three times
  // values need some time to establish
  snprintf(String_buffer, sizeof(String_buffer), "Battery status:\n%.2fV  ---  %.2fmA\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
  tb_display_print_String(String_buffer);
  delay(1500);
  snprintf(String_buffer, sizeof(String_buffer), "%.2fV  ---  %.2fmA\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
  tb_display_print_String(String_buffer);
  delay(1500);
  snprintf(String_buffer, sizeof(String_buffer), "%.2fV  ---  %.2fmA\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
  tb_display_print_String(String_buffer);
 
  // pause to make the messages readeble on the screen
  delay(1000);

  // create the Bluetooth ID
  // The chip ID is essentially its MAC address(length: 6 bytes)
  chipid = ESP.getEfuseMac();
  sprintf( chipname, "M5StickC_%04X", (uint16_t)(chipid >> 32));
  // and print it to the serial port an on the screen
  Serial.printf("\n\nBluetooth: %s\n", chipname);
  snprintf(String_buffer, sizeof(String_buffer), "[--]  Bluetooth name:\n --> %s\n", chipname);
  tb_display_print_String(String_buffer);
  // pause to make the messages readeble on the screen
  delay(1000);
  // start Bluetooth
  if(SerialBT.begin(chipname))
    tb_display_print_String("[OK] Bluetooth Init\n");
  else
    tb_display_print_String("[ERR] Unable to start Bluetooth!\n");
  // pause to make the messages readable on the screen
  delay(1000);
  
  last_second = second(now());
  tb_display_print_String("\n... ready ...\n");
}

void loop() {
  M5.update();

  // get actual time
  time_t t=now();

  // Button A initiates a demo movement
  if (M5.BtnA.wasPressed()){
    tb_display_print_String("\n.. DEMO ..\n");
    delay(3000);
    // simple square movement
    code_queue_add(0);
    code_queue_add(0);
    code_queue_add(90);
    code_queue_add(90);
    code_queue_add(180);
    code_queue_add(180);
    code_queue_add(270);
    code_queue_add(270);
    // circular movement
    for(int n=0; n<=18; n++)
      code_queue_add(n*20);
    process_state = ps_run_program;
  }

  // Button B shows the Battry status
  if (M5.BtnB.wasPressed()){
    rover_stop();
    char String_buffer[128]; 
    snprintf(String_buffer, sizeof(String_buffer), "Battery status:\n%.2fV  ---  %.2fmA\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
    tb_display_print_String(String_buffer);
    delay(1500);
    snprintf(String_buffer, sizeof(String_buffer), "%.2fV  ---  %.2fmA\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
    tb_display_print_String(String_buffer);
  }

  // call every second:
  if (second(t) != last_second) {

    // state machine
    switch (process_state) {
      // stop the rover
      case ps_stop: {   
        rover_stop();  
        process_state = ps_wait_for_BT;
        break;
      }

      // wait until the first cmd was received vis Bluetooth
      case ps_wait_for_BT: {   
        if(SerialBT.available()){
          read_String = "";
          code_queue_clear();
          // timout for BT data
          last_BT_cmd = millis();
          process_state = ps_recv_BT_codes;
        }
        break;
      }

      // receive all commands via Bluetooth until a '*' is received
      // or if no data is received for 5 seconds
      case ps_recv_BT_codes: {   
        while(SerialBT.available()){
          int data = SerialBT.read();
          // only numbers
          if(isDigit(data)){ 
            read_String += (char)data;
          }
          // space is the delimiter between direction codes
          if(data == ' ' || (data == '*' && read_String != "")){
            int value = read_String.toInt();
            if(value >=0 && value <= 360){
              code_queue_add(value);
            }
            read_String = "";
          }
          // * is the start command for the motion sequence
          if(data == '*'){
            tb_display_print_String("\n-START-\n");
            process_state = ps_run_program;
          }
          // timout for BT data
          last_BT_cmd = millis();
        }
        // 5 seconds timeout after last BT cmd
        if(millis() > last_BT_cmd+5000){
          tb_display_print_String("\n-START-\n");
          process_state = ps_run_program;
        }
        break;
      }

      // run the movement programm out of the command_queue
      case ps_run_program: {  
        char String_buffer[128]; 
        int value = 0;
        // proceed all commands out of the queue
        // code_queue_get() returns false if the queue is empty
        while(code_queue_get(&value)){
          snprintf(String_buffer, sizeof(String_buffer), "--> %i\n", value);
          tb_display_print_String(String_buffer);
          // The function move_rover() also includes the acceleration. 
          // Therefore it can take a different amount of time to proceed.
          move_rover(value, 50);
          // let the rover move for 250ms (excluding the acceleration phase)
          delay(250);
        } 
        rover_stop();  
        tb_display_print_String("\n-FINISH-\n");
        // display the battery status
        snprintf(String_buffer, sizeof(String_buffer), "Battery status:\n%.2fV  ---  %.2fmA\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
        tb_display_print_String(String_buffer);
        process_state = ps_stop;
        break;
      }
    }

    last_second = second(t);
  }
}
