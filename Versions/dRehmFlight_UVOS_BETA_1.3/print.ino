//========================================================================================================================//
//                                                 DEBUG PRINT FUNCTIONS                                                  //
//========================================================================================================================//

#define PRN_BUFFER_SIZE  128
static char prn_buffer[PRN_BUFFER_SIZE] = {0};

void printRadioData()
{
  if ( current_time - print_counter > 50000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "CH1: %-6dCH2: %-6dCH3: %-6dCH4: %-6dCH5: %-6dCH6: %-6d\r\n", \
          ( int )channel_1_pwm, \
          ( int )channel_2_pwm, \
          ( int )channel_3_pwm, \
          ( int )channel_4_pwm, \
          ( int )channel_5_pwm, \
          ( int )channel_6_pwm \
        );
    // hw.usb_handle.TransmitInternal((uint8_t*)prn_buffer, prn_buflen);
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printDesiredState()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "thro_des: %5.3f roll_des: %5.3f pitch_des: %5.3f yaw_des: %7.2f\r\n", \
          thro_des, \
          roll_des, \
          pitch_des, \
          yaw_des \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printGyroData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "GyroX: %7.1f GyroY: %7.1f GyroZ: %7.1f\r\n", \
          GyroX, \
          GyroY, \
          GyroZ \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printAccelData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "AccX: %7.3f AccY: %7.3f AccZ: %7.3f\r\n", \
          AccX, \
          AccY, \
          AccZ \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printMagData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "MagX: %7.1f MagY: %7.1f MagZ: %7.1f\r\n", \
          MagX, \
          MagY, \
          MagZ \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printRollPitchYaw()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "roll: %7.3f pitch: %7.3f yaw: %7.3f\r\n", \
          roll_IMU, \
          pitch_IMU, \
          yaw_IMU \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printPIDoutput()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "roll_PID: %6.3f  pitch_PID: %6.3f  yaw_PID: %6.3f\r\n", \
          roll_PID, \
          pitch_PID, \
          yaw_PID \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printMotorCommands()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "m1_command: %6.3f m2_command: %6.3f m3_command: %6.3f m4_command: %6.3f m5_command: %6.3f m6_command: %6.3f\r\n", \
          ( double )m1_command_PWM, \
          ( double )m2_command_PWM, \
          ( double )m3_command_PWM, \
          ( double )m4_command_PWM, \
          ( double )m5_command_PWM, \
          ( double )m6_command_PWM \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printServoCommands()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    int prn_buflen \
      = snprintf \
        ( prn_buffer, sizeof( prn_buffer ), \
          "s1_cmd: %5d s2_cmd: %5d s3_cmd: %5d s4_cmd: %5d s5_cmd: %5d s6_cmd: %5d\r\n", \
          ( int )s1_command_PWM, \
          ( int )s2_command_PWM, \
          ( int )s3_command_PWM, \
          ( int )s4_command_PWM, \
          ( int )s5_command_PWM, \
          ( int )s6_command_PWM  \
        );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}

void printLoopRate()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("dt = "));
    // Serial.println(dt*1000000.0);
    int prn_buflen = snprintf ( prn_buffer, sizeof( prn_buffer ), "dt(sec.) = %10.5f\r\n", dt  );
    Devices::debug_uart.BlockingTransmit((uint8_t*)prn_buffer, prn_buflen);
  }
}