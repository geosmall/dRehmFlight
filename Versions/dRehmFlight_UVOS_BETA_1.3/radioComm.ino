//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

//Create SerialReceiver of type IBUS
SerialReceiver ibus_rx(SerialReceiver::IBUS);

ParsedMessage msg; // uint16_t channels[SERRX_NUM_CHAN];

void radioSetup() {

  //Create a config for SerialReceiver object and initialize it
  SerialReceiver::Config ser_rx_config;
  ser_rx_config.periph = UartHandler::Config::Peripheral::USART_6;
  ser_rx_config.rx = {UVS_GPIOC, 7};
  ser_rx_config.tx = {UVS_GPIOC, 6};

  ibus_rx.Init(ser_rx_config);

  //Start the SerialReceiver
  ibus_rx.StartRx();

}

void rxPoll() {
  //DESCRIPTION: Poll Rx, should be called from the main loop
  bool msgAvailable = ibus_rx.Listener();
  //Check for new message available from serial Rx
  if (msgAvailable)
  {
    ibus_rx.GetMessage(&msg); //Retrieve message from Rx FIFO
  }
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from ParsedMessage msg
  if ((ch_num >= 0) && (ch_num <= (int)(SERRX_NUM_CHAN-1))) {
    return (unsigned long)msg.channels[ch_num];
  } else {
    return 0; //Return 0 if out of bounds channel
  }
}