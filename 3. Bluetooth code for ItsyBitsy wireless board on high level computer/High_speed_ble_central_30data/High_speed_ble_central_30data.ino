/*********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today 
  in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

/*
   This sketch demonstrate the central API(). A additional bluefruit
   that has bleuart as peripheral is required for the demo.
*/
#include <bluefruit.h>

BLEClientUart clientUart; // bleuart client
int rs232_datalength = 20;
int ble_datalength = 32;
uint32_t rx_count = 0;
char data_rs232_rx[247] = { 0 };
char data_ble_rx[247] = { 0 };
void setup()
{
  Serial.begin(115200);
  //while ( !Serial )
  delay(10);   // for nrf52840 with native usb

//  Serial.println("Bluefruit52 Central BLEUART Example");
//  Serial.println("-----------------------------------\n");

  // Config the connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);

  Bluefruit.setName("Bluefruit52 Central");

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Don't use active scan
     - Start(timeout) with timeout = 0 will scan forever (until connected)
  */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
   Callback invoked when scanner pick up an advertising data
   @param report Structural advertising data
*/
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
//    Serial.print("BLE UART service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  } else
  {
    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();
  }
}

/**
   Callback invoked when an connection is established
   @param conn_handle
*/
void connect_callback(uint16_t conn_handle)
{
//  Serial.println("Connected");

//  Serial.print("Discovering BLE Uart Service ... ");
  if ( clientUart.discover(conn_handle) )
  {
//    Serial.println("Found it");
//    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();
//    Serial.println("Ready to receive from peripheral");
  } else
  {
//    Serial.println("Found NONE");
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
//  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

/**
   Callback invoked when uart received data
   @param uart_svc Reference object to the service where the data
   arrived. In this example it is clientUart
*/
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  ble_datalength = uart_svc.available();
  uart_svc.readBytes(data_ble_rx, ble_datalength);
  Serial.write(data_ble_rx, ble_datalength);
  // Serial.println(data_ble_rx[2],DEC);
//  Serial.print("Received from peripheral: ");
//  for (int i = 0; i < ble_datalength; i++)
//  {
//    Serial.print(data_ble_rx[i],DEC);
//    Serial.print("  ");
//  }  
//  Serial.println("");
}

void loop()
{
  // Discovered means in working state
  // Get Serial input and send to Peripheral
  if (Serial.available() >= rs232_datalength )
  {
    data_rs232_rx[0] = Serial.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial.read();
      if (data_rs232_rx[1] == 90 | data_rs232_rx[1] == 95) 
      {
        data_rs232_rx[2]=Serial.read();
        if (data_rs232_rx[2] == rs232_datalength)
        {
        Serial.readBytes(&data_rs232_rx[3], rs232_datalength - 3);
        }
        if ( Bluefruit.Central.connected() && clientUart.discovered() )
        {
          clientUart.write(data_rs232_rx, rs232_datalength);
          
        }
//      
////      Serial.print("Sent to peripheral: ");
////      for (int i = 0; i < rs232_datalength; i++)
////      {
////        Serial.print(data_rs232_rx[i],DEC);
////        Serial.print("  ");
////      }
////      Serial.println("");
      }
    }
  }
//  data_rs232_rx[0] = 165;
//  data_rs232_rx[1] = 90;
//  data_rs232_rx[2] = data_rs232_rx[2] + 1;
//  if ( Bluefruit.Central.connected() && clientUart.discovered() )
//  {
//    clientUart.write(data_rs232_rx, rs232_datalength);
//  }
//  delay(1000);
}
