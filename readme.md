# Long range demo kit
This application is a demo kit that can be used to easily test the Bluetooth 5 long range feature with the nRF52840.

## Requirements
* 2x nRF52840-DK (with battery, if it is not connected to PC during the testing)
* A PC running either Windows, Linux, or Mac OS-X with [nrfjprog](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_installation.html&cp=5_1_1) installed
* The latest version of the J-Link [Software and Documentation pack](https://www.segger.com/downloads/jlink#)
* 1 or 2 micro USB cables
### Optional, for  logging and debugging
* Serial Port Viewer (for example Tera Term or Realterm)
* [Segger Embedded Studio](https://www.segger.com/products/development-tools/embedded-studio/)
* [nRF5_SDK_v15.0.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/)

## How to test long range
1. Program the two nRF52840-DK: one peripheral and one central 

    a) Peripheral: program peripheral_long_range_demo_pca10056_s140.hex found in the following folder:  ..\nrf52840-long-range-demo-kit\peripheral_long_range_demo_kit\hex
    
    b) Central: program central_long_range_demo_pca10056_s140.hex found in the following folder: ..\nrf52840-long-range-demo-kit\central_long_range_demo_kit\hex

2. Use the buttons on the two nRF52840-DKs to select mode. 
3. Ready to test! Prepare yourself for a long walk!

## Compile, debug, log
* Compile and debug: The demo is made with SDK 15.0.0 and the nrf52840-long-range-demo-kit folder should be placed in the following folder: ..\nRF5_SDK_15.0.0_a53641a\examples\ble_central_and_peripheral\
* Log: To see the logged information, use either a Serial Port Viewer (UART log) or RTT viewer. 

**Note 1:** The central filters devices based on device name, both when initiating a connection and when reporting RSSI of the advertiser. The default device name is "Long range demo".

**Note 2:** For non-connectable advertising on coded phy, the central does not filter on device name because the device name is not included in the advertising packet. 

## Buttons and LED configurations
### Peripheral 


<table>
<tr><th>  </th><th>  </th></tr>
<tr><td>

| LEDs        |            |                                                                               
| ------------- |:-------------:|                                                                      
| LED 1      | «on»: coded phy  |     
|            | «slow blinking»: 1Mbps     |                
| LED 2      | «on»: 0 dBm |    
|            | «slow blinking»: 8 dBm  |                 
| LED 3 |   «fast blinking»: non-connectable advertising  |   
| ·                |                   |                                         
| LED 4 |  «fast blinking»: connectable advertising  |  
|            |  «on»: connected state   |     

</td><td>

| Buttons        |            |       
| ------------- |:-------------:|                                  
| Button 1      |  Switch between coded phy and 1Mbps |  
|     ·          |                          |                  
| Button 2      | Switch between 0 dbm and 8 dBm |    
| ·                |                   |                     
| Button 3 |  Switch between non-connectable and connectable advertising  |
| ·                |                   |          
| Button 4  |  Not in use |          
| ·        |            |                         

</td></tr> </table>
               

### Central 

<table>
<tr><th>  </th><th>  </th></tr>
<tr><td>

| LEDs        |            |                                                                                          
| ------------- |:-------------:|                                                                                     
| LED 1      | «on»: coded phy|    
|     | «slow blinking»: 1Mbps|                                  
| LED 2      | «on»: 0 dBm |     
|     |  «slow blinking»: 8 dBm |                              
| LED 3 |   «on»: scanning, trying to connect | 
| | «slow blinking»: scanning|                 
| LED 4 |  «on»: connected state |     
| | «toggling»: changing state upon received adv report |     


</td><td>

| Buttons        |            |       
| ------------- |:-------------:|                                  
| Button 1      |  Switch between coded phy and 1Mbps |     
| ·        |            |                
| Button 2      | Switch between 0 dbm and 8 dBm |       
| ·        |            |                   
| Button 3 | Switch between «scanning» and «scanning, trying to	connect» |
| ·        |            | 
| Button 4  |  Not in use |
| ·        |            |                                      

</td></tr> </table>


## Default parameters
### Peripheral
* Coded phy
* 8 dBm
* Connectable advertising

### Central
* Coded phy
* 8 dBm
* Scanning only
* If connected to PC: log RSSI over UART/RTT  


## About this project
This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub.

The application is built to be used with the official nRF5 SDK, that can be downloaded from https://www.nordicsemi.no 

Please post any questions about this project on https://devzone.nordicsemi.com.                   
                                  
                                  
                                  