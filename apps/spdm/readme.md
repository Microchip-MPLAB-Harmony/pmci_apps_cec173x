[![MCHP](https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png)](https://www.microchip.com)

# SPDM (Secure Protocol and Data Model) message exchange

This example application demonstrates SPDM message exchange between CEC173x running as an SPDM device and an SPDM capable host (an attester).

## Description

This example uses the SPDM library along with MCTP library, SMBUS driver and SMBUS peripheral library to receive SPDM request messages from a host and send back a SPDM response message

## Downloading and building the application

To clone or download this application from Github, go to the [main page of this repository](https://github.com/Microchip-MPLAB-Harmony/pmci_apps_cec173x) and then click **Clone** button to clone this repository or download as zip file.
This content can also be downloaded using content manager by following these [instructions](https://github.com/Microchip-MPLAB-Harmony/contentmanager/wiki).

Path of the application within the repository is **apps/spdm/firmware** .

To build the application, refer to the following table and open the project using its IDE.

| Project Name      | Description                                    |
| ----------------- | ---------------------------------------------- |
| spdm_demo.X | MPLABX project for [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A)     |
|||

## Setting up the hardware

The following table shows the target hardware for the application projects.

| Project Name| Board|
|:---------|:---------:|
| spdm_demo.X | [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A) 
|||

### Setting up [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A)

- Interface the [Aardvark I2C/SPI Host adapter](https://www.totalphase.com/catalog/product/view/id/2/s/aardvark-i2cspi/?GA_network=g&GA_device=c&GA_campaign=9527865813&GA_adgroup=123178533820&GA_target=&GA_placement=&GA_creative=522202133248&GA_extension=&GA_keyword=aardvark%20usb&GA_loc_physical_ms=9061891&GA_landingpage=https://www.totalphase.com/catalog/product/view/id/2/s/aardvark-i2cspi/&ga_keyword_match=e&ga_ad_position=&gclid=Cj0KCQiAqOucBhDrARIsAPCQL1ZEkMYUWtPbkQwv6DorBIbkHLq7YfA3ZDtB47hNQ49LSObCFBtchIUaAmj0EALw_wcB) by making the following connections:
    - Connect a wire from GPIO140 (I2C06_SCL) available on Pin 28 of P4 header to the SCL pin of the I2C capable device
    - Connect a wire from GPIO132 (I2C06_SDA) available on Pin 26 of P4 header to the SDA pin of the I2C capable device
    - Connect GND and VCC between the [Aardvark I2C/SPI Host adapter](https://www.totalphase.com/catalog/product/view/id/2/s/aardvark-i2cspi/?GA_network=g&GA_device=c&GA_campaign=9527865813&GA_adgroup=123178533820&GA_target=&GA_placement=&GA_creative=522202133248&GA_extension=&GA_keyword=aardvark%20usb&GA_loc_physical_ms=9061891&GA_landingpage=https://www.totalphase.com/catalog/product/view/id/2/s/aardvark-i2cspi/&ga_keyword_match=e&ga_ad_position=&gclid=Cj0KCQiAqOucBhDrARIsAPCQL1ZEkMYUWtPbkQwv6DorBIbkHLq7YfA3ZDtB47hNQ49LSObCFBtchIUaAmj0EALw_wcB) and the [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A) 
    - Open the [Control Center Serial Software](https://www.totalphase.com/products/control-center-serial/)
    - Enter the slave address as 0x64 under the master tab
    - Enter the slave address as 0x62 under the slave tab and Enable the slave
- Connect the Debug USB port on the board to the computer using a micro USB cable

## Running the Application

1. Build and Program the application using its IDE
2. Send SPDM request messages using the [Control Center Serial Software](https://www.totalphase.com/products/control-center-serial/)
3. Skip the "Destination Slave Address" field (first byte) while entering the control message in the message tab of the [Control Center Serial Software](https://www.totalphase.com/products/control-center-serial/)
4. For every SPDM request message a response message can be seen under the transaction log window of the [Control Center Serial Software](https://www.totalphase.com/products/control-center-serial/)