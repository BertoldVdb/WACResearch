# Apple 'Wireless Accessory Configuration' (WAC) research
## Introduction
This repository contains some research on how the WAC protocol works. I was mostly interested to know how the WiFi network credentials are protected from being stolen.
This lead to two main findings:

 1. The IoT device does not authenticate the client in any way. A malicious configurator can configure any device. A demo video can be found [here](https://www.youtube.com/watch?v=pEIG0Prjm5A). 
 2. While the iPhone does authenticate the IoT device (with a hardware chip), there is no binding between the device identity and its private key. As such, an extracted key can be used to spoof any device and obtain the credentials. A demo video of this attack can be seen [here](https://www.youtube.com/watch?v=FUDFplPQymU). 

To carry out the second attack you will need to desolder a chip from a legitimate accessory and mount it on another board. 

## Repository contents
### Software:
This repository contains three major parts:

 1. A [library](https://github.com/BertoldVdb/WACResearch/tree/master/authchip) for interfacing with the MFI chip.
 2. An [application](https://github.com/BertoldVdb/WACResearch/tree/master/authserver) that can share access to the MFI chip over a network. While not needed, this will simplify your experimentation as you can now run the rest of the code on any system, even without attaching the chip to it.
 3. A Golang [library](https://github.com/BertoldVdb/WACResearch/tree/master/waclib) that implements the WAC protocol. It contains an implementation of the client and server. Only the server needs access to an MFI chip to work.

### Hardware:
A breakout board for the authentication chips that can be mounted on a Raspberry Pi Zero is provided [here](https://github.com/BertoldVdb/WACResearch/tree/master/hardware). To use it you need to add the following to your config.txt:

    dtoverlay=i2c-gpio,bus=2,i2c_gpio_sda=27,i2c_gpio_scl=17
    dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=19,i2c_gpio_scl=13

Bus 2 is used to access a MFI337S3959 (2.0C, addr=0x11) chip, while bus 3 is used to access a MFI343S00177 (3.0, addr=0x10) chip. It should be noted the library has only been tested using the 2.0C chip, but the code for 3.0 is likely pretty close to what is needed as it is based on Apple open source code.

When using this breakout board, start the authentication server as follows:
`authserver/authserver platform:/dev/i2c-2:22 platform:/dev/i2c-3:6:0x10`

The finished board can be seen here:
![MFI chip breakout board](https://raw.githubusercontent.com/BertoldVdb/WACResearch/master/hardware/breakout.png)Simply mount the de-soldered chip onto the matching footprint and you should be good to go :).

If you are not confident soldering such a small chip, it may be possible to solder wires to the usually rather large I2C bus pull-up resistors in the accessory. I didn't try this, make sure to check that the voltage levels are compatible...

The library includes support to access the chip using a MCP2221(A) USB to I2C adapter. This can be another option to make an easy experimental setup. To use this, run the authentication server as follows:
`authserver/authserver usb`

## Software usage
### Configuring a device:
Connect the WiFi card of your computer to the accessory you want to configure. It will provide an open access point with DHCP. Run the client example as follows:

    waclib/examples/client/client -ssid thessid -password thepassword
Follow the instructions shown on the screen. The device should connect to the network you configure

### Receiving configuration from iPhone:
The first step is setting up an access point that provides the required WAC beacon. A program to encode and decode such beacons is included. Its help is pretty self explanatory:

    waclib/examples/beacon/beacon -h

The default flags are for a normal device, if you change the 7 to F the device will also ask for the AirPlay password.
After running the beacon program with desired options it will output the WAC IE, for example:

> Encode result:    dd2400a0400007060011223344550002700301044e616d6502067761636c6962030454797065

If you are using hostapd to make the access point, simply put this hex string as a 'vendor_elements=' configuration entry. Set up the access point to not use any encryption and provide DHCP.

When the hostapd is running you should see your configured accessory parameters in the iPhone WiFi settings application.

Next you need to run the server application. This will handle the actual WAC protocol. Before you continue ensure you have a working MFI chip connected to your workstation in some way. To ensure the chip is working, try the authserver program as it performs an elaborate self-test while initializing the chip. If you see output like this, it is working:

> -> Chip ready: Type=2.0C FirmwareVersion=1 ProtocolVersion=2.0 Signature=20->128
> -> Registering as 'IPA_xxxxxxxxxxxxxxxxxxx' and '0'

The xxxx will be replaced with the unique serial number of your chip. Proceed to run the WAC server usage command:
`waclib/examples/server/server -h`

The parameters have the following function

 - Network interface configuration:
   - -apiface: Specify name of access point (hostapd) interface. If it is the same as the client interface, you can omit it.
   - -clientiface: Specify the name of the client interface. This is the interface that will join the configured network and is used for confirming the configuration.
 - Chip access:
   - -chip: Use a local chip. The format is the same as for the authserver. For example "-chip usb" will use a local USB to I2C bridge
   - -url: Specify url to access the authentication server. By default localhost with the default authserver configuration is used.
 - Discovery related:
   - -deviceid: A MAC address, should match what was entered in the beacon application.
   - -port: Port to run the WAC HTTP server on, value is not critical but you need to be able to bind to it.
- Functionality:
   - -cfgscript: A script that will be executed with the configued WiFi SSID and password. It should connect the client interface to the desired network.
  - -honeypot: Just listen for requests and log them but never complete the process. You do not need the client interface.

If the server program is running you should be able to complete the WAC procedure on the iPhone and obtain the configured plaintext credentials.

