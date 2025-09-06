This folder contains my experiments with ESPHOME's host platform.


### Project: ESPH-SYSMON

  A sample DOCKER COMPOSE setup running ESPHOME's host platform.
  Also demonstrates how to perform system monitoring using lamda sensors.

  - Status: Works Fine
  - Problems: None (!?!)


### Components: I2C

  I2C driver for ESPHOME on LINUX. Requires I2C LINUX kernel driver.

  - Status: Works (Hack)
  - Problems: No I2C Clock Frequency Management
  - Todo: I2C Bus Scanning


### Components: CH341_I2C

  LIBUSB based driver for the CH341 USB to I2C chip. Does not require any drivers.

  - Status: Works (Hack)
  - Problems: Limited Transfer Length
  - Todo: Support more than one chip, Better LIBUSB integration, Hotplug detection, Make ESPHOME recognize this as 'i2c:' bus driver
