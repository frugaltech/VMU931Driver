# VMU931Driver
Simple Python driver for Variense VMU931 with an integrated test showing some issues with the device. Based in part on PyVMU. Simply run the driver by itself to start a loop querying the attached device for gyro data. After a few minutes, the device is likely to stop streaming data. It seems there is a firmware bug somewhere.

The driver implements getting and setting the device configuration and querying for gyro, euler, magnetometer and acceleration data. I haven't implemented heading or quaternions yet but it seems easy enough to do according to the device manual available at https://variense.com/Docs/VMU931/VMU931_UserGuide.pdf


