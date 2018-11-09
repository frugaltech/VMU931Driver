# VMU931Driver
Simple Python driver for Variense VMU931 with an integrated test showing some issues with the device. Based in part on PyVMU. Simply run the driver by itself to start a loop querying the attached device for gyro data. After a few minutes, the device is likely to stop streaming data. It seems there is a firmware bug somewhere. If you keep the connection open across requests rather than opening and closing it on each request, the device keeps streaming data for longer - but it fails eventually in any case.

The driver implements getting and setting the device configuration and querying for gyro, euler, magnetometer and acceleration data. I haven't implemented heading or quaternions yet but it seems easy enough to do according to the device manual available at https://variense.com/Docs/VMU931/VMU931_UserGuide.pdf

Feel free to modify, comment, ... this is mostly an effort to get Variense support to fix their (potentially) nice device.
