Dynamixel MX64 MATLAB library
=============================

A wrapper class to interface with Robotis's USB2Dynamixel and control MX64 series servos using communication version 1.0.

## Requirements
Usage of this library requires Robotis's USB2Dynamixel SDK, available at http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel.htm

## Usage
The bin and import directories from Robotis's SDK must be in your path before initializing the library.  This can be done through the MATLAB GUI, or the command
```matlab
addpath('.\dxl_sdk_win32_v1_02\bin', '.\dxl_sdk_win32_v1_02\import');
```

The library is then initialized by providing the port to connect through, and the baudrate the servos are configured for. The example below initializes an object named Dynamixel to use COM port 31 at 1Mb (1000000 bps).
```matlab
Dynamixel = MX64(31,1000000);
```

Commands can then be called through dot object syntax.
```matlab
Dynamixel.position(ID,angle);
Dynamixel.setPID(ID,Kp,Ki,Kd);
Dynamixel.setAngleLimit(ID,lowerLimit,upperLimit);
```


## Changelog
* April 2015 - Initial library release.

## More Information
* Robotis e-Manual<br/>
  [http://support.robotis.com](http://support.robotis.com)
* MX Series Control Table<br/>
  [http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm](http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm)
* API Reference<br />
  [http://support.robotis.com/en/software/dynamixel_sdk/api_reference.htm](http://support.robotis.com/en/software/dynamixel_sdk/api_reference.htm)
* Example MATLAB code
  [http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel/window_communication_1/matlab.htm](http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel/window_communication_1/matlab.htm)

## License
Released under the MIT license, see `LICENSE.txt`
