
cam = serialport("COM4", 10000000);
configureCallback(cam,"terminator",@readSerialData);




