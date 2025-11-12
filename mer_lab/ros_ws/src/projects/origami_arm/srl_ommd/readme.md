# Soft Robotics Lab Origami Module Motor Driver


## USB HID Device Installation on Linux
The Origami Module Controller is a USB HID Device. The following steps are required to use it on a Linux system.
1. Install the HIDAPI Library with the following commands: 
```
  $ sudo apt-get install python-dev libusb-1.0-0-dev libhidapi-dev libudev-dev
  $ sudo pip install --upgrade setuptools
  $ sudo pip install hidapi
```
2. Add [99-wpi-srl-device.rules](99-wpi-srl-device.rules) to /etc/udev/rules.d

3. Refresh udev rules with
```
  $ sudo udevadm trigger
```
