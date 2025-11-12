clear all
close all
clc

% addpath("../origami_continuum_misc");

%% SETUP ORIGAMI MANIPULATOR

                           % %arduino_master = arduino('com51','uno');
                           % module1 = i2cdev(arduino_master,'0x9');
                           % module2 = i2cdev(arduino_master,'0x8');
                           % module3 = i2cdev(arduino_master,'0xA');

arduino_master = arduino('/dev/ttyACM0' ,'uno', 'Libraries', 'I2C');
%arduino_master = arduino('/dev/cu.usbmodem14301', 'uno', 'Libraries', 'I2C')
%%
% module1 = device(arduino_master, 'I2CAddress', '0xE', 'Bus', 0);
% module2 = device(arduino_master, 'I2CAddress', '0xD', 'Bus', 0);
% module3 = device(arduino_master, 'I2CAddress', '0xC', 'Bus', 0);
% module4 = device(arduino_master, 'I2CAddress', '0xB', 'Bus', 0);
module5 = device(arduino_master, 'I2CAddress', '0xB', 'Bus', 0);

total_section = 1;

%% INITIALIZE MODULES TO FULLY COMPRESSED
max_module_length = 160; % unit: mm
min_module_length = 40; %unit: mm

init_max_length_hi = bitshift(max_module_length,-8);
init_max_length_lo = bitand(max_module_length,255);

init_min_length_hi = bitshift(min_module_length,-8);
init_min_length_lo = bitand(min_module_length,255); 

write(module5, [init_max_length_hi init_max_length_lo init_min_length_hi init_min_length_lo 0 0 0]);
%write(module5,zeros(1,7));
pause(10)
 
% write(module4, [init_max_length_hi init_max_length_lo+20 init_min_length_hi init_min_length_lo 0 0 0]);
% pause(10)
%  
% write(module3, [init_max_length_hi init_max_length_lo init_min_length_hi init_min_length_lo 0 0 0]);
% pause(10)
%  
% write(module2, [init_max_length_hi init_max_length_lo init_min_length_hi init_min_length_lo 0 0 0]);
% pause(10)
%  
% write(module1, [init_max_length_hi init_max_length_lo init_min_length_hi init_min_length_lo 0 0 0]);
% pause(10)

%gear_ratio = [150];
gear_ratio = [150];
m_command = zeros(total_section,3);
M = zeros(3,2,total_section);

%% CALCULATE AND SEND MOTOR COMMAND
%max_length = max_module_length / 1000;      % in meter
max_length = 0.16;
L0 = min_module_length;  % mm
a = 30; % mm

%%
% to fix the init errors
%G = [a a a a a a a a a a-20 a-20 a-20 a-20 a-20 a-20];
b = 50;
a = 65;
G = [a b a];
cable_length_traj = (G + L0)/1000;
for i = 1:total_section
  [m1_command, m2_command, m3_command] = ...
  cableLength2MotorCommand(cable_length_traj(3*(i-1)+1), cable_length_traj(3*(i-1)+2), cable_length_traj(3*(i-1)+3), max_length, gear_ratio(i));

  m_command(i,:) = [m1_command, m2_command, m3_command];
  M(:,:,i) = motorCommandConvert(m_command(i,:));
end

write(module5,[M(1,:,1) M(2,:,1) M(3,:,1) 1]);
