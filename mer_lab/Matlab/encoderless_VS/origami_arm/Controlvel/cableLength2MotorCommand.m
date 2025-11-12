function [m1_command, m2_command, m3_command]  = cableLength2MotorCommand(length1, length2, length3, max_length, gear_ratio)
%     encoder_multiplier = 14;
    encoder_multiplier = 7;        % HIGH encoder pulses per revolution
    quadrature_count_multiplier = 4; % 4X quadrature count mode, this is set in the origami PCB firmwar
    
    %     motor_shaft_hub_diameter = 0.009; % in m, measured using caliper
    %motor_shaft_hub_diameter = 0.0054;  % in m, measured using caliper
    motor_shaft_hub_diameter = 0.007;   % in m, measured using caliper
    motor_shaft_hub_circum = pi*motor_shaft_hub_diameter;
%     m1_command = ceil((max_length - length1)/motor_shaft_hub_circum*gear_ratio*encoder_multiplier); % motor command in encoder ticks as a unit, obtained by dividing delta length over m/rotation of the motor hub 
%     m2_command = ceil((max_length - length2)/motor_shaft_hub_circum*gear_ratio*encoder_multiplier);
%     m3_command = ceil((max_length - length3)/motor_shaft_hub_circum*gear_ratio*encoder_multiplier);
    
    m1_command = ceil((max_length - length1)/motor_shaft_hub_circum*gear_ratio*encoder_multiplier*quadrature_count_multiplier); %old circumference = 0.01455:, /0.012*83;   % motor command in encoder ticks as a unit, obtained by dividing delta length over m/rotation of the motor hub 
    m2_command = ceil((max_length - length2)/motor_shaft_hub_circum*gear_ratio*encoder_multiplier*quadrature_count_multiplier); %/0.0109*68;
    m3_command = ceil((max_length - length3)/motor_shaft_hub_circum*gear_ratio*encoder_multiplier*quadrature_count_multiplier); %/0.012*79;%

end
