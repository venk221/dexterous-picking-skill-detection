clc; clear; close all; 

p = panda_whisperer(true, true);
p.getRosParams();

pause(2);

% [L,R] = p.getFK()

loop_rate = 10;
rate_ctrl = rateControl(loop_rate); %10Hz
reset(rate_ctrl);
fprintf("Starting Panda visual servoing control loop\n")
while p.r_em > 4 || p.l_em > 4
    % visual servo until error magnitude is < 4 pixels in both images

    % update visual servoing controller
    p.updateController();

    % ensure control loop runs at desired loop rate
    waitfor(rate_ctrl);
end