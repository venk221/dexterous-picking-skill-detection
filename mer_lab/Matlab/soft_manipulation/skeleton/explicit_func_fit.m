clear
clc

%% Robot specifications
cur_joint_pos = [deg2rad(115), deg2rad(-25)];
r = rateControl(100); % control rate
itr = 0;
param1 = 0.1;
param2 = 0.5;

while(itr <= 40)
    %% Compute end effector Position
    [ee_pos, j2_pos] = compute_cart_pos(cur_joint_pos);
    
    %% Fit curve
    [curve, coefs, skeleton] = fit_explicit_curve(cur_joint_pos);
    
    %% Apply velocity to joints
    j_update = (1/100)*[20*sin(param1), 30*sin(param2)];
    cur_joint_pos = cur_joint_pos + j_update;
    
    %% Visualization
    figure(1)
    plot([0, j2_pos(1), ee_pos(1)], [0, j2_pos(2), ee_pos(2)], 'r', 'LineWidth', 5); % current robot state
    hold on
    txt = append('Step#:',int2str(itr));
    text(50,-45,txt)
    axis([-165 165 -165 165])
    title('Robot Vis')
    pbaspect([1 1 1])
    grid on
    hold off
    
    
    figure(2)
    hold on
    plot(curve,skeleton(:,1),skeleton(:,2))
    axis([-165 165 -165 165])
    title('Curve fit')
    legend({'current robot pose', 'skeleton'},'Location','northeastoutside')
    pbaspect([1 1 1])
    grid on
    hold off
    
    %% video
    


    %% Increase itr
    itr = itr + 1;
    param1 = param1 + 0.25;
    param2 = param2 + 0.25;
    waitfor(r);
end