clc
%% Experiment parameters
start_joint_pos = [deg2rad(132), deg2rad(-22)];
goal_joint_pos = [deg2rad(144), deg2rad(38)];
exp_no = "2";

%% Location for storing plots
% Location and folder # to store plots
str_loc = "exp/"+exp_no; % for Windows OS, modify for Ubuntu
if not(isfolder(str_loc))
    mkdir(str_loc)
end

%% Estimation variables
window = 30; % Window size
it = 1; % Iterator
st = 0;
dr = []; % sliding window for change in joint angles
ds = []; % sliding window for change in curve coefs
old_joint_pos = start_joint_pos;
old_coefs = get_fourier_coefs(old_joint_pos);
gamma1 = 10e-5; % learning rate for initial estimation
gamma2 = 10e-8; % learning rate during control loop
qhat = zeros(6,2); % initial Jacobian

%% Visual Servoing variables
thresh = 1; % servoing error threshold
error_norm = thresh; % initializing error_norm to threshold
lam = 5e-2*eye(6); % servoing gain
rate =100; % control loop rate
r = rateControl(rate); % visual servoing sampling rate
[goal_curve, goal_coefs] = get_fourier_coefs(goal_joint_pos); % goal parameters
[ee_goal, j2_goal] = get_config(goal_joint_pos); % goal joint config for visualization

%% Data record variables
ds_ = [];
dr_ = [];
Jplot = [];
err_ = [];
traj_ = [];
j_dot_ = [];

%% Video recording variables
robotvid = str_loc + "/robotVideo.avi";
curvevid = str_loc + "/curveVideo.avi";

robot_video = VideoWriter(robotvid);
robot_video.FrameRate = 25;
open(robot_video)

curve_video = VideoWriter(curvevid);
curve_video.FrameRate = 25;
open(curve_video)

%% Collect sampling data by moving robot
while(it <= window)
    % move the robot
    cur_joint_pos = start_joint_pos + [sin(st) + 0.1*((rand - 0.5)), cos(st) + 0.12*((rand - 0.5))];
    %cur_joint_pos = start_joint_pos + [sin(st), sin(st)];
    
    % current robot state
    [ee_cart_pos, j2_cart_pos] = get_config(cur_joint_pos);
    [cur_curve, cur_coefs] = get_fourier_coefs(cur_joint_pos);

    % compute change in state
    dr = [dr; get_dr(cur_joint_pos, old_joint_pos)];
    old_joint_pos = cur_joint_pos;
    
    ds = [ds; get_ds(cur_coefs, old_coefs)];
    old_coefs = cur_coefs;
    
    % Increment step
    st = st + 0.17;

    % data collection for vis
    ds_ = [ds_; ds(it,:)]; % storing change in ee pos for vis
    dr_ = [dr_; dr(it,:)]; % storing change in joint positions for vis
    traj_ = [traj_; ee_cart_pos]; % storing exploration trajectory

    % visualize robot state
    figure(1)
    plot([0, j2_cart_pos(1), ee_cart_pos(1)], [0, j2_cart_pos(2), ee_cart_pos(2)], 'r', 'LineWidth', 5); % current robot state
    hold on
    plot(traj_(:,1),traj_(:,2), 'mo')
    vis_txt = 'Initial Shape Jacobian Estimation';
    text(-100,130,vis_txt)
    txt = append('Step#:',int2str(it));
    text(50,-65,txt)
    axis([-165 165 -165 165])
    title('Robot Trajectory')
    legend({'Robot', 'EE Trajectory'},'Location','northeastoutside')
    pbaspect([1 1 1])
    grid on
    frame = getframe(gcf);
    writeVideo(robot_video, frame)
    hold off

    % visualize fourier approximation
    figure(2)
    axis([-165 165 -165 165])
    hold on
    title('Fourier Approximation')
    pbaspect([1 1 1])
    legend({'current curve'},'Location','northeastoutside')
    grid on
    plot(cur_curve, 'b')
    frame = getframe(gcf);
    writeVideo(curve_video, frame)
    hold off

    % increment iterator
    it = it + 1;
end

%% Estimate initial Jacobian
it = 1; % reset iterator
while it <= window
    [J, qhat_dot] = get_energy_fun(ds, dr, qhat, it, gamma1);
    qhat = qhat + qhat_dot;
    Jplot = [Jplot; J];
    it = it+1;
end
Q = qhat; % Initial Jacobian estimate

%% Visual Servoing control loop
while error_norm >= thresh
    
    % compute error vector and update norm
    error = cur_coefs - goal_coefs;
    error_norm = norm(error);

    % compute velocity vector using inverse Jacobian and error vector
    j_dot = get_velocity(Q, lam, error);

    % update robot state
    j_update = (1/rate)*j_dot;
    cur_joint_pos = cur_joint_pos + j_update;
    
    % current robot state
    [ee_cart_pos, j2_cart_pos] = get_config(cur_joint_pos);
    [cur_curve, cur_coefs] = get_fourier_coefs(cur_joint_pos);

    %% TO DO check this for current params
    % compute change in state   
    ds(1,:) = get_ds(cur_coefs, old_coefs); % replacing oldest data instance
    % ds = transpose(circshift(transpose(ds),-1,2)); % left circular shift
    ds = transpose(circshift(transpose(ds),-1,6));
    old_coefs = cur_coefs;

    dr(1,:) = get_dr(cur_joint_pos, old_joint_pos);
    dr = transpose(circshift(transpose(dr),-1,2)); % left circular shift
    old_joint_pos = cur_joint_pos;

    % compute Jacobian update
    [J, qhat_dot] = get_energy_fun(ds, dr, Q, window, gamma2);
    Q = Q + qhat_dot;

    % data collection
    ds_ = [ds_; ds(window,:)]; % storing change in ee pos for vis
    dr_ = [dr_; dr(window,:)]; % storing change in joint positions for vis
    Jplot = [Jplot; J]; % Store model errors for vis
    err_ = [err_; error]; % Store errors for vis
    traj_ = [traj_; ee_cart_pos]; % store end effector trajectory for vis
    j_dot_ = [j_dot_; j_dot]; % Store velocities for vis

    % visualize robot
    figure(1)
    plot([0, j2_cart_pos(1), ee_cart_pos(1)], [0, j2_cart_pos(2), ee_cart_pos(2)], 'r', 'LineWidth', 5); % current robot state
    hold on
    plot(traj_(window:end,1),traj_(window:end,2), 'b--', 'LineWidth', 2)
    plot(ee_goal(1),ee_goal(2),'g*', 'MarkerSize',12)
    goaltxt = sprintf('(%.2f, %.2f)',ee_goal(1),ee_goal(2));
    text(ee_goal(1) - 70, ee_goal(2) -20, goaltxt,'FontSize',10);
    vis_txt = 'Servoing to Goal';
    text(-100,130,vis_txt)
    txt = append('Step#:',int2str(it));
    text(50,-65,txt)
    axis([-165 165 -165 165])
    title('Robot Trajectory')
    legend({'robot','trajectory','goal'},'Location','northeastoutside')
    pbaspect([1 1 1])
    grid on
    frame = getframe(gcf);
    writeVideo(robot_video, frame)
    hold off
    
    % visualize curve
    figure(2)
    axis([-165 165 -165 165])
    hold on
    title('Fourier Approximation')
    pbaspect([1 1 1])
    grid on
    plot(cur_curve,'r-')
    plot(goal_curve, 'g')
    % plot(curve,'r-',skel(:,1),skel(:,2),'k.')
    % plot(goal_curve,'g',goal_skel(:,1),goal_skel(:,2),'g*')
    legend({'current curve', 'goal curve'},'Location','northeastoutside')
    frame = getframe(gcf);
    writeVideo(curve_video, frame)  
    hold off
    
    % increment iterator
    it = it + 1;
    
    % maintain loop rate
    waitfor(r);
end

close(robot_video)
close(curve_video)

%% Data Visualization
disp('Generating results')

figure()
plot(Jplot)
title('Model Error')
xlabel('Steps')
ylabel('Estimation Error')
model_err = str_loc+"/model_err.png";
saveas(gcf, model_err)

figure()
subplot(2,1,1)
plot(j_dot_(:,1))
title('Velocity J1')
xlabel('steps')
ylabel('Velocity')

subplot(2,1,2)
plot(j_dot_(:,2))
title('Velocity J2')
xlabel('steps')
ylabel('Velocity')
vel = str_loc+"/vel.png";
saveas(gcf, vel)

%figure()
%subplot(6,1,1)
figure()
t = tiledlayout(3,2)
nexttile
plot(err_(:,1))
title('coefficient 1')
xlabel('steps')
ylabel('error')

%subplot(6,2,2)
nexttile
plot(err_(:,2))
title('coefficient 2')
xlabel('steps')
ylabel('error')

%subplot(6,3,3)
nexttile
plot(err_(:,3))
title('coefficient 3')
xlabel('steps')
ylabel('error')

%subplot(6,4,4)
nexttile
plot(err_(:,4))
title('coefficient 4')
xlabel('steps')
ylabel('error')

%subplot(6,5,5)
nexttile
plot(err_(:,5))
title('coefficient 5')
xlabel('steps')
ylabel('error')

%subplot(6,6,6)
nexttile
plot(err_(:,6))
title('coefficient 6')
xlabel('steps')
ylabel('error')

err = str_loc+"/coeff_err.png";
saveas(gcf, err)

for i = 1:length(err_)
    err_norm(i) = norm(err_(i,:));
end
figure()
plot(err_norm)
title('Coefficient Error Magnitude')
legend('coefficient error magnitude')
xlabel('steps')
ylabel('error magnitude')
err_normloc = str_loc+"/coeff_err_norm.png";
saveas(gcf,err_normloc)

%% Save Trajectory Plot
figure(1)
traj = str_loc+"/traj.png";
saveas(gcf, traj)

%% Video Processing
%video_processing(str_loc);