%% Robot parameters
start_joint_pos = [deg2rad(20), deg2rad(-45)];
exp_no = "9"
%% Sampling parameters
it = 1;
window = 100;
cur_joint_pos = start_joint_pos;
freq = 100;
r = rateControl(freq);
% vel = [0.75, 0.5];
vel = [0, 90];

%% Data logging variables
traj_ = [];
coefs_ = [];
vel_ = [];

str_loc = "exp/fourier_coefs/"+exp_no; % for Windows OS, modify for Ubuntu
if not(isfolder(str_loc))
    mkdir(str_loc)
end

robotvid = str_loc + "/robotVideo.avi";
robot_video = VideoWriter(robotvid);
robot_video.FrameRate = 25;
open(robot_video)

%% display robot at start

%% Sample skeleton while moving the robot
while(it<=window)
    % get robot config
    [ee_cart_pos, j2_cart_pos] = get_config(cur_joint_pos);

    % get fourier coefs
    [cur_curve, cur_coefs] = get_fourier_coefs(cur_joint_pos);
    
    % move robot
    sin_vel = [1.2*sind(vel(1)), -0.75*sind(vel(2))];
    cur_joint_pos = cur_joint_pos + sin_vel*(1/freq);
    % cur_joint_pos = cur_joint_pos + vel*(1/freq);
    
    % data logging
    coefs_ = [coefs_; cur_coefs];
    traj_ = [traj_; ee_cart_pos];
    vel_ = [vel_; sin_vel];
    % vel_ = [vel_; vel];

    % display robot
    figure(1)
    plot([0, j2_cart_pos(1), ee_cart_pos(1)], [0, j2_cart_pos(2), ee_cart_pos(2)], 'r', 'LineWidth', 5); % current robot state
    hold on
    plot(traj_(:,1),traj_(:,2), 'mo')
    % vel_txt = strcat("J1: ", num2str(vel(1)), " J2: ",num2str(vel(2)));
    vel_txt = strcat("J1: ", num2str(sin_vel(1)), " J2: ",num2str(sin_vel(2)));
    vis_txt = strcat("sin joint vel- ", vel_txt);    
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
    
    % rate control
    waitfor(r);

    % increment iterator
    vel = vel + [3.6, 3.6];
    it = it + 1;
end

close(robot_video)
location = strcat(str_loc,"/traj.png");
saveas(gcf,location)


%% Plot coefs
figure()
t = tiledlayout(3,2);
for i = 1:6
    nexttile
    coefname = ["a0", "a1", "b1", "a2", "b2", "w"];
    title_str = strcat("coefficient: ",coefname(i));
    plot(coefs_(:,i))
    title(title_str)
    xlabel('iterations')
    ylabel('coefficient value')
    grid on
end
location = strcat(str_loc,"/plot.png");
saveas(gcf, location)

%% Plot velocity
figure()
t = tiledlayout(2,1);
for i = 1:2
    nexttile
    title_str = strcat("Joint ", num2str(i))
    plot(vel_(:,i))
    title(title_str)
    xlabel('iterations')
    ylabel('velocity')
    grid on
end
location = strcat(str_loc, "/vel.png");
saveas(gcf, location)


close all
