%% Robot parameters
start_joint_pos = [deg2rad(65), deg2rad(-120)];
goal_joint_pos = [deg2rad(100), deg2rad(-38)];

%% Get config
[ee_start_cart_pos, j2_start_cart_pos] = get_config(start_joint_pos);
[ee_goal_cart_pos, j2_goal_cart_pos] = get_config(goal_joint_pos);

%% Get skeleton representation
[init_curve, init_coefs] = get_fourier_coefs(start_joint_pos);
[goal_curve, goal_coefs] = get_fourier_coefs(goal_joint_pos);

%% Plot config
figure(1)
plot([0, j2_start_cart_pos(1), ee_start_cart_pos(1)], [0, j2_start_cart_pos(2), ee_start_cart_pos(2)], 'r', 'LineWidth', 5); % robot config at start state
hold on
axis([-165 165 -165 165])
plot([0, j2_goal_cart_pos(1), ee_goal_cart_pos(1)], [0, j2_goal_cart_pos(2), ee_goal_cart_pos(2)], 'g', 'LineWidth', 5); % robot config at goal state
plot(init_curve, 'b-.')
plot(goal_curve, 'b-.')
legend({'initial config', 'goal config','initial skel', 'goal skel'},'Location','northeastoutside')
pbaspect([1 1 1])
grid on

%% Plot skeleton representation

