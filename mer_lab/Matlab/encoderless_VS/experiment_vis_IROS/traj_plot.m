clear
clc
% Read image
im = imread("franka_8_init_light.png");
imshow(im)
hold on;
target = [243,107];
% Read and pre-process baseline traj
baseline_traj = readmatrix("ee_pos.csv");
baseline_traj = baseline_traj(40:end,1:2);

% Read and pre-process shape traj
shape_traj = readmatrix("cp.csv");
shape_traj = shape_traj(50:end,5:6)
start = [(shape_traj(1,1)+baseline_traj(1,1))/2,(shape_traj(1,2)+baseline_traj(1,2))/2]
% Plot trjectory on image
plot(shape_traj(:,1),shape_traj(:,2),'y-','MarkerSize',1,'LineWidth',3);
plot(target(1), target(2),'ro', 'MarkerSize',25, 'LineWidth', 5)
plot(start(1), start(2), 'go', 'MarkerSize',25,'LineWidth',5)
plot(baseline_traj(:,1),baseline_traj(:,2),'b-.', 'MarkerSize', 1, 'LineWidth',3);

exportgraphics(gcf,'trajectory8.png','Resolution',600)