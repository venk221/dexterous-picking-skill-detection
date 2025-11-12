clear
clc
% Read image
path = "~/Desktop/IROS-2022-media/robustness_imgs/shape/3/";
path_to_img = path+"3a_target.png";
im = imread(path_to_img);
imshow(im)
hold on;

target = [450,304];
% Read and pre-process benchmark trajectory
file_path = path+"3a_traj"
benchmark_trajectory = readmatrix(file_path);
benchmark_traj = benchmark_trajectory(50:end,5:6);

% Read and pre-process trajectory
file_path = path+"3b_traj"
trajectory = readmatrix(file_path);
traj = trajectory(50:end,5:6);

% Plot trjectory on image
plot(traj(:,1),traj(:,2),'g-','MarkerSize',1,'LineWidth',3);
plot(benchmark_traj(:,1),benchmark_traj(:,2),'r--', 'MarkerSize', 1, 'LineWidth',3);
plot(target(1), target(2),'m+', 'MarkerSize',20, 'LineWidth', 5)

% legend('target position', '50 % stiffness', '100% stiffness')
save_path = path+ "trajectory.png"
% exportgraphics(gcf,save_path,'Resolution',600)
saveas(gcf,save_path)