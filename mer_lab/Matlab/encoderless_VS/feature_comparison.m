clc; clear; close all;

folder_path = "/home/jc-merlab/Pictures/Data/plotting_data/";

% Read keypoint position data
filename = folder_path + "keypoints.csv";
keypoint_pos = readmatrix(filename);

% Read keypoint image
kp_img = imread(folder_path+"keypoint_img.png");

% Read control point position data
filename = folder_path + "controlpoints.csv";
controlpoint_pos = readmatrix(filename);

% Read control point image
cp_img = imread(folder_path+"controlpoint_img.png")

% Upsample control point data
for i=1:length(controlpoint_pos)
    controlpoint_pos(i,:) = 4*controlpoint_pos(i,:);
end


% Plot respective control points and keypoints

% figure(1)
% 
% 
% subplot(3,1,1)
% plot(keypoint_pos(1:end,1), -keypoint_pos(1:end,2),LineWidth=2)
% hold on
% xlim([0,640])
% ylim([-480,0])
% hold on
% plot(controlpoint_pos(1:end,1), -controlpoint_pos(1:end,2),LineWidth=2)
% legend("key point 1 trajectory", "control point 1 trajectory")
% 
% subplot(3,1,2)
% plot(keypoint_pos(1:end,3),-keypoint_pos(1:end,4),LineWidth=2)
% hold on
% xlim([0,640])
% ylim([-480,0])
% hold on
% plot(controlpoint_pos(1:end,3),-controlpoint_pos(1:end,4),LineWidth=2)
% legend("key point 2 trajectory", "control point 2 trajectory")
% 
% subplot(3,1,3)
% plot(keypoint_pos(1:end,5),-keypoint_pos(1:end,6),LineWidth=2)
% hold on
% xlim([0,640])
% ylim([-480,0])
% hold on
% plot(controlpoint_pos(1:end,5),-controlpoint_pos(1:end,6),LineWidth=2)
% legend("key point 3 trajectory", "control point 3 trajectory")

figure(2)
subplot(3,2,1)
imshow(kp_img)
% xlim([0,640])
% ylim([-480,0])
hold on
plot(keypoint_pos(1:end,1), keypoint_pos(1:end,2),LineWidth=2,Color="m")
l=legend("key point 1 trajectory")
l.FontSize=8;

subplot(3,2,2)
imshow(cp_img)
% xlim([0,640])
% ylim([-480,0])
hold on
plot(controlpoint_pos(1:end,1), controlpoint_pos(1:end,2),LineWidth=2)
l=legend("control point 1 trajectory")
l.FontSize=8;

subplot(3,2,3)
% xlim([0,640])
% ylim([-480,0])
imshow(kp_img)
hold on
plot(keypoint_pos(1:end,3), keypoint_pos(1:end,4),LineWidth=2,Color="m")
l=legend("key point 2 trajectory")
l.FontSize=8;

subplot(3,2,4)
imshow(cp_img)
xlim([0,640])
ylim([-480,0])
hold on
plot(controlpoint_pos(1:end,3), controlpoint_pos(1:end,4),LineWidth=2)
l=legend("control point 2 trajectory")
l.FontSize=8;

subplot(3,2,5)
% xlim([0,640])
% ylim([-480,0])
imshow(kp_img)
hold on
plot(keypoint_pos(1:end,5), keypoint_pos(1:end,6), LineWidth=2,Color="m")
l=legend("key point 3 trajectory")
l.FontSize=8;

subplot(3,2,6)
imshow(cp_img)
% xlim([0,640])
% ylim([-480,0])
hold on
plot(controlpoint_pos(1:end,5), controlpoint_pos(1:end,6), LineWidth=2)
l=legend("control point 3 trajectory")
l.FontSize=8;

figure(3)
subplot(3,2,1)
% imshow(kp_img)
xlim([0,640])
ylim([-480,0])
hold on
plot(keypoint_pos(1:end,1), -keypoint_pos(1:end,2),LineWidth=2,Color="m")
l = legend("key point 1 trajectory")
l.FontSize=8

subplot(3,2,2)
xlim([0,640])
ylim([-480,0])
hold on
plot(controlpoint_pos(1:end,1), -controlpoint_pos(1:end,2),LineWidth=2)
l=legend("control point 1 trajectory")
l.FontSize=8;

subplot(3,2,3)
xlim([0,640])
ylim([-480,0])
% imshow(kp_img)
hold on
plot(keypoint_pos(1:end,3), -keypoint_pos(1:end,4),LineWidth=2,Color="m")
l=legend("key point 2 trajectory")
l.FontSize=8;

subplot(3,2,4)
xlim([0,640])
ylim([-480,0])
hold on
plot(controlpoint_pos(1:end,3), -controlpoint_pos(1:end,4),LineWidth=2)
l=legend("control point 2 trajectory")
l.FontSize=8;

subplot(3,2,5)
xlim([0,640])
ylim([-480,0])
% imshow(kp_img)
hold on
plot(keypoint_pos(1:end,5), -keypoint_pos(1:end,6), LineWidth=2,Color="m")
l=legend("key point 3 trajectory")
l.FontSize=8;

subplot(3,2,6)
xlim([0,640])
ylim([-480,0])
hold on
plot(controlpoint_pos(1:end,5), -controlpoint_pos(1:end,6), LineWidth=2)
l=legend("control point 3 trajectory")
l.FontSize=8;