clear
clc
% Read image
im = imread("init_pose4");
imshow(im)
hold on;

target_x = [309, 178, 398, 281, 363, 541, 328, 243, 467, 248];
target_y = [126, 121, 116, 189, 338, 289, 207, 107, 319, 146];

plot(target_x, target_y,'g+', 'MarkerSize',15, 'LineWidth', 3)


exportgraphics(gcf,'targets4.png','Resolution',600)