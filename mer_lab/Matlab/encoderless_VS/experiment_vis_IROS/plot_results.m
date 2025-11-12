clc
clear
close all
% _ is for baseline

% Read Error
error = readmatrix("err.csv");
error_ = readmatrix("err_.csv");

% Read model error
J = readmatrix("modelerror.csv");
J_ = readmatrix("modelerror_.csv");

% Read velocity
v1 = readmatrix("j1vel.csv");
v1_ = readmatrix("j1vel_.csv");

v2 = readmatrix("j2vel.csv");
v2_ = readmatrix("j2vel_.csv");

error_norm = [];
% compute norm
for i=1:length(error)
   err = norm(error(i,:));
   error_norm = [error_norm, err];
end
error_norm_ = [];
% compute norm
for i=1:length(error_)
   err_ = norm(error_(i,:));
   error_norm_ = [error_norm_, err_];
end

% Plot error
figure()
subplot(2,2,1)
hold on
plot(error(:,5),'LineWidth',2)
plot(error(:,6),'LineWidth',2)
grid on
xlabel("Iteration")
ylabel("End effector error (px)")
legend("x-error", "y-error")

subplot(2,2,2)
title("Baseline")
plot(error_norm, 'LineWidth', 2)
xlabel("Iteration")
ylabel("shape feature error norm")
grid on

subplot(2,2,3)
grid on
hold on
plot(error_(:,1),'LineWidth',2)
plot(error_(:,2),'LineWidth',2)
xlabel("Iteration")
ylabel("End-effector error (px)")
legend("x-error", "y-error")

subplot(2,2,4)
grid on
plot(error_norm_, 'LineWidth',2)
xlabel("Iteration")
ylabel("error norm")


exportgraphics(gcf,'error.png','Resolution',600)

% Plot model error
figure()
subplot(2,1,1)
hold on
plot(J, 'LineWidth', 2)
grid on
xlabel("Iteration")
ylabel("Model error")

subplot(2,1,2)
plot(J_,'LineWidth',2)
grid on
xlabel("Iteration")
ylabel("Model error")


exportgraphics(gcf,'model_error.png','Resolution',600)

% Plot Velocity
figure()
hold on
subplot(2,1,1)
plot(v1(51:end),'LineWidth',2)
grid on
xlabel("Iteration")
ylabel("Joint 1 velocity (rad/s)")

subplot(2,1,2)
plot(v2(51:end), 'LineWidth',2)
grid on
xlabel("Iteration")
ylabel("Joint 2 velocity (rad/s)")

exportgraphics(gcf,'vel_plot.png','Resolution',600)