% Script to plot numbers from experiments
% Drag and drop CSV to import columnwise the data
t = linspace(0,16,163)
% Point feature error
figure(1)
hold on
plot(t,x0(23:end),'LineWidth', 2.0)
plot(t,y0(23:end),'LineWidth', 2.0)
plot(t,x1(23:end),'LineWidth', 2.0)
plot(t,y1(23:end),'LineWidth', 2.0)
plot(t,x2(23:end),'LineWidth', 2.0)
plot(t,y2(23:end),'LineWidth', 2.0)
legend("x0","y0","x1","y1","x2","y2")

% kappa error
figure(2)
hold on
plot(t,k0(23:end),'LineWidth', 2.0)
plot(t,k1(23:end),'LineWidth', 2.0)
plot(t,k2(23:end),'LineWidth', 2.0)
legend("k0","k1","k2")

% Velocities
figure(3)
hold on
subplot(2,2,1)
plot(t,v1(23:end),'LineWidth',2.0)
subplot(2,2,2)
plot(t,v2(23:end),'LineWidth',2.0)
subplot(2,2,3)
plot(t,v3(23:end),'LineWidth',2.0)
subplot(2,2,4)
plot(t,v4(23:end),'LineWidth',2.0)
t = linspace(0,16,162)
% Model error
figure(4)
hold on
plot(t,modelerror(23:end),'LineWidth',2.0)