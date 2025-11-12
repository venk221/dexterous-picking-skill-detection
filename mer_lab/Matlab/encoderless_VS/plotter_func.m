%%File Paths

% file_path = "~/Pictures/adaptive_baseline_gazebo/initial estimation/trial";
% file_path = "~/Pictures/adaptive_baseline_gazebo/servoing/exps/";
% file_path = "~/Pictures/curve_estimation_adaptive_vs_gazebo/debugging/"
function[] = plotter_func(trial_no)
file_path = "~/Pictures/curve_estimation_adaptive_vs_gazebo/servoing/exps/";
% trial_no = 4;

str_J = file_path+trial_no+"/modelerror.csv";
str_jvel1 = file_path+trial_no+"/j1vel.csv";
str_jvel2 = file_path+trial_no+"/j2vel.csv";
str_jvel3 = file_path+trial_no+"/j3vel.csv";
str_err = file_path+trial_no+"/err.csv";

%% Read data
J = readtable(str_J,'NumHeaderLines',1);
J_vec = table2array(J);

jvel1 = readtable(str_jvel1,'NumHeaderLines',1);
jvel1_vec = table2array(jvel1);

jvel2 = readtable(str_jvel2,'NumHeaderLines',1);
jvel2_vec = table2array(jvel2);

jvel3 = readtable(str_jvel3,'NumHeaderLines',1);
jvel3_vec = table2array(jvel3);

err = readtable(str_err,'NumHeaderLines',1);
err_vec = table2array(err);

%% Plot
figure(1)
hold on
subplot(4,1,1)
plot(J_vec,'LineWidth',2)
title('Model Error')
xlabel('Iteration#')
ylabel('J')

subplot(4,1,2)
plot(jvel1_vec,'LineWidth',2)
title("J1 input velocity")
xlabel('Iteration#')
ylabel('Velocity')

subplot(4,1,3)
plot(jvel2_vec,'LineWidth',2)
title("J2 input velocity")
xlabel('Iteration#')
ylabel('Velocity')

subplot(4,1,4)
plot(jvel3_vec,'LineWidth',2)
title("J3 input velocity")
xlabel('Iteration#')
ylabel('Velocity')

plot_loc = file_path + trial_no + "/plot.png";
saveas(gcf,plot_loc)



%% Error Norm compute
for i = 1:length(err_vec)
    err_norm(i) = norm(err_vec(i,:));
end
% 
% figure(2) %Error Plot
% hold on
% subplot(3,1,1)
% plot(err_vec(:,1),'LineWidth',2)
% title('Error X in px')
% xlabel('Iteration#')
% ylabel('Error')
% 
% subplot(3,1,2)
% plot(err_vec(:,2),'LineWidth',2)
% title('Error y in px')
% xlabel('Iteration#')
% ylabel('Error')
% 
% subplot(3,1,3)
% plot(err_norm,'LineWidth',2)
% title('Error norm')
% xlabel('Iteration#')
% ylabel('Error norm')
% err_plot_loc = file_path + trial_no + "/err_plot.png";

figure(2) % Error Plot
hold on
subplot(2,1,1)
grid on
hold on
plot(err_vec(:,1), 'Linewidth',2)
plot(err_vec(:,2), 'Linewidth',2)
plot(err_vec(:,3), 'Linewidth',2)
plot(err_vec(:,4), 'Linewidth',2)
plot(err_vec(:,5), 'Linewidth',2)
plot(err_vec(:,6), 'Linewidth',2)
plot(err_vec(:,7), 'LineWidth',2)
plot(err_vec(:,8), 'LineWidth',2)
legend('cp1 x', 'cp1 y', 'cp2 x', 'cp2 y', 'cp3 x', 'cp3 y','cp4x','cp4y','Location', 'NorthEastOutside')
title('Feature error')
xlabel('Iteration #')
ylabel('Error in px')

subplot(2,1,2)
plot(err_norm, 'LineWidth',2)
title('Error norm')
xlabel('Iteration#')
ylabel('Error norm')

err_plot_loc = file_path + trial_no + "/err_plot.png";
saveas(gcf,err_plot_loc)


%% Plot without initialization data
figure(3)
hold on
subplot(4,1,1)
grid on
plot(J_vec(121:end),'LineWidth',2)
title('Model Error')
xlabel('Iteration#')
ylabel('J')

subplot(4,1,2)
grid on
plot(jvel1_vec(121:end),'LineWidth',2)
title("J1 input velocity")
xlabel('Iteration#')
ylabel('Velocity')

subplot(4,1,3)
grid on
plot(jvel2_vec(121:end),'LineWidth',2)
title("J2 input velocity")
xlabel('Iteration#')
ylabel('Velocity')

subplot(4,1,4)
grid on
plot(jvel3_vec(121:end),'LineWidth',2)
title("J3 input velocity")
xlabel('Iteration#')
ylabel('Velocity')

plot_loc = file_path + trial_no + "/plot_b.png";
saveas(gcf,plot_loc)

% figure(4) % Error Plot
% hold on
% subplot(2,1,1)
% grid on
% hold on
% plot(err_vec(:,1), 'Linewidth',2)
% plot(err_vec(:,2), 'Linewidth',2)
% plot(err_vec(:,3), 'Linewidth',2)
% plot(err_vec(:,4), 'Linewidth',2)
% legend('cp1 x', 'cp1 y', 'cp2 x', 'cp2 y', 'Location', 'NorthEastOutside')
% title('Feature error')
% xlabel('Iteration #')
% ylabel('Pixel error')
% 
% subplot(2,1,2)
% plot(err_norm, 'LineWidth',2)
% title('Error norm')
% xlabel('Iteration#')
% ylabel('Error norm')
% 
% err_plot_loc = file_path + trial_no + "/err_plot_b.png";
% saveas(gcf,err_plot_loc)

end