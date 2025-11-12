
folder_path = "~/.ros/";

% Read error data
file_name = folder_path + "error.csv";
error_data = readmatrix(file_name);

% Remove time stamps & data offset
error_data = error_data(:,3:end);

% Compute error norm
error_norm = [];
for row = 1:length(error_data)
    sum = 0;
    for col = 1:length(error_data(row,:))
        sum = sum + error_data(row,col)^2;
    end
    error_norm = [error_norm, sqrt(sum)];
end

% Plot feature errors and error norm
num_features = size(error_data,2);

figure(1)
for i=1:num_features
    subplot(ceil(num_features/2), 2, i)
    plot(error_data(:,i),'LineWidth',2.0)
    title('Feature error')
    ylabel('error')
    xlabel('t')
end

% Save plot to file
save_path = folder_path + "feature_error";
saveas(gcf,save_path, 'png')

figure(2)
plot(error_norm,'LineWidth',2.0)
title('Error norm')
ylabel('error')
xlabel('t')

% Save plot to file
save_path = folder_path + "error_norm";
saveas(gcf,save_path, 'png')


% Read velocity data
file_name = folder_path + "velocity.csv";
velocity_data = readmatrix(file_name);

% Remove time stamps & data offset
velocity_data = velocity_data(:,3:end);

% Plot velocity data
num_actuators = size(velocity_data,2);

figure(3)
for i=1:num_actuators
    subplot(ceil(num_actuators/2),2,i)
    plot(velocity_data(:,i),'LineWidth',2.0)
    title('Actuator Velocities')
    ylabel('velocity')
    xlabel('t')
end

% Save plot to file
save_path = folder_path + "velocity";
saveas(gcf,save_path, 'png')

% Plot velocity data without initialization trajectory
% Look for row with all 0s
start_pt = 1;
for i = 1:size(velocity_data,1)
    vel_vector = velocity_data(i,:);
    if norm(vel_vector) == 0
        start_pt = i;
        break
    end
end

figure(4)
for i=1:num_actuators
    subplot(ceil(num_actuators/2),2,i)
    plot(velocity_data(start_pt:end,i),'LineWidth',2.0)
    title('Actuator Velocities')
    ylabel('velocity')
    xlabel('t')
end

% Save plot to file
save_path = folder_path + "control_velocity";
saveas(gcf,save_path, 'png')

% Read model error data
file_name = folder_path + "model_error.csv";
model_error_data = readmatrix(file_name);

% Remove time stamps
% clean_model_error_data = model_error_data(start_pt:end, 2:end);

% Plot model error data
figure(5)
plot(model_error_data(start_pt:end,2:end),'LineWidth',2.0)
title('Model error')
ylabel('model error')
xlabel('t')

% Save plot to file
save_path = folder_path + "model_error";
saveas(gcf,save_path, 'png')