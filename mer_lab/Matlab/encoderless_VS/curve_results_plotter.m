%%File Paths
clear
% file_path = "~/Pictures/adaptive_baseline_gazebo/initial estimation/trial";
file_path = "~/Pictures/curve_estimation_&_adaptive_vs_gazebo/debugging/";
trial_no = 1;

str_J = file_path+trial_no+"/modelerror.csv";
str_jvel1 = file_path+trial_no+"/j1vel.csv";
str_jvel2 = file_path+trial_no+"/j2vel.csv";
str_err = file_path+trial_no+"/err.csv";

%% Read data
J = readtable(str_J,'NumHeaderLines',1);
J_vec = table2array(J);

jvel1 = readtable(str_jvel1,'NumHeaderLines',1);
jvel1_vec = table2array(jvel1);

jvel2 = readtable(str_jvel2,'NumHeaderLines',1);
jvel2_vec = table2array(jvel2);

err = readtable(str_err,'NumHeaderLines',1);
err_vec = table2array(err);
