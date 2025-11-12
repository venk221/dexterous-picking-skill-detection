function [htm_lcam_ee, htm_rcam_ee] = getPandaHTM_DH(q, tftree)

%% Camera Transforms to base frame of arm

% get transform from left camera optical frame to left camera frame
% tf_lcop_lcam = getTransform(tftree,'left_camera_optical_frame','left_camera_frame');
% T_lcop_lcam = HTM_from_tf(tf_lcop_lcam);
% 
% % get transform from right camera optical frame to right camera frame
% tf_rcop_rcam = getTransform(tftree,'right_camera_optical_frame','right_camera_frame');
% T_rcop_rcam = HTM_from_tf(tf_rcop_rcam);
% 
% % get transform from left camera frame to Panda base frame
% tf_lcam_l0 = getTransform(tftree, 'left_camera_frame', 'panda_link0');
% T_lcam_0 = HTM_from_tf(tf_lcam_l0);
% 
% % get transform from left camera frame to Panda link 0 (base frame)
% tf_rcam_l0 = getTransform(tftree, 'right_camera_frame', 'panda_link0');
% T_rcam_0 = HTM_from_tf(tf_rcam_l0);

% DH params:
%#          a(m) d(m)           alpha(rad)      theta(rad)
dh_params = [0,      0.333,             0,   q(1); 
             0,          0,  -(pi/2),   q(2);
             0,      0.316,   (pi/2),   q(3);
             0.0825,     0,   (pi/2),   q(4);
             -0.0825, 0.384,  -(pi/2),   q(5);
             0,          0,   (pi/2),   q(6);
             0.088,      0,   (pi/2),   q(7)+(pi/4);
             0.1,      0.1070,   0,   0];
% obtain arm links directly from DH parameters instead
T_0_1 = aTM(dh_params(1,:));
T_1_2 = aTM(dh_params(2,:));
T_2_3 = aTM(dh_params(3,:));
T_3_4 = aTM(dh_params(4,:));
T_4_5 = aTM(dh_params(5,:));
T_5_6 = aTM(dh_params(6,:));
T_6_7 = aTM(dh_params(7,:));
T_7_8 = aTM(dh_params(8,:));

% get symbolic FK of end effector in left camera frame
% htm_lcam_ee = T_lcop_lcam*T_lcam_0*T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*T_8_h*T_h_ee;
htm_lcam_ee = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8; %*T_8_h*T_h_ee;

% get symbolic FK of end effector in right camera frame
% htm_rcam_ee = T_rcop_rcam*T_rcam_0*T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*T_8_h*T_h_ee;
htm_rcam_ee = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8; %*T_8_h*T_h_ee;

% @TODO: can add the links for the fingertips for more accurate modeling

