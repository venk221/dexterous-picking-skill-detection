function [htm_lcam_ee, htm_rcam_ee] = getPandaHTM(q, tftree) 

%% Camera Transforms to base frame of arm

% get transform from left camera optical frame to left camera frame
tf_lcop_lcam = getTransform(tftree,'left_camera_optical_frame','left_camera_frame');
T_lcop_lcam = HTM_from_tf(tf_lcop_lcam);

% get transform from right camera optical frame to right camera frame
tf_rcop_rcam = getTransform(tftree,'right_camera_optical_frame','right_camera_frame');
T_rcop_rcam = HTM_from_tf(tf_rcop_rcam);

% get transform from left camera frame to Panda base frame
tf_lcam_l0 = getTransform(tftree, 'left_camera_frame', 'panda_link0');
T_lcam_0 = HTM_from_tf(tf_lcam_l0);

% get transform from left camera frame to Panda link 0 (base frame)
tf_rcam_l0 = getTransform(tftree, 'right_camera_frame', 'panda_link0');
T_rcam_0 = HTM_from_tf(tf_rcam_l0);

%% Arm joint transforms

% get transform from Panda link 0 to link 1 Rz(q1)
tf_l0_l1 = getTransform(tftree, 'panda_link0', 'panda_link1');
T_l0_l1 = HTM_from_tf(tf_l0_l1);
R1 = R_HTM(q(1),0);
T_0_1 = [R1 T_l0_l1(1:3,4); 0 0 0 1];

% get transform from Panda link 1 to link 2 
tf_l1_l2 = getTransform(tftree, 'panda_link1', 'panda_link2');
T_l1_l2 = HTM_from_tf(tf_l1_l2);
R2 = R_HTM(-q(2), pi/2)';
T_1_2 = [R2 T_l1_l2(1:3,4); 0 0 0 1];

% get transform from Panda link 2 to link 3
tf_l2_l3 = getTransform(tftree, 'panda_link2', 'panda_link3');
T_l2_l3 = HTM_from_tf(tf_l2_l3);
R3 = R_HTM(-q(3), -pi/2)';
T_2_3 = [R3 T_l2_l3(1:3, 4); 0 0 0 1];

% get transform from Panda link 3 to link 4
tf_l3_l4 = getTransform(tftree, 'panda_link3', 'panda_link4');
T_l3_l4 = HTM_from_tf(tf_l3_l4);
R4 = R_HTM(-q(4), -pi/2)';
T_3_4 = [R4 T_l3_l4(1:3, 4); 0 0 0 1];

% get transform from Panda link 4 to link 5
tf_l4_l5 = getTransform(tftree, 'panda_link4', 'panda_link5');
T_l4_l5 = HTM_from_tf(tf_l4_l5);
R5 = R_HTM(-q(5), pi/2)';
T_4_5 = [R5 T_l4_l5(1:3, 4); 0 0 0 1];

% get transform from Panda link 5 to link 6
tf_l5_l6 = getTransform(tftree, 'panda_link5', 'panda_link6');
T_l5_l6 = HTM_from_tf(tf_l5_l6);
R6 = R_HTM(-q(6), -pi/2)';
T_5_6 = [R6 T_l5_l6(1:3, 4); 0 0 0 1];

% get transform from Panda link 6 to link 7
tf_l6_l7 = getTransform(tftree, 'panda_link6', 'panda_link7');
T_l6_l7 = HTM_from_tf(tf_l6_l7);
R7 = inv(R_HTM(-q(7), -pi/2));
T_6_7 = [R7 T_l6_l7(1:3, 4); 0 0 0 1];

% get transform from Panda link 7 to link 8
tf_l7_l8 = getTransform(tftree, 'panda_link7', 'panda_link8');
T_7_8 = HTM_from_tf(tf_l7_l8);

% get transform from Panda link 8 to panda hand
tf_l8_h = getTransform(tftree, 'panda_link8', 'panda_hand');
T_8_h = HTM_from_tf(tf_l8_h);

% get transform from Panda hand to left finger
tf_h_lf = getTransform(tftree, 'panda_hand', 'panda_leftfinger');
T_h_lf = HTM_from_tf(tf_h_lf);

% get transform from Panda hand to right finger
tf_h_rf = getTransform(tftree, 'panda_hand', 'panda_rightfinger');
T_h_rf = HTM_from_tf(tf_h_rf);

% get midpoint between left and right finger: this is end effector frame
mid_ = (T_h_rf(1:3,4)+T_h_lf(1:3,4))/2;
T_h_ee = [T_h_rf(1:3,1:3) mid_; 0 0 0 1];

% get symbolic FK of end effector in left camera frame
htm_lcam_ee = T_lcop_lcam*T_lcam_0*T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*T_8_h*T_h_ee;
% htm_lcam_ee = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*T_8_h*T_h_ee;

% get symbolic FK of end effector in right camera frame
htm_rcam_ee = T_rcop_rcam*T_rcam_0*T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*T_8_h*T_h_ee;
% htm_rcam_ee = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*T_8_h*T_h_ee;

% @TODO: can add the links for the fingertips for more accurate modeling
