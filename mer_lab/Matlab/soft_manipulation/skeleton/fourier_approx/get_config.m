function [ee_pos, j2_pos] = get_config(cur_joint_pos)
%% Robot model
l1 = 90; l2 = 75; % link lengths

th1 = cur_joint_pos(1);
th2 = cur_joint_pos(2);

eeX = l1*cos(th1) + l2*cos(th1 + th2);
eeY = l1*sin(th1) + l2*sin(th1 + th2);

j2X = l1*cos(th1);
j2Y = l1*sin(th1);

ee_pos = [eeX, eeY]; % end-effector position
j2_pos = [j2X, j2Y]; % joint2 position

end

