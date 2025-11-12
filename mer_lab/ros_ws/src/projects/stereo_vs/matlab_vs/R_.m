function camera_frame_rotation = R_(c_i)
%R returns the 3x3 rotation of the pose of the camera, c_i
Rx = [1 0 0;
      0 cos(c_i(6)) -sin(c_i(6));
      0 sin(c_i(6)) cos(c_i(6));];
Ry = [cos(c_i(5))  0 sin(c_i(5));
      0             1            0;
      -sin(c_i(5)) 0 cos(c_i(5));];
  
Rz = [cos(c_i(4)) -sin(c_i(4)) 0;
      sin(c_i(4)) cos(c_i(4))  0;
      0        0         1;];

camera_frame_rotation = Rz*Ry*Rx;
end

