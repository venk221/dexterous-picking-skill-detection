

pixelx = 15;
pixely = -15;

b = 30;
a = 30;
e = 30;
for i = 1:10
    load('curval.mat','res', 'anglecur', 'posx', 'posy', 'pixcurx', 'pixcury');
    %Swapping axes
    ydiff = (pixelx - pixcurx)*res;
    xdiff = (pixely - pixcury)*res;

    c = clock;
    A = [pixelx, pixely, pixcurx, pixcury, c(5), c(6)];
    ES{i} = A;
    if abs(ydiff)<2 && abs(xdiff)<2
        break;
    end

    len1 = a; len2 = b; len3 = e-2;

    [u1, u2, u3] = Jac(len2, len3, len1, ydiff, -xdiff);
    v1 = u3; v2 = u1; v3 = u2;

    a = a + v1;
    b = b + v2;
    e = e+ v3;

    G = [a b e-2];
    cable_length_traj = (G + L0)/1000;

    [m1_command, m2_command, m3_command] = ...
      cableLength2MotorCommand(cable_length_traj(3*(1-1)+1), cable_length_traj(3*(1-1)+2), cable_length_traj(3*(1-1)+3), max_length, gear_ratio(1));

    m_command(1,:) = [m1_command, m2_command, m3_command];
    M(:,:,1) = motorCommandConvert(m_command(1,:));


    write(module5,[M(1,:,1) M(2,:,1) M(3,:,1) 1]);

    pause(1.5);       
end   

save('Iter6.mat', 'ES');