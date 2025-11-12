function [] = Initialize(r, xdist, ydist)
    if length(r) == 9
        r = str2double(r);
        cxdiff = abs(r(3) - r(7));
        cydiff = abs(r(4) - r(8));
        %Resolution
        %res = (xdist/cxdiff + ydist/cydiff) / 2;
        res = 1.78;
        %Tag 0 on robot
        angle = 90;
        pixelx = r(3);
        pixely = r(4);
        disp("Initialize Success");
        save('initval.mat', 'res', 'angle', 'pixelx', 'pixely');
    else
        disp("Initialize Fail");
    end
