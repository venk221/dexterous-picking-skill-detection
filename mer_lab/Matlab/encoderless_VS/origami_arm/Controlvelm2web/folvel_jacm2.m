
% Using modified Jacobian
% Using rate control

pixelx = 15;
pixely = -30;

clear cam;
cameracom
pause(1.5);
clear cam
cameracom


r = rateControl(25);
i = 1;
while 1
    for i = 1:10
        try
            load('curval.mat', 'res', 'pixcurx', 'pixcury');
            break;
        catch
            gbg = 1;
        end
    end
    load('tlen2.mat','clen1', 'clen2');
    
    %Swapping axes
    ydiff = (pixelx - pixcurx)*res;
    xdiff = (pixely - pixcury)*res;

    c = clock;
    A = [pixelx, pixely, pixcurx, pixcury, c(5), c(6)];
    ES{i} = A;
    
    if abs(ydiff)<2 && abs(xdiff)<2
        break;
    end
    
    a = clen1; b = clen2;
    len1 = a+1; len2 = b; len3 = a;
    
    [u1, u2, u3] = Jac_m2(len2, len3, len1, ydiff, -xdiff);
    v1 = u3; v2 = u1; v3 = u2;
    
    bo = int8([v1, v2, v3] < 0);
    
    write(module5,[bo(1), abs(int8(v1)), bo(2), abs(int8(v2)), bo(3), abs(int8(v3)), 2]);
    
    i = i + 1;
    waitfor(r);
end
write(module5,[0,0,0,0,0,0,2]);
clear cam
save('Iter4.mat', 'ES');
clear ES