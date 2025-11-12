

pixelx = 15;
pixely = -15;
cameracom
b = 30;
a = 30;

i = 0;
while True
    load('curval.mat','res', 'anglecur', 'posx', 'posy', 'pixcurx', 'pixcury');
    load('tlen.mat','clen1', 'clen2');
    
    %Swapping axes
    ydiff = (pixelx - pixcurx)*res;
    xdiff = (pixely - pixcury)*res;

    c = clock;
    A = [pixelx, pixely, pixcurx, pixcury, c(5), c(6)];
    ES{i} = A;
    
    if abs(ydiff)<5 && abs(xdiff)<5
        break;
    end
    
    a = clen1; b = clen2;
    len1 = a; len2 = b; len3 = a;
    
    [u1, u2, u3] = Jac(len2, len3, len1, ydiff, -xdiff);
    v1 = u3; v2 = u1; v3 = u2;
    
    bo = int8([v1, v2, v3] > 0);
    
    write(module5,[bo(1), abs(int8(v1)), bo(2), abs(int8(v2)), bo(3), abs(int8(v3)), 2]);
    
    i = i + 1;
end

clear cam
save('Iter1.mat', 'ES');
clear ES