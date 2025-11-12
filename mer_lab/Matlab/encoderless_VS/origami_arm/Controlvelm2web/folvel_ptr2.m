
% Using modified Jacobian
% Using rate control

tarx = 30;
tary = -60;

%Targets
for i = 1:10
    try
        load('curval.mat', 'res', 'pixcurx', 'pixcury');
        break;
    catch
        gbg = 1;
    end
end
%Not Swapping axes
xdiff = (tarx - pixcurx)*res;
ydiff = (tary - pixcury)*res;
plen = sqrt(xdiff^2 + ydiff^2);
seglen = 2;
pseg = int8(plen / seglen);
numseg = 1:pseg;
tarsx = pixcurx + double(numseg) * (xdiff/double(pseg)/res);
tarsy = pixcury + double(numseg) * (ydiff/double(pseg)/res);

r = rateControl(10);
reset(r)
ti = 1;
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
    for i = 1:10
        try
            load('tlen2.mat','clen1', 'clen2');
            break;
        catch
            gbg = 1;
        end
    end
    
    
    if ti > pseg
        ti = pseg;
    end
    %Swapping axes
    ydiff = (tarsx(ti) - pixcurx)*res;
    xdiff = (tarsy(ti) - pixcury)*res;

    A = [tarx, tary, pixcurx, pixcury, r.TotalElapsedTime];
    ES{i} = A;
    
    if sqrt(xdiff^2 + ydiff^2)< 2 && ti == pseg
        break;
    end
    
    a = clen1; b = clen2;
    len1 = a+1; len2 = b; len3 = a;
    
    [u1, u2, u3] = Jac_m2cont(len2, len3, len1, ydiff, -xdiff);
    v1 = u3; v2 = u1; v3 = u2;
    
    bo = int8([v1, v2, v3] < 0);
    
    write(module5,[bo(1), abs(int8(v1)), bo(2), abs(int8(v2)), bo(3), abs(int8(v3)), 2]);
    
    i = i + 1;
    ti = ti + 1;
    waitfor(r);
end
write(module5,[0,0,0,0,0,0,2]);
clear cam
save('Iter5.mat', 'ES');
clear ES