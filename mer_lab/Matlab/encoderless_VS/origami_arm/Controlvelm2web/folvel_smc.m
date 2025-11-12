
% Using modified Jacobian
% Using rate control

tarx = 120;
tary = -70;

clear ES
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

e11 = 0;
e21 = 0;
sign_e = [0;0];
rate = 25;
r = rateControl(rate);
reset(r)
ti = 1;
store = 1;
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
    ES{store} = A;
    
    if sqrt(xdiff^2 + ydiff^2)< 2 && ti == pseg
        break;
    end
    
    a = clen1; b = clen2;
    len1 = a+1; len2 = b; len3 = a;
    e12 = ydiff;
    e22 = -xdiff;
    %SMC
    k1 = 0.2; k2 = 0.1; lambda = 5;
    e = [e12-e11;e22-e21]*rate + lambda * [e12;e22];
    sign_e = 0.8 * sign_e + 0.2 * sign(e);
    s = [k1,0;0,k1]*e / sqrt(norm(e)) + [k2,0;0,k2]*sign_e;
    s1 = s(1); s2 = s(2);
    
    [u1, u2, u3] = Jac_m2smc(len2, len3, len1, s1, s2);
    v1 = u3; v2 = u1; v3 = u2;
    
    bo = int8([v1, v2, v3] < 0);
    
    write(module5,[bo(1), abs(int8(v1)), bo(2), abs(int8(v2)), bo(3), abs(int8(v3)), 2]);
    
    e11 = e12;
    e21 = e22;
    store = store + 1;
    ti = ti + 1;
    waitfor(r);
end
write(module5,[0,0,0,0,0,0,2]);
save('Iter4.mat', 'ES');
clear ES