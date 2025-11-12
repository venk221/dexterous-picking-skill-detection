
%Be careful with axes since coordinate systems used changes occasionally
% Using modified Jacobian
% Using rate control

%Target
tarx = 40;
tary = -30;

% Just to override some camera errors
clear cam
clear ES
cameracom
pause(1.5);
clear cam
cameracom


%Continouous Targets
load('curval.mat','res', 'anglecur', 'posx', 'posy', 'pixcurx', 'pixcury');
%Not Swapping axes
xdiff = (tarx - pixcurx)*res;
ydiff = (tary - pixcury)*res;
plen = sqrt(xdiff^2 + ydiff^2); %Total path length
seglen = 2; %Length between successive points in generated path
pseg = int8(plen / seglen);
numseg = 1:pseg;
%Series of target points
tarsx = pixcurx + double(numseg) * (xdiff/double(pseg)/res);
tarsy = pixcury + double(numseg) * (ydiff/double(pseg)/res);

e11 = 0;
e21 = 0;
sign_e = [0;0];
rate = 25;
r = rateControl(rate);
reset(r)
ti = 1;
i = 1;
while 1
    load('curval.mat','res', 'anglecur', 'posx', 'posy', 'pixcurx', 'pixcury');
    load('tlen2.mat','clen1', 'clen2');
    
    if ti > pseg
        ti = pseg;
    end
    %Swapping axes
    ydiff = (tarsx(ti) - pixcurx)*res;
    xdiff = (tarsy(ti) - pixcury)*res;
    
    %Storing data
    A = [tarx, tary, pixcurx, pixcury, r.TotalElapsedTime];
    ES{i} = A;
    
    if sqrt(xdiff^2 + ydiff^2)< 4 && ti == pseg
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
    
    %Calling Jacobian. Some reordering of parameters done because ordering
    %of parameters in Jacobian function different than in rest of code
    [u1, u2, u3] = Jac_m2cont(len2, len3, len1, s1, s2);
    v1 = u3; v2 = u1; v3 = u2;
    
    bo = int8([v1, v2, v3] < 0);
    
    %Velocity Input
    %Inputs to each actuator are in sets of two - 1. zero or non zero
    %indicating direction of velocity, 2. number between 0 and 255
    %indicating magnitude of velocity
    write(module5,[bo(1), abs(int8(v1)), bo(2), abs(int8(v2)), bo(3), abs(int8(v3)), 2]);
    
    e11 = e12;
    e21 = e22;
    i = i + 1;
    ti = ti + 1;
    waitfor(r);
end
write(module5,[0,0,0,0,0,0,2]);
clear cam
save('Iter6.mat', 'ES');
clear ES