function [basex, basey, diffangle] = Findbase(a,b)
    %Finds base wrt initialization position
    load('C:\Users\sayan\OneDrive - Worcester Polytechnic Institute (wpi.edu)\New Study Material\DR - Soft Vision\Matlab code\initval.mat','res', 'angle', 'pixelx', 'pixely');
    angle = 85;
    diffangle = 90 - angle;
    dist = a + b;
    basex = - dist * cosd(diffangle);
    basey = - dist * sind(diffangle);