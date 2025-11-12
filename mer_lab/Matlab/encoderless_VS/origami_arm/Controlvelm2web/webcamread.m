clear('cam');
cam = webcam('HD Pro Webcam C920');
cam.Resolution = '800x600';
load('initval.mat','res', 'angle', 'pixelx', 'pixely');
tagFamily = ["tag36h11"];
iter1 = 1;
while 1
   % Acquire a single image.
   I = snapshot(cam);
   name = 'C:\Users\sayan\OneDrive - Worcester Polytechnic Institute (wpi.edu)\New Study Material\DR - Soft Vision\Matlab code\Controlvelm2web';
   imgname = append(name,'\vid6\','img',int2str(iter1),'.jpg');
   imwrite(I, imgname);
   c = clock;
   [id,loc,detectedFamily] = readAprilTag(I,tagFamily);
   idx = find(id==0);
   if length(idx)==1
       pos = (loc(1,:,idx)+loc(2,:,idx)+ loc(3,:,idx)+ loc(4,:,idx))/4;
       pixcury = 800 - pos(1) - pixely;
       pixcurx = pos(2) - pixelx;
       posx = pixcurx * res;
       posy = pixcury * res;
       A = [pixcurx,pixcury];
       disp(A);
       save('curval.mat', 'res', 'posx', 'posy', 'pixcurx', 'pixcury');
       %     [basex, basey, diffangle] = Findbase(88, 37);
       basex = -125; basey = 0; diffangle = 90 - angle;
       [obx, oby] = Transform(basex, basey, diffangle);
       Tendonlen2(obx, oby);
   else
       continue;
   end
   
   idx = find(id==1);
   if length(idx)==1
       pos = (loc(1,:,idx)+loc(2,:,idx)+ loc(3,:,idx)+ loc(4,:,idx))/4;
       pixcury = 800 - pos(1) - pixely;
       pixcurx = pos(2) - pixelx;
       posx = pixcurx * res;
       posy = pixcury * res;
       save('target.mat', 'res', 'posx', 'posy', 'pixcurx', 'pixcury');
   else
       continue;
   end
   %imshow(I);
   iter1 = iter1 + 1;
end

clear('cam');