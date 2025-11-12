
name = 'C:\Users\sayan\OneDrive - Worcester Polytechnic Institute (wpi.edu)\New Study Material\DR - Soft Vision\Matlab code\Softyrobot';
pos = 1;
len = 78;
trj = zeros(len,2);
video = VideoWriter('triangle.avi');
video.FrameRate = 15;
open(video);

for i = 1:len
    imgname = append(name,'\vid9\','img',int2str(i),'.jpg');
    I = imread(imgname);
    [id,loc] = readAprilTag(I,"tag36h11");
    idx = find(id==0);
    markerRadius = 4;
    numCorners = size(loc,1);
    markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
    I = insertShape(I,"FilledCircle",markerPosition,"Color","magenta","Opacity",1);
    I = insertShape(I,"Line",[loc(1,:,idx),loc(2,:,idx), loc(3,:,idx), loc(4,:,idx), loc(1,:,idx)], ...
    "Color",["blue"],"LineWidth",2);
    targetPosition = [420, 438,6];
    pos = (loc(1,:,idx)+loc(2,:,idx)+ loc(3,:,idx)+ loc(4,:,idx))/4;
    trj(i,:) = pos;
    if i ~= 1
        I = insertShape(I,"Line",trj(1:i,:),"Color",["green"],"LineWidth",2);
    end
    I = insertShape(I,"FilledCircle",targetPosition,"Color","red","Opacity",1);
    imshow(I);
    writeVideo(video,I);
end

close(video);
