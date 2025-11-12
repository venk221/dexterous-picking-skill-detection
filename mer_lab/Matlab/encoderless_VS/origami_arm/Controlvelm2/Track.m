function [anglecur, posx, posy] = Track(r)
    r = str2double(r);    
    load('initval.mat','res', 'angle', 'pixelx', 'pixely');
    load('curval.mat','anglecur');

    if any(r(:)==0)
        Index = (find(r==0));
        anglecur = 0.98 * anglecur + 0.02 * (r(Index+1) - angle);
        pixcurx = r(Index+2) - pixelx;
        pixcury = r(Index+3) - pixely;
        posx = (r(Index+2) - pixelx) * res;
        posy = (r(Index+3) - pixely) * res;
        
        save('curval.mat', 'res', 'anglecur', 'posx', 'posy', 'pixcurx', 'pixcury');
%         p = [anglecur pixcurx pixcury];
%         disp(p);
    end
end