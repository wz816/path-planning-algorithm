function [flag,ind]=findConnectNode(point,tr,steplength,map)
minDis=inf;
flag=0;
for i=numel(tr)
    dis=norm(tr(i).position-point);
    if dis < minDis
        minDis=dis;
        ind=i;
    end
end
if minDis < steplength
    flag=collisionCheck(point,tr(ind).position,map);
end
end


