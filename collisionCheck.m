function flag = collisionCheck(newnode,nearnode,map)
%COLLISIONCHECK 
%   flag==0 有障碍物，拓展失败 flag==1 无障碍物
    near2new=newnode-nearnode;
    flag=1;
    if newnode(1)>=size(map,2)||newnode(1)<=1||newnode(2)>=size(map,1)||newnode(2)<=1
        flag=0;
    end
for i=0:0.1:1
    if flag==0
        break;
    end
    checkingPoint=nearnode+i*near2new;
    if checkingPoint(1)>=size(map,2)||checkingPoint(1)<=1||checkingPoint(2)>=size(map,1)||checkingPoint(2)<=1
        flag=0;
        break;
    end
    if map(ceil(checkingPoint(2)),ceil(checkingPoint(1)))==0||map(ceil(checkingPoint(2)),floor(checkingPoint(1)))==0 ...
            ||map(floor(checkingPoint(2)),ceil(checkingPoint(1)))==0||map(floor(checkingPoint(2)),floor(checkingPoint(1)))==0
        flag=0;
        break;
    end
end

