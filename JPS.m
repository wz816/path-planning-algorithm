%********jps algorithm************
%author:wang
%date: 24.5.5
%
clc;clear;close all;

%% map parameters
m.length=60;
m.width=40;
m.obs_rate=0.1;
map=generateMap(m.length,m.width,m.obs_rate);
hold on;
%% get startPoint & goalPoint &get the path from startPoint to goalPoint
[x,y]=ginput(2);
startPoint=round([x(1),y(1)]);
goalPoint=round([x(2),y(2)]);
plot(startPoint(1),startPoint(2),'bo');
plot(goalPoint(1),goalPoint(2),'rp','MarkerFaceColor','r');
disp(['startPoint:',num2str(startPoint)]);
disp(['goalPoint:',num2str(goalPoint)]);
[path, findPathFlag]=jps(startPoint,goalPoint,map);
if findPathFlag
    disp('success');
else
    disp('fail');
end

%% generate map function
function map=generateMap(length,width,obs_rate)
    obs_nums=round(length*width*obs_rate);
    map=ones(width,length)*255;
    map(ind2sub([length,width],ceil(length*width*rand(obs_nums,1))))=0;
    map(1,:)=0;map(:,1)=0;
    map(width,:)=0; map(:,length)=0;
    imshow(map,'InitialMagnification','fit');
    title('JPS Algorithm');
end
%% jps function
function [path, findPathFlag]=jps(start,goal,map)
    openlist=[];
    closelist=[];
    findPathFlag=0;

    start_.position=start;
    start_.gcost=0;
    start_.fcost=norm(start-goal);
    start_.parentind=0;
    openlist=[openlist;start_];

    moves=[-1,0;-1,1;0,1;1,1;1,0;1,-1;0,-1;-1,-1];

    if numel(openlist)==0
        findPathFlag=0;
        return;
    end

    while 1
        minf=inf;
        minind=0;
        for i=1:numel(openlist)
            if minf>openlist(i).fcost
                minf=openlist(i).fcost;
                minind=i;
            end
        end
        currentNode=openlist(minind);
        closelist=[closelist;currentNode];
        openlist(minind)=[];
        % is goalPoint?
        if currentNode.position(1)==goal(1) && currentNode.position(2)==goal(2)            
            findPathFlag=1;
            return;
        end


        for j=1:size(moves,1)
            [jumpPoint, findFlag]=jump(moves(j,:),currentNode.position,map,goal);
            if ~findFlag
                continue;
            end
            if jumpPoint==goal
                findPathFlag=1;
                newPoint.position=jumpPoint;
                newPoint.gcost=currentNode.gcost+norm(newPoint.position-currentNode.position);
                newPoint.fcost=newPoint.gcost+norm(goal-newPoint.position);
                newPoint.parentind=findind(closelist,currentNode.position);
                closelist=[closelist;newPoint];
                break;
            end
            newPoint.position=jumpPoint;
            newPoint.gcost=currentNode.gcost+norm(newPoint.position-currentNode.position);
            newPoint.fcost=newPoint.gcost+norm(goal-newPoint.position);
            newPoint.parentind=findind(closelist,currentNode.position);
            plot(jumpPoint(1),jumpPoint(2),'*g','MarkerSize',6);
            drawnow;
            openlist=[openlist;newPoint];
        end
        if findPathFlag
            break;
        end
    end
 path=findPath(closelist,goal);
 return;
end   
%% 寻找跳跃点函数
function [new_node, flag]=jump(moves,currentNode,map,goal)
    new_node=currentNode+moves;
    flag=0;
    % 新拓展节点为终点
    if new_node==goal
        flag=1;
        return;
    end
    % 新拓展节点为障碍物或超出地图
    if map(new_node(2),new_node(1))==0 || new_node(1)<=1 || new_node(1)>size(map,2)-1 ...
            ||new_node(2)<=1||new_node(2)>size(map,1)-1
        return;
    end
   
        
    if forceNeighbor(moves,new_node,map,goal)
        flag=1;
        return;
    else 
        [new_node, flag]=jump(moves,new_node,map,goal);

    end

end
%% force neighbor function
function flag=forceNeighbor(moves,new_node,map,goal)
    x=new_node(1);
    y=new_node(2);
    flag=0;%是否存在force neighbor
    %水平移动
    if moves(1) && ~moves(2)
        if map(y+1,x)==0 && map(y+1,x+moves(1))==255
            flag=1;
            return;
        end
        if map(y-1,x)==0 && map(y-1,x+moves(1))~=0
            flag=1;
            return;
        end
    end
    %垂直移动
    if ~moves(1) && moves(2)
        if map(y,x-1)==0 && map(y+moves(2),x-1)~=0
            flag=1;
            return;
        end
        if map(y,x+1)==0 && map(y+moves(2),x+1)~=0
            flag=1;
            return;
        end
    end
    
    %对角线移动
    if moves(1) && moves(2)
        if map(y,x-moves(1))==0 &&  map(y+moves(2),x-moves(1))~=0
            flag=1;
            return;
        end
        if map(y+moves(2),x)==0 && map(y+moves(2),x+moves(1))~=0
            flag=1;
            return;
        end
        if new_node(1)==goal(1) || new_node(2)==goal(2)
            flag=1;
            return;
        end
    end

    
end

%% find the index in closelist
function ind=findind(closelist,node)
    ind=0;
    for ii=1:numel(closelist)
        if node(1)==closelist(ii).position(1) && node(2)==closelist(ii).position(2)
            ind=ii;
            break;
        end
    end
end

%% 回溯路径
function path=findPath(closelist,goal)
    path=[];
    ind=findind(closelist,goal);
    disp(['The cost is:',num2str(closelist(ind).fcost)]);
    title(['JPS Algorithm',' cost:',num2str(closelist(ind).fcost)]);
    path=[closelist(ind);path];
    while 1
        ind=closelist(ind).parentind;
        if ind==0
            break;
        end
        path=[closelist(ind);path];
    end
    for jj=1:numel(path)-1
        plot([path(jj).position(1),path(jj+1).position(1)],[path(jj).position(2),path(jj+1).position(2)],'r-','LineWidth',2);
    end
end


