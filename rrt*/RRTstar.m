clear;
close all;
clc;

%% 载入地图
figure(1);
pic=imread('../map/map2.png');
fig=rgb2gray(pic);
imshow(fig);
hold on;
mLength=size(fig,2);
mWidth=size(fig,1);
Point=ginput(2);  %获取起点坐标、终点坐标
startPoint=Point(1,:);
goalPoint=Point(2,:);

disp(['起点坐标为：(',num2str(startPoint),')']);
disp(['终点坐标为：(',num2str(goalPoint),')']);

plot(startPoint(1),startPoint(2),'mo','MarkerSize',8,'MarkerFaceColor','m');
plot(goalPoint(1),goalPoint(2),'rp','MarkerSize',8,'MarkerFaceColor','r');


%% 参数设置
sampleNums=5000;
stepLength=30;
%建立树的节点列表
nodes_list(1).position=startPoint;
nodes_list(1).parentind=-1;
nodes_list(1).cost=0;
nodes_list(1).index=1;
count=0;
lhandlelist=[];
phandlelist=[];
pathhandlelist=[];
findflag=0;
updatepathcount=0;
goalindex=1;
for j=1:sampleNums
    count=count+1;
%随机采样，rand-->new
node_rand=[mLength*rand,mWidth*rand];
N=size(nodes_list,2);
minDis=Inf;
for i=1:N
    d=norm(node_rand-nodes_list(i).position);
    if minDis > d
        minDis=d;
        ind=i;
    end
end
near2rand=(node_rand-nodes_list(ind).position)/norm(node_rand-nodes_list(ind).position)*stepLength;
newnode=nodes_list(ind).position+near2rand;
% collisionchecking
flag=collisionCheck(newnode,nodes_list(ind).position,fig);
if ~flag
    continue;
end
%以newnode节点为中心，半径R为半径搜索节点
R=30/ceil(0.001*numel(nodes_list));
dis2new=nodes_list(ind).cost+stepLength;
minCostIndex=ind;
nearNodeList=[];
for index_near=1:N
    if norm(nodes_list(index_near).position-newnode)<R
        nearNodeList=[nearNodeList,nodes_list(index_near)]; %if node in the range, put it in the nearNodeList
        if norm(nodes_list(index_near).position-newnode)+nodes_list(index_near).cost<dis2new
            dis2new=norm(nodes_list(index_near).position-newnode)+nodes_list(index_near).cost;
            flag=collisionCheck(newnode,nodes_list(index_near).position,fig);
            if flag
                minCostIndex=index_near;
            end
        end
    end
end
lhandle=plot([nodes_list(minCostIndex).position(1),newnode(1)],[nodes_list(minCostIndex).position(2),newnode(2)],'b-','LineWidth',1);
lhandlelist=[lhandlelist,lhandle];
phandle=plot(newnode(1),newnode(2),'ro','MarkerFaceColor','r','MarkerSize',2);
phandlelist=[phandlelist,phandle];
drawnow;
nodes_list(N+1).position=newnode;
nodes_list(N+1).parentind=minCostIndex;
nodes_list(N+1).cost=nodes_list(minCostIndex).cost+norm(near2rand);
nodes_list(N+1).index=N+1;
%% rewire
NearNums=length(nearNodeList);
for iterNear=1:NearNums
    nearNewCost=norm(nearNodeList(iterNear).position-newnode);
    if nearNewCost+nodes_list(N+1).cost<nearNodeList(iterNear).cost
        flag=collisionCheck(newnode,nearNodeList(iterNear).position,fig);
        if flag
            nodes_list(nearNodeList(iterNear).index).parentind=N+1;
            nodes_list(nearNodeList(iterNear).index).cost=nearNewCost+nodes_list(N+1).cost;
            %delete(lhandlelist(nearNodeList(iterNear).index-1));
            lhandlelist(nearNodeList(iterNear).index-1)=plot([nearNodeList(iterNear).position(1),newnode(1)],...
                [nearNodeList(iterNear).position(2),newnode(2)],'r-','LineWidth',2);
            drawnow;
        end
    end
end
if norm(newnode-goalPoint)<=stepLength && ~findflag
    findflag=1;
    nodes_list(N+2).position=goalPoint;
    nodes_list(N+2).parentind=N+1;
    nodes_list(N+2).cost=nodes_list(N+1).cost+norm(newnode-goalPoint);
    nodes_list(N+2).index=N+2;
    goalindex=N+2;
    %plot([newnode(1),goalPoint(1)],[newnode(2),goalPoint(2)],'b-');
end
title(['RRT* algorithm ','sampleNums:',num2str(j)]);
if findflag
    updatepathcount=updatepathcount+1;
    if updatepathcount==50
        updatepathcount=0;
        m=2;
        path=[];
        path(1).position=goalPoint;
        pathIndex=nodes_list(goalindex).parentind;
        while 1
            path(m).position = nodes_list(pathIndex).position;
            pathIndex = nodes_list(pathIndex).parentind;    % 沿终点回溯到起点
            if pathIndex == -1
                break
            end
            m=m+1;
        end
        for delete_index=1:length(pathhandlelist)
            delete(pathhandlelist(delete_index));
        end
        for m=2:length(path)
            pathhandle=plot([path(m).position(1),path(m-1).position(1)],[path(m).position(2),path(m-1).position(2)],'g-','LineWidth',3);
            pathhandlelist=[pathhandlelist,pathhandle];
        end
    end
end
end
for delete_index=1:length(pathhandlelist)
    delete(pathhandlelist(delete_index));
end
for m=2:size(path,2)
    pathhandle=plot([path(m).position(1),path(m-1).position(1)],[path(m).position(2),path(m-1).position(2)],'g-','LineWidth',3);
    pathhandlelist=[pathhandlelist,pathhandle];
end

    disp(['SUCCESS!采样次数为：',num2str(count)]);