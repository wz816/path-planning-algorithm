%*****RRT*****
clear;
close all;
clc;
%video=VideoWriter('RRT.avi');
%open(video);
%% 构建地图
figure(1);
pic=imread('map/map.png');
fig=rgb2gray(pic);
imshow(fig);
hold on;
title('RRT algorithm');
mLength=size(fig,2);
mWidth=size(fig,1);
Point=ginput(2);
startPoint=Point(1,:);
goalPoint=Point(2,:);
% startPoint=[120,340];
% goalPoint=[360,120];
disp(['起点坐标为：(',num2str(startPoint),')']);
disp(['终点坐标为：(',num2str(goalPoint),')']);
% frame=getframe(gcf);
% writeVideo(video,frame);
plot(startPoint(1),startPoint(2),'o','MarkerSize',6,'MarkerFaceColor',[1,0,1],'MarkerEdgeColor',[1,0,1]);
plot(goalPoint(1),goalPoint(2),'rp','MarkerSize',8,'MarkerFaceColor','r');
% frame=getframe(gcf);
% writeVideo(video,frame);
%% 参数初始化
sampleNums=30000;  %采样次数
stepLength=20;     %采样步长
%将起点加入树
nodes_list(1).position=startPoint;
nodes_list(1).parentind=0;
nodes_list(1).cost=0;
count=0;
for j=1:sampleNums
    count=count+1;
%随机采样，rand-->new
node_rand=[mLength*rand,mWidth*rand];
N=size(nodes_list,2); %获取树中当前节点数
minDis=Inf; %初始化树中节点距离newnode的最小距离
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
if flag
    plot(newnode(1),newnode(2),'ro','MarkerFaceColor','r','MarkerSize',2);
    plot([nodes_list(ind).position(1),newnode(1)],[nodes_list(ind).position(2),newnode(2)],'b-','LineWidth',1);
    
%     frame=getframe(gcf);
%     writeVideo(video,frame);
    
    nodes_list(N+1).position=newnode;
    nodes_list(N+1).parentind=ind;
    nodes_list(N+1).cost=nodes_list(ind).cost+norm(near2rand);
    if norm(newnode-goalPoint)<=stepLength
        nodes_list(N+2).position=goalPoint;
        nodes_list(N+2).parentind=N+1;
        nodes_list(N+2).cost=nodes_list(N+1).cost+norm(newnode-goalPoint);
        plot([newnode(1),goalPoint(1)],[newnode(2),goalPoint(2)],'b-');
        
%         frame=getframe(gcf);
%         writeVideo(video,frame);
        
        break;
    end
end
pause(0.0);
end

%%从目标点向后搜索，画出整条路径
index=N+2;
while nodes_list(index).parentind~=0
    plot([nodes_list(index).position(1),nodes_list(nodes_list(index).parentind).position(1)], ...
        [nodes_list(index).position(2),nodes_list(nodes_list(index).parentind).position(2)],'r-',...
        'LineWidth',3);
    index=nodes_list(index).parentind;
end

% for m=1:10
% frame=getframe(gcf);
% writeVideo(video,frame);
% end
% close(video);

if count==sampleNums
    disp('超出采样次数');
else
    disp(['SUCCESS!采样次数为：',num2str(count)]);
    disp(['cost:',num2str(nodes_list(N+2).cost)]);
end
        
    
    

    
    


