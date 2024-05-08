clc;
clear all;
close all;
%% 地图起点初始化
figure;
fig=imread('../map/map.png');
Imp=rgb2gray(fig);
imshow(Imp);
hold on;
width=size(Imp,1);
length=size(Imp,2);
[x,y]=ginput(2);
startPoint=[x(1) y(1)];
goalPoint=[x(2) y(2)];
plot(startPoint(1),startPoint(2),'mo','MarkerSize',5,'MarkerFaceColor','m');
plot(goalPoint(1),goalPoint(2),'rp','MarkerSize',5,'MarkerFaceColor','r');
disp(['起点：',num2str(startPoint)]);
disp(['终点：',num2str(goalPoint)]);
%% 设置初始参数   
param.sampleLength=20;      %采样步长
param.findPathFlag=0;
param.sampleNums=0;

%% 建立两棵树
tr1(1).position=startPoint;
tr1(1).parentind=0;
tr1(1).cost=0;

tr2(1).position=goalPoint;
tr2(1).parentind=0;
tr2(1).cost=0;

while 1
    randPoint=[length*rand, width*rand];
    minDis=inf;
    minPointInd=0;
    %从树中寻找距离randPoint最近的节点
    for i=1:numel(tr1)
        dis=norm(randPoint-tr1(i).position);
        if dis<minDis
            minDis=dis;
            minPointInd=i;
        end
    end
    new2rand=randPoint-tr1(minPointInd).position;
    newPoint=tr1(minPointInd).position+new2rand/norm(new2rand)*param.sampleLength;
    flag = collisionCheck(newPoint,tr1(minPointInd).position,Imp);
    if ~flag %有障碍物
        continue;
    end
    param.sampleNums=param.sampleNums+1;
    title(['RRT Connect Algorithm 采样次数：',num2str(param.sampleNums)]);
    plot(newPoint(1),newPoint(2),'ro','MarkerFaceColor','r','MarkerSize',2);
    plot([newPoint(1),tr1(minPointInd).position(1)],[newPoint(2),tr1(minPointInd).position(2)],'b-');
    pause(0.1);
    param.newPointInd=numel(tr1)+1;
    tr1(param.newPointInd).position=newPoint;
    tr1(param.newPointInd).parentind=minPointInd;
    tr1(param.newPointInd).cost=tr1(minPointInd).cost+param.sampleLength;
    % 从另一棵树寻找距离newpoint最近的节点，并判断是否可连接
    [connectFlag,connectPointInd]=findConnectNode(newPoint,tr2,param.sampleLength,Imp);
    if connectFlag
        plot([tr1(param.newPointInd).position(1),tr2(connectPointInd).position(2)],[tr1(param.newPointInd).position(2),tr2(connectPointInd).position(2)],...
            'g-','LineWidth',3);
        param.findPathFlag=1;
        break;
    else
        while 1
            anotherNewPoint=tr2(connectPointInd).position+(newPoint-tr2(connectPointInd).position)...
                /norm(tr2(connectPointInd).position-newPoint)*param.sampleLength;
            flag = collisionCheck(anotherNewPoint,tr2(connectPointInd).position,Imp);
            if ~flag
                break;
            end
            param.sampleNums=param.sampleNums+1;
            title(['RRT Connect Algorithm 采样次数：',num2str(param.sampleNums)]);
            plot(anotherNewPoint(1),anotherNewPoint(2),'ro','MarkerFaceColor','r','MarkerSize',2);
            plot([anotherNewPoint(1),tr2(connectPointInd).position(1)],[anotherNewPoint(2),tr2(connectPointInd).position(2)],...
                'b-');
            pause(0.1);
            param.newPointInd=numel(tr2)+1;
            tr2(param.newPointInd).position=anotherNewPoint;
            tr2(param.newPointInd).cost=tr2(connectPointInd).cost+param.sampleLength;
            tr2(param.newPointInd).parentind=connectPointInd;
            if norm(anotherNewPoint-newPoint)<param.sampleLength
                if collisionCheck(anotherNewPoint,newPoint,Imp)
                    plot([anotherNewPoint(1),newPoint(1)],[anotherNewPoint(2),newPoint(2)],'g-','LineWidth',3);
                    param.findPathFlag=1;
                    break;
                end
            end
            connectPointInd=numel(tr2);
        end
        if param.findPathFlag
            break;
        end
        %swap,下次拓展从较小的一棵树进行
        if numel(tr2)<numel(tr1)
            param.temp=tr1;
            tr1=tr2;
            tr2=param.temp;
        end
    end
end
%% 回溯路径
param.tempInd1=numel(tr1);
param.tempInd2=numel(tr2);
while param.tempInd1>1
    plot([tr1(param.tempInd1).position(1),tr1(tr1(param.tempInd1).parentind).position(1)],...
        [tr1(param.tempInd1).position(2),tr1(tr1(param.tempInd1).parentind).position(2)],...
        'g-','LineWidth',3);
    param.tempInd1=tr1(param.tempInd1).parentind;
end
while param.tempInd2>1
    plot([tr2(param.tempInd2).position(1),tr2(tr2(param.tempInd2).parentind).position(1)],...
        [tr2(param.tempInd2).position(2),tr2(tr2(param.tempInd2).parentind).position(2)],...
        'g-','LineWidth',3);
    param.tempInd2=tr2(param.tempInd2).parentind;
end
