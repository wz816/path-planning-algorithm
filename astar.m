%*************%
%astar algorithm
%Author:wang
%Date:2024.4.19
%*************%
clc;clear;close all;
figure;
width=50;
len=50;
map=ones(width,len)*255;
obstaclePercent=0.2;
map(ind2sub([width,len],ceil(width*len.*rand(width*len*obstaclePercent,1))))=0;

imshow(map,'InitialMagnification', 'fit');
title('AStar Algorithm');
hold on;
axis on;
axis equal;
axis([0,width+1,0,len+1]);
[x,y]=ginput(2);
startPoint=[round(x(1)),round(y(1))];
goalPoint=[round(x(2)),round(y(2))];
plot(startPoint(1),startPoint(2),'go','MarkerFaceColor','g');
plot(goalPoint(1),goalPoint(2),'rp','MarkerFaceColor','r');
disp(startPoint);
disp(goalPoint);
closelist=[];
openlist=[];
openlist(1).position=startPoint;
openlist(1).g=0;
openlist(1).h=norm(goalPoint-startPoint);
openlist(1).f=openlist(1).g+openlist(1).h;
openlist(1).parent=[0,0];
openlist(1).parentind=0;
findflag=0;
closeflag=zeros(width,len);
moves=[-1,0;-1,1;0,1;1,1;1,0;1,-1;0,-1;-1,-1];
while ~findflag
    %从closelist中寻找f值最小的节点
    minf=inf;
    minfind=0;
    for i=1:numel(openlist)
        if openlist(i).f<minf
            minf=openlist(i).f;
            minfind=i;
        end
    end
    closelist=[closelist,openlist(minfind)];
    closeflag(openlist(minfind).position(1),openlist(minfind).position(2))=1;
    k=numel(closelist);
    openlist(minfind)=[];
    for j=1:size(moves,1)
        extened.position=[closelist(k).position(1)+moves(j,1),closelist(k).position(2)+moves(j,2)];
        if extened.position(1)<1 || extened.position(1)>len || extened.position(2)<1 ...
                || extened.position(2)>width ||map(extened.position(2),extened.position(1))==0
            continue;
        end
        extened.g=closelist(k).g+norm(extened.position-closelist(k).position);
        extened.h=norm(extened.position-goalPoint);
        extened.f=extened.g+extened.h;
        extened.parent=closelist(k).position;
        extened.parentind=numel(closelist);
        if closeflag(extened.position(1),extened.position(2))  %判断是否已经在closelist中
            continue;
        end
        if extened.position==goalPoint
            closelist=[closelist,extened];
            findflag=1;
            break;
        end
        if findflag
            break;
        end
        nodeflag=0; %判断新拓展节点是否已经在openlist中
        if numel(openlist) 
            for l=1:numel(openlist)
                if extened.position==openlist(l).position
                    nodeflag=1;
                    break;
                end
            end
        end
        if nodeflag
                if extened.f<openlist(l).f
                    openlist(l).g=extened.g;
                    openlist(l).h=extened.h;
                    openlist(l).f=extened.f;
                end
        else
                openlist=[openlist,extened];
                plot(extened.position(1),extened.position(2),'o');
                pause(0.01);
        end
    end  
end
%回溯路径
currentnode=closelist(end);
while currentnode.parentind
    plot([currentnode.position(1),closelist(currentnode.parentind).position(1)],[currentnode.position(2),closelist(currentnode.parentind).position(2)],...
        'r-','LineWidth',1.5);
    currentnode=closelist(currentnode.parentind);
end
