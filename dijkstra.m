%*********%
%author: wang
%date:24.4.16
%*********%

clc;clear;close all;
figure;
width=50;
length=50;
map=ones(width,length)*255;
obstaclePercent=0.2;
map(ind2sub([width,length],ceil(width*length.*rand(width*length*obstaclePercent,1))))=0;

imshow(map,'InitialMagnification', 'fit');
title('Dijkstra Algorithm');
hold on;
ax=gca;
axis on;
axis equal;
axis([0,width+1,0,length+1]);
[x,y]=ginput(2);
startPoint=[round(x(1)),round(y(1))];
goalPoint=[round(x(2)),round(y(2))];
plot(startPoint(1),startPoint(2),'go','MarkerFaceColor','g');
plot(goalPoint(1),goalPoint(2),'rp','MarkerFaceColor','r');
disp(startPoint);
disp(goalPoint);

costs=inf(width,length);
costs(startPoint(1),startPoint(2))=0;
visited = false(width, length);
moves=[-1,0;0,1;1,0;0,-1];
while ~visited(goalPoint(1),goalPoint(2))
    minCost=min(min(costs(~visited)));
    [currentX,currentY]=find(costs==minCost & visited==0);
    visited(currentX(1),currentY(1))=1;  %标记为已访问
    for i=1:size(moves,1)
        newX=currentX(1)+moves(i,1);
        newY=currentY(1)+moves(i,2);
        if newX==goalPoint(1) && newY==goalPoint(2)
            flag=1;
            break;
        end
         if newX >= 1 && newX <= width && newY >= 1 && newY <= length
            % 检查新位置是否可以通行 (不是障碍物)
            if map(newY, newX) == 255
                % 更新代价 (假设每个移动的代价都为 1)
                newCost = costs(currentX(1), currentY(1)) + 1; % 假设每个移动的代价都为 1
                if newCost < costs(newX, newY)
                    costs(newX, newY) = newCost;
                    plot(newX,newY,'*','MarkerEdgeColor',[.5,.5,.5]);
                    drawnow;
                end
            end
        end
    end
    if flag break; end
end
path = [goalPoint(1), goalPoint(2)]; % 初始化路径，起始点是目标点
currentX = goalPoint(1); % 当前节点的 x 坐标
currentY = goalPoint(2); % 当前节点的 y 坐标

% 从目标点开始回溯到起点
while ~(currentX == startPoint(1) && currentY == startPoint(2))
    neighbors = zeros(size(moves, 1), 2); % 初始化邻居节点数组
    % 遍历当前节点的所有邻居节点
    for i = 1:size(moves, 1)
        newX = currentX + moves(i, 1); % 计算邻居节点的 x 坐标
        newY = currentY + moves(i, 2); % 计算邻居节点的 y 坐标
        % 检查邻居节点是否在地图范围内
        if newX >= 1 && newX <= width && newY >= 1 && newY <= length
            neighbors(i, :) = [newX, newY]; % 将邻居节点的坐标添加到数组中
        end
    end
    % 获取邻居节点的代价值
    neighborCosts = costs(sub2ind([width, length], neighbors(:, 1), neighbors(:, 2)));
    % 找到邻居节点中代价最小的节点的索引
    [~, idx] = min(neighborCosts);
    % 更新当前节点为代价最小的邻居节点
    currentX = neighbors(idx, 1);
    currentY = neighbors(idx, 2);
    % 将当前节点的坐标添加到路径中
    path = [currentX, currentY; path];
end

% 绘制最短路径
plot(path(:, 1), path(:, 2), 'r', 'LineWidth', 2);
    
    

