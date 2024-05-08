%*****prm algorithm*****
clc;clear;close all;
figure=imread('..\map\map3.png');
im=rgb2gray(figure);
im(im ~= 0)=255;
imshow(im);
hold on;
title('prm algorithm');

[x,y]=ginput(2);
start=[x(1),y(1)];
goal=[x(2),y(2)];
disp(['start:',num2str(start)]);
disp(['goal:',num2str(goal)]);
plot(start(1),start(2),'mo','MarkerFaceColor','b');
plot(goal(1),goal(2),'rp','MarkerFaceColor','r');

%%
nodesList=[];
startNode.position=start;
startNode.adjecentnodes=[];
startNode.cost=0;
nodesList=[nodesList;startNode];
param.pointNums=100;
param.length=400;
for ii=1:param.pointNums
    node.position=[rand*size(im,2),rand*size(im,1)];
    node.adjecentnodes=[];
    node.cost=inf;
    if node.position(1)<2 || node.position(1)>size(im,2)-1 || node.position(2)<2 || node.position(2)>size(im,1)-1
        continue;
    end
    if im(floor(node.position(2))-1,floor(node.position(1))-1)==0 ||...
            im(floor(node.position(2))-1,ceil(node.position(1))+1)==0 ||...
            im(ceil(node.position(2))+1,floor(node.position(1))-1)==0 ||...
            im(ceil(node.position(2))+1,ceil(node.position(1))+1)==0
        continue;
    end
    plot(node.position(1),node.position(2),'go','MarkerFaceColor','g');
    %drawnow;
    nodesList=[nodesList;node];
end
goalNode.position=goal;
goalNode.adjecentnodes=[];
goalNode.cost=inf; 
nodesList=[nodesList;goalNode];

for kk=1:numel(nodesList)-1
    for ll=kk+1:numel(nodesList)       
        if norm(nodesList(kk).position-nodesList(ll).position)<=param.length
            checkflag=collisionCheck(im,nodesList(kk),nodesList(ll));
            if ~checkflag
                continue;
            end
            nodesList(kk).adjecentnodes=[nodesList(kk).adjecentnodes;nodesList(ll).position];
            nodesList(ll).adjecentnodes=[nodesList(kk).position;nodesList(ll).adjecentnodes];
            plot([nodesList(kk).position(1),nodesList(ll).position(1)],[nodesList(kk).position(2),nodesList(ll).position(2)],...
                'b-','LineWidth',1);
            drawnow;
        end
    end
end

%% use dijkstra to find the path from start to goal
closelist=[];
openlist=[];
nodesList(1).parentPosition=[-1,-1];
openlist=[openlist,nodesList(1)];
findPathFlag=0;
while 1
    if ~numel(openlist)
        break;
    end
    minf=inf;
    minind=0;
    for i=1:numel(openlist)
        if minf>openlist(i).cost
            minf=openlist(i).cost;
            minind=i;
        end
    end
    curNode=openlist(minind);
    openlist(minind)=[];
    closelist=[closelist;curNode];
    for j=1:size(curNode.adjecentnodes,1)
        ind=findind(curNode.adjecentnodes(j,:),nodesList);
        % already in the closelist
        if numel(closelist)
            closeflag=0;
            for k=1:numel(closelist)
                if nodesList(ind).position==closelist(k).position
                    closeflag=1;
                    break;
                end
            end
            if closeflag
                continue;
            end
        end
        % already in the openlist or not
        openind=findind(nodesList(ind).position,openlist);
        if openind
            if curNode.cost+norm(curNode.position-curNode.adjecentnodes(j,:))<openlist(openind).cost
                openlist(openind).cost=curNode.cost+norm(curNode.position-curNode.adjecentnodes(j,:));
                continue;
            else 
                continue;
            end
        end
        nodesList(ind).cost=curNode.cost+norm(curNode.position-curNode.adjecentnodes(j,:));
        nodesList(ind).parentPosition=curNode.position;
        openlist=[openlist;nodesList(ind)];    
        if curNode.adjecentnodes(j,:)==goal
            findPathFlag=1;
            closelist=[closelist;nodesList(ind)];
            openlist(numel(openlist))=[];
            break;
        end
    end
    if findPathFlag
        break;
    end
end

if ~findPathFlag
    disp("fail");
else
    disp(['success',num2str(closelist(end).cost)]);
    path=[];
    pt=closelist(end).position;
    path=[path;pt];
    while 1
        plot([pt(1),closelist(findind(pt,closelist)).parentPosition(1)],[pt(2),closelist(findind(pt,closelist)).parentPosition(2)],...
            'r-','LineWidth',2);
        pt=closelist(findind(pt,closelist)).parentPosition;
        path=[pt;path];
        if pt==start
            break;
        end
    end
end
title(['prm algorithm  cost: ',num2str(closelist(end).cost)]);        

        

%% find the ind function
function ind=findind(node,nodesList)
    ind=0; % if return 0, show that the node isn't in nodesList
    for jj=1:numel(nodesList)
        if node==nodesList(jj).position
            ind=jj;
            return;
        end
    end
end
 
function flag=collisionCheck(map,node1,node2)  %flag==1,collision free
    flag=1;
    theta=atan2(node2.position(2)-node1.position(2),node2.position(1)-node1.position(1));
    if norm(node1.position-node2.position)<=1
        flag=1;
        return;
    end
    for i=0:1:norm(node1.position-node2.position)
        newPoint=node1.position+[i*cos(theta),i*sin(theta)];
        if map(round(newPoint(2)),round(newPoint(1)))==0
            flag=0;
            return;
        end
    end
end
    

