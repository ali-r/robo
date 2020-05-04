clear all;
close all;
clc;




a=1;
b=1;
c=1;


% Create two platforms
platform1 = collisionBox(0.5,0.5,0.5);
platform1.Pose = trvec2tform([a b/2 c]);

platform2 = collisionCylinder(0.2,0.6);
platform2.Pose = trvec2tform([a/2 b/2 c/2]);

% Store in a cell array for collision-checking
worldCollisionArray = {platform1 platform2};

robot = loadrobot("kinovaGen3","DataFormat","column");
%ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
%show(robot,homeConfiguration(robot),"Parent",ax);


% Generate an array of collision objects from the visuals of the associated tree
collisionArray = exampleHelperManipCollisionsFromVisuals(robot);

startPose = trvec2tform([a/2,0,c/2])*axang2tform([1 0 0 pi/2]);
endPose = trvec2tform([0.0001,0.85,0.0001])*axang2tform([1 0 0 -pi/2]);

% Use a fixed random seed to ensure repeatable results
rng(0);
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);
startConfig = ik("EndEffector_Link",startPose,weights,robot.homeConfiguration);
endConfig = ik("EndEffector_Link",endPose,weights,robot.homeConfiguration);


%%

% Plot the 
ax2 = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);

% Visualize the robot in its home configuration
show(robot,startConfig,"Parent",ax2);

np = 300;

% prm 
random_q = [5 ;5 ;5 ;5 ;5 ;5; 0 ].*rand(7,np) - 2.5;
endConfig(7) = -2.5 ; 
startConfig(7) = -2.5 ; 
okPoint = false(np,1);

for i=1:np
    conf = random_q(:,i);
    pos = getTransform(robot,conf,'EndEffector_Link');
    
    if all(pos(1:3,4)>0) && pos(3,4)<c/2
        [collision,~,~] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,conf,false);
        okPoint(i) = ~collision;
        % draw
        if okPoint(i)==1
            plot3(pos(1,4),pos(2,4),pos(3,4),'b.','MarkerSize',15);
        else 
            plot3(pos(1,4),pos(2,4),pos(3,4),'r.','MarkerSize',10);
        end
    end
end

random_q = random_q(:,okPoint);
random_q =[ random_q , startConfig , endConfig ];
np = length(random_q(1,:))



%%

adjMat = ones(np)*inf;

for i=1:np
    conf = random_q(:,i);
    pos = getTransform(robot,conf,'EndEffector_Link');
        
    [idxs,mD] = knnsearch(random_q',conf','K',5);
    for i2 = 1:length(idxs)
        conf2 = random_q(:,idxs(i2));
        pos2 = getTransform(robot,conf2,'EndEffector_Link');
        q2 = trapveltraj([conf,conf2],5);
        anyCollision = false;
        for i3 = 1:length(q2(1,:))
            pos3 = getTransform(robot,q2(:,i3),'EndEffector_Link');
            [isCollision,~,~] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,q2(:,i3),false);
            anyCollision = anyCollision || isCollision;
            anyCollision = anyCollision ||~(all(pos3(1:3,4)>0) && pos3(3,4)<c/2);
            if anyCollision
                break
            end
        end
        if (anyCollision==1)
            plot3( [pos(1,4) pos2(1,4)],[pos(2,4) pos2(2,4)],[pos(3,4) pos2(3,4)],'r-.' )
        else
            plot3( [pos(1,4) pos2(1,4)],[pos(2,4) pos2(2,4)],[pos(3,4) pos2(3,4)],'b-' )
            adjMat(i,idxs(i2))=mD(i2);
            adjMat(idxs(i2),i)=adjMat(i,idxs(i2));
        end
    end
end

%%
path_idxs = dijkstra(np, adjMat, np-1, np);

q = trapveltraj(random_q(:,path_idxs),150);

for i = 1:length(q)
    
    pos=getTransform(robot,q(:,i),'EndEffector_Link');
    plot3(pos(1,4),pos(2,4),pos(3,4),'g.','MarkerSize',15);

end 

%%
% % animate
%{
 
for i = 1:length(q)
    
    show(robot,q(:,i),"Parent",ax2,"PreservePlot",false);
    drawnow
    
end

%}










