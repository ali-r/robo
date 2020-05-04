clear all;
close all;
clc;




a=1.2;
b=1.2;
c=1.2;

%%
% Create two platforms
platform1 = collisionBox(0.5,0.5,0.5);
platform1.Pose = trvec2tform([a b/2 c]);

platform2 = collisionCylinder(0.15,0.4);
platform2.Pose = trvec2tform([a/2 b/2 c/2]);

% Store in a cell array for collision-checking
worldCollisionArray = {platform1 platform2};

%%
robot = loadrobot("kinovaGen3","DataFormat","column","Gravity",[0 0 -9.81]);
%ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
%show(robot,homeConfiguration(robot),"Parent",ax);

%%
% Generate an array of collision objects from the visuals of the associated tree
collisionArray = exampleHelperManipCollisionsFromVisuals(robot);

startPose = trvec2tform([a/2,0,c/2])*axang2tform([1 0 0 pi/2]);
endPose = trvec2tform([0,0.85,0])*axang2tform([1 0 0 -pi/2]);

% Use a fixed random seed to ensure repeatable results
rng(0);
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);
startConfig = ik("EndEffector_Link",startPose,weights,robot.homeConfiguration);
endConfig = ik("EndEffector_Link",endPose,weights,robot.homeConfiguration);

% Show initial and final positions
%show(robot,startConfig);
%show(robot,endConfig);

[q,qd,qdd,t] = trapveltraj([startConfig,endConfig],100,"EndTime",2);%homeConfiguration(robot),

%Initialize outputs
isCollision = false(length(q),1); % Check whether each pose is in collision
selfCollisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision
worldCollisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision

for i = 1:length(q)
    [isCollision(i),selfCollisionPairIdx{i},worldCollisionPairIdx{i}] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,q(:,i),false);
end
isTrajectoryInCollision = any(isCollision)

%%

% Plot the 
ax2 = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);

% Visualize the robot in its home configuration
show(robot,q(:,length(q)),"Parent",ax2);

% Update the axis size

% Loop through the other positions
for i = 1:length(q)
    
    pos=getTransform(robot,q(:,i),'EndEffector_Link');
    
    if isCollision(i)==1
        plot3(pos(1,4),pos(2,4),pos(3,4),'r.','MarkerSize',20);
    else 
        plot3(pos(1,4),pos(2,4),pos(3,4),'b.','MarkerSize',20);
    end
end 

% %%
% % Loop through the other positions
% for i = 1:length(q)
%     
%     show(robot,q(:,i),"Parent",ax2,"PreservePlot",false);
%     
%     % Update the figure    
%     drawnow
%     
% end


np = 500

% prm 
random_q = [4 ;4 ;4 ;4 ;4 ;4; 4 ].*rand(7,np) - 2;

okPoint = false(np,1);

for i=1:np
    conf = random_q(:,i);
    pos = getTransform(robot,conf,'EndEffector_Link');
    if all(pos(1:3,4)>0) && pos(3,4)<c/2
        [collision,~,~] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,conf,false);
  
        okPoint(i) = ~collision;
        % draw
        if okPoint(i)==1
            plot3(pos(1,4),pos(2,4),pos(3,4),'b.','MarkerSize',10);
        else 
            plot3(pos(1,4),pos(2,4),pos(3,4),'r.','MarkerSize',10);
        end
        
    end
end

random_q = random_q(:,okPoint);

random_q =[ random_q , startConfig , endConfig ]

np = length(random_q(1,:))


for i=1:np
    conf = random_q(:,i);
    pos = getTransform(robot,conf,'EndEffector_Link');
        
    idxs = knnsearch(random_q',conf','K',4);
    for i2 = 1:length(idxs)
        pos2 = getTransform(robot,random_q(:,idxs(i2)),'EndEffector_Link');
        plot3( [pos(1,4) pos2(1,4)],[pos(2,4) pos2(2,4)],[pos(3,4) pos2(3,4)],'k-' )
        
    end
   
end

%%















