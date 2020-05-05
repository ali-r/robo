function [collision] = CollisionCheck(conf)

global robot collisionArray worldCollisionArray checkA checkB checkC;

    collision=true;
    pos = getTransform(robot,conf,'EndEffector_Link');
    if all(pos(1:3,4)>0) && pos(1,4)<checkA && pos(2,4)<checkB && pos(3,4)<checkC
        [collision,~,~] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,conf,false);
    end
    
end

