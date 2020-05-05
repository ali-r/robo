function [collistion] = CollisionCheck(robot,conf,collisionArray,worldCollisionArray,a,b,c)
    collistion=true;
    pos = getTransform(robot,conf,'EndEffector_Link');
    if all(pos(1:3,4)>0) && pos(1,4)<a && pos(2,4)<b && pos(3,4)<c 
        [collision,~,~] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,conf,false);
    end
end

