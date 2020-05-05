function [ok] = PathCheck(conf1,conf2,number)
    q = trapveltraj([conf1,conf2],number);
    anyCollision = false;
	for i = 2:length(q(1,:))-1
        if CollisionCheck(q(:,i))
            anyCollision=true;
            break;
        end
    end
    ok=~anyCollision;
end

