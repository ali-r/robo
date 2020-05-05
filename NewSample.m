function [sample,idxs] = NewSample(graph,maxAng,maxLen)

sample= 2*maxAng*ones(7,1).*rand(7,1)-maxAng;

[idxs,mD] = knnsearch(sample(1:6,1)',graph(1:6,:)','K',1);          % error

nn=graph(:,idxs);
while norm(sample(1:6,1)-nn(1:6,1))>maxLen
    sample=(sample+nn)/2;
end

end

