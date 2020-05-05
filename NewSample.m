function [sample,idx] = NewSample(graph,maxAng,maxLen,bias)

sample= (2*maxAng*ones(7,1).*rand(7,1)-maxAng)+bias;

[m,idx]=min(vecnorm(graph(1:6,:)-sample(1:6,1)));

nn=graph(:,idx);
while norm(sample(1:6,1)-nn(1:6,1))>maxLen
    sample=(sample*5+nn)/6;
end

end

