function [s] = feat2frame(Demo,INDEX,boolDisp)
nbSamples=length(Demo);
s(nbSamples)=struct('p',struct('b',[],'A',[]),'Data0',[],'nbData',0,'Data',[]);

i=1;
iter=1;
for j=1:length(Demo(i).corners)
    if length(find(INDEX(:,j,i)))==nbSamples
        for k=1:nbSamples
            %forming the translation vector
            s(k).p(iter).b=[0;Demo(k).corners.Location(INDEX(k,j,i),:)'];
            a=Demo(k).corners.Orientation(INDEX(k,j,i));
            %forming the rotation matrix
            s(k).p(iter).A=[[1,0,0];[0,cos(a), sin(a)];[0,-sin(a),cos(a)]];
            s(k).p(iter).ind=INDEX(k,j,i);
        end
        iter=iter+1;
    end
end

end

