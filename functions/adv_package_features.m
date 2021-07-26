function features = adv_package_features(Demo,leadFrames,INDEX)
features=zeros(length(Demo),64,length(leadFrames));
i=1;
iter=1;
for j=1:length(Demo(i).corners)
    if length(find(INDEX(:,j,i)))==length(Demo) %&& length(find(leadFrames==j))==1
        for k=1:length(Demo)
            features(k,:,iter)=Demo(k).Features(INDEX(k,j,i),:);
        end
        iter=iter+1;
    end
end
end

