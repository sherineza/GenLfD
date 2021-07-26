function [orientless] = identify_orientationless (model,Data)

%% Method 3: check thin Gs and check distance between paths
orientless = ones(model.nbFrames, 1);
for j=1:model.nbFrames
    orientless(j)=1;
    for p=1:model.nbStates
        [eigvec,eigval]=eig(model.Sigma(2:3,2:3,j,p));
        eigval=eig(model.Sigma(2:3,2:3,j,p));
        %[eigvec3,eigval3]=eig(model.Sigma(:,:,j,p));
        [eigval_sort, indx] = sort(eigval);
        eig_ratio(p,j)=eigval_sort(2)/eigval_sort(1);
        if eig_ratio(p,j)>10
            %Need to add more restrictions on the consitutuants of the ...
            %state. make sure the state has equal contribution from all
            %frames. even if at different time steps.
%             for k=1:model.nbSamples
               %get points belonging to the gaussian
%                [val,indx]=max(model.Pix,[],1);
%                points = (indx==p);
               tvec = [model.Sigma(1,2,j,p), model.Sigma(1,3,j,p)];%vector 
               longvec = eigvec(:,indx(end));
               angle1=acos(dot(tvec,longvec)/(sqrt(sum(dot(tvec,tvec)))*sqrt(sum(dot(longvec,longvec)))))
               %             end
               if abs(angle1) < 0.1 || abs(angle1) > (pi-0.1)
                   orientless(j)=0;
               end
        end
    end
end
eig_ratio

%% Method 2: model theta GMM and see if any of the G are horizontal stretched.
%Thats means that the orientation was constant for a while. However this
%doesn't work when the frame is offset 
%{
model_theta.nbFrames = model.nbFrames; %Number of candidate frames of reference
model_theta.nbVar = 2; %Dimension of the datapoints in the dataset (here: t,x1,x2)
model_theta.dt = model.dt; %Time step duration
model_theta.params_diagRegFact = model.params_diagRegFact; %Optional regularization term
model_theta.nbData = model.nbData; %Number of datapoints in a trajectory
model_theta.nbStates = model.nbStates ; %Number of Gaussians in the GMM
model_theta.nbSamples=model.nbSamples;

for n=1:model.nbSamples
    for m=1:model.nbFrames
        Data2(1,m,(n-1)*model.nbData+1:n*model.nbData)=Data(1,m,(n-1)*model.nbData+1:n*model.nbData);
        Data2(2,m,(n-1)*model.nbData+1:n*model.nbData)=sqrt(Data(2,m,(n-1)*model.nbData+1:n*model.nbData).^2+Data(3,m,(n-1)*model.nbData+1:n*model.nbData).^2);
        Data2(3,m,(n-1)*model.nbData+1:n*model.nbData)=atan2(Data(3,m,(n-1)*model.nbData+1:n*model.nbData),Data(2,m,(n-1)*model.nbData+1:n*model.nbData));
    end
end

model_theta=init_tensorGMM_timeBased(Data2([1,3],:,:), model_theta);
model_theta=EM_tensorGMM(Data2([1,3],:,:), model_theta);

%Precomputation of covariance inverses
for m=1:model.nbFrames
    for i=1:model.nbStates
        model_theta.invSigma(:,:,m,i) = inv(model_theta.Sigma(:,:,m,i));
    end
end

orientless = ones(1, model.nbFrames);
for j=1:model. nbFrames
    for i=1:model.nbStates
       eigval=eig(model_theta.Sigma(:,:,j,i));
        eigval_sort=sort(eigval);
        eig_ratio(p,j)=eigval_sort(2)/eigval_sort(1);
        if eig_ratio(p,j)>10          
            orientless(j)=0;
        end
    end
end
%}

%% Method v1: check the closest Gaussian and see the range of angle difference between the paths. 
% This fails when the frame is offset from the rigid body.

% orientless = zeros(1, model.nbFrames);
% %automatic detection
% for m=1:model.nbFrames
%     %check for the state that is the closest to the frames (potentially go
%     %straight to one of the paths and check which is the closest set of
%     %points)
%     closestd=10000;
%     closestid=0;
%     for i=1:model.nbStates
%         tempd= sqrt(model.Mu(2,m,i)^2+model.Mu(3,m,i)^2);
%         if closestd > tempd
%             closestd=tempd;
%             closestid=i;
%         end
%     end
%
%     %check if points are at a very dispersed angles?
%
%     %loop through points
%     startperct=sum(model.Priors(1:closestid-1));
%     endperct=sum(model.Priors(1:closestid));
%     startpoint=ceil(1+model.nbData*startperct);
%     endpoint=floor(model.nbData*endperct);
%     for n=1:model.nbSamples
%         for k=startpoint:endpoint
%             %find angles
%             angles(n,k-startpoint+1)=atan2(Data(2,m,k+(n-1)*model.nbData),Data(3,m,k+(n-1)*model.nbData));
%         end
%     end
%     %find measure of variability
%     score = range(angles,1);
%     score_av=sum(score)/length(score);
%     %threshold the measure and see if it is orientless
%     threshold = 90/180*pi;
%     if score_av > threshold
%         orientless(m)=1;
%     end
% end
end