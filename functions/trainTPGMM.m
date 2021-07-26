function [model] = trainTPGMM(model,leadFrames,s)

Data = zeros(model.nbVar, model.nbFrames, model.nbSamples*model.nbData);
for n=1:model.nbSamples
    for k=1:model.nbFrames
        m=leadFrames(k);
        Data(1:3,k,(n-1)*model.nbData+1:n*model.nbData) = inv(s(n).p(m).A) * (s(n).Data(1:3,:) - repmat(s(n).p(m).b, 1, model.nbData));
        if model.nbVar>3
            Data(4:model.nbVar,k,(n-1)*model.nbData+1:n*model.nbData) =s(n).Data(4:model.nbVar,:);
        end
    end
end

fprintf('Parameters estimation of TP-GMM with EM:');
%model = init_tensorGMM_kmeans(Data, model);
model = init_tensorGMM_timeBased(Data, model);
model = EM_tensorGMM(Data, model);

for m=1:model.nbFrames
    for i=1:model.nbStates
        model.invSigma(:,:,m,i) = inv(model.Sigma(:,:,m,i));
    end
end

% plot
% for m=1:model.nbFrames
%     figure;hold on;
%     for n=1:model.nbSamples
%         plot(squeeze(Data(2,m,(n-1)*model.nbData+1)), ...
%             squeeze(Data(3,m,(n-1)*model.nbData+1)), '.','markersize',15);
%         plot(squeeze(Data(2,m,(n-1)*model.nbData+1:n*model.nbData)), ...
%             squeeze(Data(3,m,(n-1)*model.nbData+1:n*model.nbData)), '-','linewidth',1.5, 'color', 'green');
%     end
%     
%      plotGMM(squeeze(model.Mu(2:3,m,:)), squeeze(model.Sigma(2:3,2:3,m,:)), [.5 .5 .5],.5);
% end

%% which frame is orientationless?
%initialise

for j=1:model.nbFrames
    n=length(model.orientless);
    model.orientless(n+1)=1;
    for p=1:model.nbStates
        [eigvec,eigval]=eig(model.Sigma(2:3,2:3,j,p));
        eigval=eig(model.Sigma(2:3,2:3,j,p));
        %[eigvec3,eigval3]=eig(model.Sigma(:,:,j,p));
        [eigval_sort, indx] = sort(eigval);
        eig_ratio(p,j)=eigval_sort(2)/eigval_sort(1);
        if eig_ratio(p,j)>10
            tvec = [model.Sigma(1,2,j,p), model.Sigma(1,3,j,p)];%vector
            longvec = eigvec(:,indx(end));
            
            angle1=acos(dot(tvec,longvec)/(sqrt(sum(dot(tvec,tvec)))*sqrt(sum(dot(longvec,longvec)))));
            %             end
            if abs(angle1) < 0.1 || abs(angle1) > (pi-0.1)
                model.orientless(n+1)=0;
            end
        end
    end
end

% for m=1:model.nbFrames
%     %check for the state that is the closest to the frames (potentially go
%     %straight to one of the paths and checl which is the closest set of
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
%         model.orientless(m)=1;
%     end
% end

%% Calculate the ring Gaussians. This function is in 'C:\PhD\orientationless-frames-TP_ringGMMR' Folder.
[model.Mu_dohnut(:,:),model.Sigma_dohnut(:,:)] = calculate_ringGMM (model,Data);

end

