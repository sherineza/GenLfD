function [Mu_ring,Sigma_ring] = calculate_ringGMM (model,Data)
Mu_ring = zeros(model.nbData,model.nbFrames);
Sigma_ring=zeros(model.nbData,model.nbFrames);
method = 2;

%% Method 2: Calculating r-GMM and regressing to find mean and sigma at each point.
if method==2
    Dataxy=Data;
    
    model_r.nbVar=2;
    model_r.nbFrames = model.nbFrames ;
    model_r.dt = model.dt ;
    model_r.params_diagRegFact = model.params_diagRegFact;
    model_r.nbStates = model.nbStates;
    
    model_r.nbData= model.nbData;
    model_r.nbSamples = model.nbSamples;
    
    %converting to radius coordinate
    for n=1:model_r.nbSamples
        for m=1:model.nbFrames
            Data(1,m,(n-1)*model_r.nbData+1:n*model_r.nbData)=Dataxy(1,m,(n-1)*model_r.nbData+1:n*model_r.nbData);
            Data(2,m,(n-1)*model_r.nbData+1:n*model_r.nbData)=sqrt(Dataxy(2,m,...
                (n-1)*model_r.nbData+1:n*model_r.nbData).^2+Dataxy(3,m,(n-1)*model_r.nbData+1:n*model_r.nbData).^2);
        end
    end
    
    % Train the r-GMM
    model_r = init_tensorGMM_timeBased(Data([1,2],:,:), model_r);
    model_r = EM_tensorGMM(Data([1,2],:,:), model_r);
    
    %Precomputation of covariance inverses
    for m=1:model.nbFrames
        for i=1:model.nbStates
            model_r.invSigma(:,:,m,i) = inv(model_r.Sigma(:,:,m,i));
        end
    end
    
    DataIn(1,:) = Data(1,m,1:model_r.nbData); %1:nbData;
    in = 1;
    out = 2;
    
    MuGMR_r = zeros(model_r.nbData, model_r.nbFrames);
    SigmaGMR_r = zeros(model_r.nbData, model_r.nbFrames);
    
    %Gaussian mixture regression
    for m=1:model.nbFrames
        %Compute activation weights
        for i=1:model.nbStates
            H_r(i,:) = model_r.Priors(i) * gaussPDF(DataIn, model_r.Mu(in,m,i), model_r.Sigma(in,in,m,i));
        end
        H_r = H_r ./ (repmat(sum(H_r),model_r.nbStates,1)+realmin);
        
        for t=1:model_r.nbData
            %Compute conditional means
            for i=1:model.nbStates
                MuTmp(:,i) = model_r.Mu(out,m,i) + model_r.Sigma(out,in,m,i) / model_r.Sigma(in,in,m,i) * (DataIn(:,t) - model_r.Mu(in,m,i));
                MuGMR_r(t,m) = MuGMR_r(t,m) + H_r(i,t) * MuTmp(:,i);
            end
            %Compute conditional covariances
            for i=1:model.nbStates
                SigmaTmp = model_r.Sigma(out,out,m,i) - model_r.Sigma(out,in,m,i) / model_r.Sigma(in,in,m,i) * model_r.Sigma(in,out,m,i);
                SigmaGMR_r(t,m) = SigmaGMR_r(t,m) + H_r(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
            end
            SigmaGMR_r(t,m) = SigmaGMR_r(t,m) - MuGMR_r(t,m) * MuGMR_r(t,m)' + eye(length(out)) * model_r.params_diagRegFact;
        end
    end
    
    Mu_ring = MuGMR_r;
    Sigma_ring = SigmaGMR_r;
end

%Plot Ring Gaussian
% for m=1:model.nbFrames
%     figure;hold on;
%     for n=1:model.nbSamples
%         plot(squeeze(Dataxy(2,m,(n-1)*model.nbData+1)), ...
%             squeeze(Dataxy(3,m,(n-1)*model.nbData+1)), '.','markersize',15);
%         plot(squeeze(Dataxy(2,m,(n-1)*model.nbData+1:n*model.nbData)), ...
%             squeeze(Dataxy(3,m,(n-1)*model.nbData+1:n*model.nbData)), '-','linewidth',1.5, 'color', 'green');
%     end
%     for i=1:model.nbStates
%      t=linspace(-pi,pi,1000);
%      x=model_r.Mu(out,m,i)*[cos(t);sin(t)];x=[x,x(:,1)];
%      patch(x(1,:),x(2,:),[.5 .5 .5],'EdgeColor', [.5 .5 .5], 'facealpha', 0,'edgealpha', 0.5,'LineWidth',10000*model_r.Sigma(out,out,m,i));
%      patch(x(1,1:end-1),x(2,1:end-1),[.5 .5 .5]*0.5,'EdgeColor', [.5 .5 .5]*0.5, 'facealpha', 0,'edgealpha', 0.5,'LineWidth',1);
%     end
% end

%% Method 1: Calculating the mean and sigma for each time step individually
if method==1
    for n=1:model.nbFrames
        if model.orientless(n)==1
            for m=1:model.nbData
                for k=1:model.nbSamples
                    dtemp(k)=sqrt(Data(2,n,m+(k-1)*model.nbData)^2+Data(3,n,m+(k-1)*model.nbData)^2);
                    Mu_ring(m,n)=Mu_ring(m,n)+dtemp(k);
                end
                Mu_ring(m,n)=Mu_ring(m,n)/model.nbSamples;
                Sigma_ring(m,n)=(std(dtemp))^2;
            end
        end
    end
end