function [SigmaGMR,MuGMR] = getTPGMR(model,s)

DataIn(1,:) = s(1).Data(1,:); 
in = 1;
out = 2:model.nbVar;
MuGMR = zeros(length(out), model.nbData, model.nbFrames);
SigmaGMR = zeros(length(out), length(out), model.nbData, model.nbFrames);
H=[];
%Gaussian mixture regression
for m=1:model.nbFrames
    %Compute activation weights
    for i=1:model.nbStates
        H(i,:) = model.Priors(i) * gaussPDF(DataIn, model.Mu(in,m,i), model.Sigma(in,in,m,i));
    end
    H = H ./ (repmat(sum(H),model.nbStates,1)+realmin);
    
    for t=1:model.nbData
        %Compute conditional means
        for i=1:model.nbStates
            MuTmp(:,i) = model.Mu(out,m,i) + model.Sigma(out,in,m,i) / model.Sigma(in,in,m,i) * (DataIn(:,t) - model.Mu(in,m,i));
            MuGMR(:,t,m) = MuGMR(:,t,m) + H(i,t) * MuTmp(:,i);
        end
        %Compute conditional covariances
        for i=1:model.nbStates
            SigmaTmp = model.Sigma(out,out,m,i) - model.Sigma(out,in,m,i) / model.Sigma(in,in,m,i) * model.Sigma(in,out,m,i);
            SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
        end
        SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) - MuGMR(:,t,m) * MuGMR(:,t,m)' + eye(length(out)) * model.params_diagRegFact;
    end
end

end

