function [model, GAMMA0, GAMMA2] = EM_tensorGMM(Data, model)
% Training of a task-parameterized Gaussian mixture model (GMM) with an expectation-maximization (EM) algorithm.
% The approach allows the modulation of the centers and covariance matrices of the Gaussians with respect to
% external parameters represented in the form of candidate coordinate systems.
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon15,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%   year="2015"
% }
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.


%Parameters of the EM algorithm
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-5; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,3);

%diagRegularizationFactor = 1E-2; %Optional regularization term
diagRegularizationFactor = 1E-8; %Optional regularization term


for nbIter=1:nbMaxSteps
	fprintf('.');
	
	%E-step
	[L, GAMMA, GAMMA0] = computeGamma(Data, model); %See 'computeGamma' function below
    
    %ADDEDD BY SHIRINE to try to avoid overfitting to one demonstration as
    %opposed to respecting many demos. This method wasn't very effective. 
    if mod(nbIter,1)==0
        NBDATA=model.nbData;
        NBDEMOS=model.nbSamples;
        window=3;%floor(NBDATA/(model.nbStates*10));
        for i=1+window:NBDATA-window
            av=0;
            for j=1:NBDEMOS
                av=av+sum(GAMMA(:,(j-1)*NBDATA+i-window:(j-1)*NBDATA+i+window),2);
            end
            GAMMA(:,i)=av./(NBDEMOS*window);
        end
        GAMMA=repmat(GAMMA(:,1:NBDATA),1,NBDEMOS);
    end
    
    GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
	model.Pix = GAMMA2;
	
    % maybe change this to make the total nb of points contributed from
    % each demo equal in a gaussian so that g doesn't tend to one demo over
    % other.
    
	%M-step
	for i=1:model.nbStates
		
		%Update Priors
		model.Priors(i) = sum(sum(GAMMA(i,:))) / nbData;
		
		for m=1:model.nbFrames
			%Matricization/flattening of tensor
			DataMat(:,:) = Data(:,m,:);
			
			%Update Mu
			model.Mu(:,m,i) = DataMat * GAMMA2(i,:)';
			
			%Update Sigma (regularization term is optional)
			DataTmp = DataMat - repmat(model.Mu(:,m,i),1,nbData);
			model.Sigma(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(size(DataTmp,1)) * diagRegularizationFactor;
		end
	end
	
	%Compute average log-likelihood
	LL(nbIter) = sum(log(sum(L,1))) / size(L,2);
	%Stop the algorithm if EM converged (small change of LL)
	if nbIter>nbMinSteps
		if LL(nbIter)-LL(nbIter-1)<maxDiffLL || nbIter==nbMaxSteps-1
			disp(['EM converged after ' num2str(nbIter) ' iterations.']);
			return;
		end
	end
end
disp(['The maximum number of ' num2str(nbMaxSteps) ' EM iterations has been reached.']);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Lik, GAMMA, GAMMA0] = computeGamma(Data, model)
nbData = size(Data, 3);
Lik = ones(model.nbStates, nbData);
GAMMA0 = zeros(model.nbStates, model.nbFrames, nbData);
for i=1:model.nbStates
	for m=1:model.nbFrames
		DataMat(:,:) = Data(:,m,:); %Matricization/flattening of tensor
		GAMMA0(i,m,:) = gaussPDF(DataMat, model.Mu(:,m,i), model.Sigma(:,:,m,i));
        
        %%ADDED BY SHIRINE. effective.
        if 0
            NBDATA=model.nbData;
            NBDEMOS=model.nbSamples;
            win=floor(NBDATA/(model.nbStates*10));
            filt=[0:1/(win+1):1];filt=[filt(2:end),fliplr(filt(2:end-1))];
            
            for iter_nbData=1:NBDATA
                av=0;
                for j=1:NBDEMOS
                    %pad the edges so that the entire thing is coverred
                    if iter_nbData-win<1
                        filtdata=[zeros(-iter_nbData+win+1,1);squeeze(GAMMA0(i,m,(j-1)*NBDATA+1:(j-1)*NBDATA+iter_nbData+win))];
                    elseif iter_nbData+win>NBDATA
                        filtdata=[squeeze(GAMMA0(i,m,(j-1)*NBDATA+iter_nbData-win:(j-1)*NBDATA+NBDATA));zeros(+iter_nbData+win-NBDATA,1)];
                    else
                        filtdata=squeeze(GAMMA0(i,m,(j-1)*NBDATA+iter_nbData-win:(j-1)*NBDATA+iter_nbData+win));
                    end
                    elem=filt*filtdata;
                    av=av+elem/(1+win);
                end
                diff=av/NBDEMOS-GAMMA0(i,m,(j-1)*NBDATA+iter_nbData);
                GAMMA0(i,m,(j-1)*NBDATA+iter_nbData)=GAMMA0(i,m,(j-1)*NBDATA+iter_nbData)+diff*0.5;
            end
        end
        %%end
    
		Lik(i,:) = Lik(i,:) .* squeeze(GAMMA0(i,m,:))'; 
	end
	Lik(i,:) = Lik(i,:) * model.Priors(i);
end
GAMMA = Lik ./ repmat(sum(Lik,1)+realmin, size(Lik,1), 1);
end
