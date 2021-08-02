function [indx] = findirrelevant (task_path)
% in this version, the frames and paths are in real life coordinates and
% are converted before plotting to pixels
if nargin==0
    clc; clear; close all;
    %Adds all subfolders to path
    currentFolder=fileparts(mfilename('fullpath'));
    cd(currentFolder);
    addpath(genpath(currentFolder));
    %load images
    task_path = "tasks/batteryinbox";
end

%% Load Data
load(strcat(task_path,'/model.mat'));
load(strcat(task_path,'/paths.mat'));
load(strcat(task_path,'/features.mat'));
load(strcat(task_path, '/calib.mat'));

imgs = [dir(fullfile(task_path,'/*.PNG')),...
    ;dir(fullfile(task_path,'/*.jpg'));...
    dir(fullfile(task_path,'/*.JPEG'))];

%combine paths into struct s
for i=1:length(s)
    s(i).Data=paths(i).path(:,:);
end
image=imread(strcat(imgs(1).folder,'/', imgs(1).name));imshow(image); hold on;
lengthim = size(image,1);
widthim = size(image,2);
cntr_real=[cam_pos(1),cam_pos(2),cam_posrel(3)];
cntr_im=[widthim/2,lengthim/2];
length_real=2*cam_posrel(3)*tan(pi/6);
width_real=length_real*widthim/lengthim;

clear r;

%% Initialise Plot
clrmap = colormap('jet');
xx = round(linspace(1,64,model.nbSamples));
clrmap = min(clrmap(xx,:),.95);
figure('WindowState','maximized');
title('Ground Truth Path (green) + Reproduced path accounting for all frames (red) + Optimised path accounting for relevant frames only (white)');
%% Parameters
model.nbFrames=length(leadFrames);
Relev = ones(1, model.nbFrames)/model.nbFrames;
deltaRelev = zeros(1, model.nbFrames);
count = 0;
MuTmp = zeros(model.nbVar-1, model.nbData, model.nbFrames);
SigmaTmp = zeros(model.nbVar-1,model.nbVar-1,model.nbData, model.nbFrames);
rl_data = []; % contains cost, deltaRelev history
poolsize=2;
scalei =5E-2;scale=scalei;
period=model.nbFrames;
totalNum=period*10*poolsize;
% period=1
%% Train
% for iterNum=1:period%totalNum
for iterNum=1:totalNum
    cost=0;
    
    %obtain the relevency vector for this iteration (reinforment learning exploration)
    deltaRelev=((rand(1,model.nbFrames))-Relev)*scalei;
    thetaRelev = (Relev + deltaRelev)/sum(Relev + deltaRelev);
    [out,indx] = sort(thetaRelev,'descend');
    indx = indx(1:poolsize);
   
    % change between demonstrations
    id=ceil(rand()*(model.nbSamples));
    
    r(1).Data(:,:)=s(id).Data(2:3,:); %ground truth
    
    temp=reproGMR(s(id), model, SigmaGMR, MuGMR,leadFrames); %all frames
    r(4).Data(:,:)=temp.Data(1:2,:);
    
    temp=reproGMR(s(id), model, SigmaGMR, MuGMR,leadFrames(indx),indx);
    r(3).Data(:,:)=temp.Data(1:2,:);
    r(3).Sigma(:,:,:)=temp.Sigma(1:2,1:2,:);
    
    % calculate the cost, distance between ground truth and the
    % reproduced path with automatically chosen features
    SigmaP = zeros(2);
    MuP = zeros(2, 1);
    tempx=-(-cntr_im(1)+(r(1).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
    tempy=-(-cntr_im(2)+(r(1).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
    
    tempxorg=-(-cntr_im(1)+(r(3).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
    tempyorg=-(-cntr_im(2)+(r(3).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
    
    for t=1:model.nbData
        distance1 = pdist2(single([tempx(t);tempy(t)]'), single([tempxorg(t);tempyorg(t)]'));
        cost=cost+distance1;
    end
    cost = cost/size(MuGMR,2);
    
    % save the data for this iteration
    count = count+1;
    rl_data(iterNum,:) = [deltaRelev indx cost];
    reward = zeros(model.nbFrames,1);
    
    % Update for every period number of iterations
    if (count == period)
        count = 0;
        countReward = ones(1,model.nbFrames);
        sumExplore = zeros(1,model.nbFrames);
        % normalise cost
        cost_norm = normalize(rl_data(end-period+1:end,end),'range',[0,1]);
        % cost_norm = normalize(rl_data(:,end),'range',[0,1]); %normalise cost
        
        for j = iterNum-period+1:iterNum
            indx = rl_data(j,model.nbFrames+1:end-1);
            % calculate the reward
            reward(indx) = sigmf(-cost_norm(-iterNum+period+j),[5 0]);
            % reward(indx) = sigmf(-cost_norm(j),[5 0]);
            
            % saving the normalised rewards and costs for plotting
            saveCost(j) = cost_norm(-iterNum+period+j);
            saveReward(j) = reward(indx(1));
            sumExplore(indx) = sumExplore(indx)+rl_data(j,indx).*reward(indx)';
            countReward(indx) = countReward(indx)+1;
        end
        
        % update the relevancy vector
        for m=1:model.nbFrames
            Relev(m)=Relev(m)+sumExplore(m)/countReward(m);
        end
        Relev=Relev/sum(Relev);
        
        % plot the results
        [out,indx] = sort(Relev,'descend');
        indx = indx(1:poolsize);
        
        temp=reproGMR(s(id), model, SigmaGMR, MuGMR,leadFrames(indx),indx);
        r(2).Data(:,:)=temp.Data(1:2,:);
        
        %         figure(1);
        %         subplot(1,3,id); hold on;
        
        subplot(2,3,id);
        image=imread(strcat(imgs(id).folder,'/', imgs(id).name));imshow(image); hold on;
        %         lengthim = size(image,1);
        %         widthim = size(image,2);
        %         cntr_real=[cam_pos(1),cam_pos(2),cam_posrel(3)];
        %         cntr_im=[widthim/2,lengthim/2];
        %         length_real=2*cam_posrel(3)*tan(pi/6);
        %         width_real=length_real*widthim/lengthim;
        %         % plot frames
        %         for m=1:model.nbFrames
        %             plot(pTmp(m).b(2), pTmp(m).b(3), '.','markersize',30,'color',...
        %                 colPegs(min(21,m),:));
        %             plot([pTmp(m).b(2) pTmp(m).b(2)+8*pTmp(m).A(2,3)], [pTmp(m).b(3) pTmp(m).b(3)+8*pTmp(m).A(3,3)], 'MarkerSize',30,'color','black');
        %
        %             text(double(pTmp(m).b(2)), double(pTmp(m).b(3)), int2str(leadFrames(m)),'color','white','fontsize',20);
        %          end
        
        % plot track
        tempx=-(-cntr_im(1)+(r(1).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
        tempy=-(-cntr_im(2)+(r(1).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
        plot(tempx, tempy,'.','linewidth',2,'color','green');%groundtruth
        
        tempx=-(-cntr_im(1)+(r(2).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
        tempy=-(-cntr_im(2)+(r(2).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
        plot(tempx, tempy,'.','linewidth',1.5,'color','white');%relevant ring - after period
        
        tempx=-(-cntr_im(1)+(r(4).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
        tempy=-(-cntr_im(2)+(r(4).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
        %         plot(tempx, tempy,'-','linewidth',1,'color','red');%%all frames
        
        
        % plot winning frames
        for n=1:length(indx)
            m=leadFrames(indx(n));
            tempx=-(-cntr_im(1)+(s(id).p(m).b(3)-cntr_real(2)).*(widthim)./width_real);
            tempy=-(-cntr_im(2)+(s(id).p(m).b(2)-cntr_real(1)).*(lengthim)./length_real);
            
            if model.orientless(indx(n))==1
                plot(tempx, tempy,'.','markersize',30,'color','yellow');
%                 text(double(tempx), double(tempy), int2str(n),'color','yellow', "FontSize",15); %
            else
                plot(tempx, tempy,'.','markersize',30,'color','blue');
                %             text(double(tempx), double(tempy), int2str(mm),'color','blue', "FontSize",15);
            end
            %             plot(tempx, tempy,'.','markersize',30);
            plot([tempx tempx+s(id).p(m).A(2,3)], [tempy tempy+s(id).p(m).A(3,3)], '-','linewidth',6);
        end
         hold off;
    end
end

%% plot final result
figure;
image=imread(strcat(imgs(id).folder,'/', imgs(id).name));imshow(image);hold on;
for n=1:length(indx)
    m=leadFrames(indx(n));
    tempx=-(-cntr_im(1)+(s(id).p(m).b(3)-cntr_real(2)).*(widthim)./width_real);
    tempy=-(-cntr_im(2)+(s(id).p(m).b(2)-cntr_real(1)).*(lengthim)./length_real);
    
    plot(tempx, tempy,'.','markersize',30);
    plot([tempx tempx+s(id).p(m).A(2,3)], [tempy tempy+s(id).p(m).A(3,3)], '-','linewidth',6);
end
tempx=-(-cntr_im(1)+(r(1).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
tempy=-(-cntr_im(2)+(r(1).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
plot(tempx, tempy,'-','linewidth',2,'color','green');%groundtruth

tempx=-(-cntr_im(1)+(r(2).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
tempy=-(-cntr_im(2)+(r(2).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
plot(tempx, tempy,'.','linewidth',1.5,'color','white');%relevant ring - after period
figure;
image=imread(strcat(imgs(id).folder,'/', imgs(id).name));imshow(image);hold on;
for n=1:length(leadFrames)
    m=leadFrames(n);
    tempx=-(-cntr_im(1)+(s(id).p(m).b(3)-cntr_real(2)).*(widthim)./width_real);
    tempy=-(-cntr_im(2)+(s(id).p(m).b(2)-cntr_real(1)).*(lengthim)./length_real);
    
    plot(tempx, tempy,'.','markersize',30);
    plot([tempx tempx+s(id).p(m).A(2,3)], [tempy tempy+s(id).p(m).A(3,3)], '-','linewidth',6);
end
tempx=-(-cntr_im(1)+(r(1).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
tempy=-(-cntr_im(2)+(r(1).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
plot(tempx, tempy,'.','linewidth',2,'color','green');%groundtruth

tempx=-(-cntr_im(1)+(r(4).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
tempy=-(-cntr_im(2)+(r(4).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
plot(tempx, tempy,'.','linewidth',1,'color','red');%%all frames

%% assess
%distance between all leads and demo paths
cost_lead=0;cost_rel=0;
for t=1:model.nbData
    dist_all_lead = pdist2(single([r(1).Data(1:2,t)]'), single([r(4).Data(1:2,t)]'));
    dist_all_rel=pdist2(single([r(1).Data(1:2,t)]'), single([r(2).Data(1:2,t)]'));
    cost_lead=cost_lead+dist_all_lead;
    cost_rel=cost_rel+dist_all_rel;
end
cost_lead = cost_lead/size(MuGMR,2)
cost_rel = cost_rel/size(MuGMR,2)

dist_start_lead = pdist2(single([r(1).Data(1:2,1)]'), single([r(4).Data(1:2,1)]'))
dist_start_rel=pdist2(single([r(1).Data(1:2,1)]'), single([r(2).Data(1:2,1)]'))
dist_end_lead = pdist2(single([r(1).Data(1:2,end)]'), single([r(4).Data(1:2,end)]'))
dist_end_rel=pdist2(single([r(1).Data(1:2,end)]'), single([r(2).Data(1:2,end)]'))
%% show the cost function
aveLen=period; % average the cost vector over intervals to smoothen it
for i=1:iterNum/aveLen
    low=(i-1)*aveLen+1;
    high=i*aveLen;
    saveCost1(i)=mean(saveCost(low:high));
    saveReward1(i)=mean(saveReward(low:high));
end

figure(21);
plot(saveCost,'markersize',10,'color','red');
hold on;
plot(saveReward,'markersize',10,'color','green');
figure(22);
plot(saveCost1,'markersize',10,'color','red');
hold on;
plot(saveReward1,'markersize',10,'color','green');
Relev1=Relev;
save('mainRL_Varypoolsize_output','Relev1');

%% Try on Validation image
% this runs the end results on a validation image that hasn't been used for training

% image=imread(strcat(imgs(end).folder,'\', imgs(end).name));
% [out,indx]=sort(Relev,'descend');
% ii=1;
% INDX(ii,:)=indx;
%
% % need to fix this to fit purpose
% [cost,path]=GeneratePath(image, s(end) ,indx(1:poolsize), leadFrames, objs,MuGMR,SigmaGMR,model);
% Cost(ii)=cost;
%
% [minim,indx_poolsize]=min(Cost);
% indx=INDX(indx_poolsize,1:poolsize);
%
% save(strcat(task_path,"\result.mat"),'path','indx','poolsize');
indx=sort(indx);
save(strcat(task_path,'/parameters.mat'), 'leadFrames', 'objs','s', 'indx');
