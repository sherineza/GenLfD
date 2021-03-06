function [] = mainTPGMM (task_path)

%In this version, x y and real life
%positions so before plotting, we convert the points to pixels

if nargin==0
    clc; clear; close all; clear classes;
    %Adds all subfolders to path
    currentFolder=fileparts(mfilename('fullpath'));
    cd(currentFolder);
    addpath(genpath(currentFolder));
    %load images
    task_path = "tasks\Demo3";
end

%% Load Data
demo_files = [dir(fullfile(task_path,'\*.PNG')),...
    ;dir(fullfile(task_path,'\*.jpg'));
    dir(fullfile(task_path,'\*.JPEG'))];
load(strcat(task_path, '\parameters.mat'));
load(strcat(task_path, '\paths.mat'));
load(strcat(task_path, '\calib.mat'));
nbDemos = length(demo_files);

%combine paths into struct s
for i=1:nbDemos
    s(i).Data=paths(i).path([1:4],:);
end

%% Model Parameters
disp("Initializing Training Model...")
model.nbFrames =1;%length(leadFrames);
model.nbVar = length(s(1).Data(:,1));
model.dt = 1E-2;
model.params_diagRegFact = 1E-8;
model.nbData = length(s(1).Data);
model.nbRepros=0;
model.nbSamples=length(s);
model.nbStates=3;

model.orientless=[];
model.Mu_dohnut=[];
model.Sigma_dohnut=[];

%% Train
% for i=1:length(leadFrames)
%     [model] = trainTPGMM(model, leadFrames(i),s);
%     [SigmaGMR(:,:,:,i), MuGMR(:,:,i)] = getTPGMR(model,s);
% end
disp("Training Model...")
model.nbFrames=length(leadFrames);
[model] = trainTPGMM(model, leadFrames,s);
[SigmaGMR, MuGMR] = getTPGMR(model,s);
%%
for m=1:length(s(1).p) 
    x=[];
    %for each frame, find in objINDX
    [row,col]=find(objs==m);
    leadframes=(objs(1,col));
    for i=1:length(leadframes)
        x(i)=numel(find(leadframes(i)==objs(:,col)));
    end
    [dummy,id] = max(x);
    if length(find(leadframes==m))>0
        MuGMRtemp(:,:,m)=MuGMR(:,:, find(leadFrames==m));
        SigmaGMRtemp(:,:,:,m) = SigmaGMR(:,:,:,find(leadFrames==m));
    else
        MuGMRtemp(:,:,m) = MuGMR(:,:,col(id));
        SigmaGMRtemp(:,:,:,m) = SigmaGMR(:,:,:,col(id));
    end
end
MuGMR=MuGMRtemp; SigmaGMR=SigmaGMRtemp;

%% Reproduction
r = reproGMR(s, model, SigmaGMR, MuGMR, leadFrames);

%% Visualise
imgs = [dir(fullfile(task_path,'\*.PNG')),...
    ;dir(fullfile(task_path,'\*.jpg'));
    dir(fullfile(task_path,'\*.JPEG'))];
for ind=1:length(imgs)
    img=imgs(ind);
    figure(ind);
    I=imread(strcat(img.folder,'\', img.name));
    imshow(I);hold on;
    
    lengthim = size(I,1);
    widthim = size(I,2);
    cntr_real=[cam_pos(1),cam_pos(2),cam_posrel(3)];
    cntr_im=[widthim/2,lengthim/2];
    length_real=2*cam_posrel(3)*tan(pi/6);
    width_real=length_real*widthim/lengthim;

    %Plot frames
    for mm=1:length(leadFrames)
        m=leadFrames(mm);
        tempx=-(-cntr_im(1)+(s(ind).p(m).b(3)-cntr_real(2)).*(widthim)./width_real);
        tempy=-(-cntr_im(2)+(s(ind).p(m).b(2)-cntr_real(1)).*(lengthim)./length_real);
        
        plot(tempx, tempy,'.','markersize',30);
        plot([tempx tempx+s(ind).p(m).A(2,3)], [tempy tempy+s(ind).p(m).A(3,3)], '-','linewidth',6);
        if model.orientless(mm)==1
            text(double(tempx), double(tempy), int2str(mm),'color','yellow', "FontSize",15); %
        else
            text(double(tempx), double(tempy), int2str(mm),'color','white', "FontSize",15);
        end
    end
    %Plot trajectories    
    tempx=-(-cntr_im(1)+(s(ind).Data(3,:)-cntr_real(2)).*(widthim)./width_real);
    tempy=-(-cntr_im(2)+(s(ind).Data(2,:)-cntr_real(1)).*(lengthim)./length_real);
    plot(tempx, tempy,'-','linewidth',1.5,'color','green');
    
    tempx=-(-cntr_im(1)+(r(ind).Data(2,:)-cntr_real(2)).*(widthim)./width_real);
    tempy=-(-cntr_im(2)+(r(ind).Data(1,:)-cntr_real(1)).*(lengthim)./length_real);
    plot(tempx, tempy,'-','linewidth',1.5,'color','white');
    %Plot Gaussians
    %    plotGMM(r(ind).Data(1:2,1:10:end), r(ind).Sigma(1:2,1:2,1:10:end),.5);
    hold off;
end

%% Save data
save(strcat(task_path, '\model.mat'), 'model', 'MuGMR', 'SigmaGMR');

end
