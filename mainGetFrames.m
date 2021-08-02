function [] = mainGetFrames(task_path)
% In this version, the frames are given real life positions instead of
% pixel positions
currentFolder=fileparts(mfilename('fullpath'));

% Specify task path is function called with no inputs
if nargin==0
    clc; clear; close all;clear classes;
    %Adds all subfolders to path
    currentFolder=fileparts(mfilename('fullpath'));
    cd(currentFolder);
    addpath(genpath(currentFolder));
    %load images
    task_path = "tasks/batteryinbox";
end

load(strcat(task_path, '/calib.mat')); %loads camera calibration information

demo_files = [dir(fullfile(task_path,'/*.PNG')),...
    ;dir(fullfile(task_path,'/*.jpg'));
    dir(fullfile(task_path,'/*.JPEG'))];
nbDemos = size(demo_files,1);

%% Detect features
% detecting features happens in three steps
disp("Detecting SURF Features...")
[Demo, INDEX] = get_features(demo_files); % 1. detecting features in each demonstration image
disp("Matching Features across Demonstration Images...")
INDEX=  match_feat_demos(Demo, INDEX, 0); % 2. matching features between demonstration images
s = feat2frame(Demo,INDEX);% 3. converting data types to create required matrix b and A inputs for TP-GMM


thresh=0.05:0.01:0.2; %for redundant frames
disp("Group Redundant Features (Frames)...")
[leadFrames, objs] = group_redundant(s,thresh);

%% Detecting Hand features
disp("Detecting Hand Features...")
% P_hand = gethandfeatures(strcat(currentFolder,'\',task_path));
P_hand=[];
if length(P_hand)~=0
    for demoindx=1:nbDemos
        s(demoindx).p(end+1).b=P_hand(demoindx).b;
        s(demoindx).p(end).A=P_hand(demoindx).A;
        handfeature=length(s(1).p);
    end
    leadFrames=[leadFrames,length(s(1).p)];
    objs(1,end+1)=length(s(1).p);
else
    handfeature=0;
end


%% Plot Features
figure('WindowState','maximized');
xx = round(linspace(1,256,length(leadFrames)));
clrmap = colormap('jet');
colors = min(clrmap(xx,:),.95);
title(strcat('Grouped Frames in Demonstration Images.'));
for k=1:nbDemos
    image = imread(strcat(demo_files(k).folder,'/', demo_files(k).name));
    subplot(2,3,k);
    imshow(image);hold on;
    
    
    lengthim = size(image,1);
    widthim = size(image,2);
    
    for obj_group=1:length(leadFrames)
        for iter1=1:length(find(objs(:,obj_group)~=0))
            iter=objs(iter1,obj_group);
            plot(s(k).p(iter).b(2), s(k).p(iter).b(3),'.','markersize',30,'color',colors(obj_group,:));
            plot([s(k).p(iter).b(2) s(k).p(iter).b(2)+10*s(k).p(iter).A(2,3)], [s(k).p(iter).b(3) s(k).p(iter).b(3)+10*s(k).p(iter).A(3,3)], 'MarkerSize',25,'color','black');
        end
    end
    for obj_group=1:length(leadFrames)
        for iter1=1:length(find(objs(:,obj_group)~=0))
            iter=objs(iter1,obj_group);
            %             if iter1==1
            %                 text(double(s(k).p(iter).b(2)), double(s(k).p(iter).b(3)), int2str(iter),'color','white', "FontSize",15);
            %             end
            
        end
    end
    
    % Convert pixels to real life positions
    for obj_group=1:length(leadFrames)
        for iter1=1:length(find(objs(:,obj_group)~=0))
            iter=objs(iter1,obj_group);
            cntr_real=[cam_pos(1),cam_pos(2),cam_posrel(3)];
            cntr_im=[widthim/2,lengthim/2];
            length_real=2*cam_posrel(3)*tan(pi/6);
            width_real=length_real*widthim/lengthim;
            pt_x(iter)=length_real*(cntr_im(2)-s(k).p(iter).b(3))/lengthim+cntr_real(1);
            pt_y(iter)=width_real*(cntr_im(1)-s(k).p(iter).b(2))/widthim+cntr_real(2);
        end
    end
    
    for iter=1:length(s(k).p)
        s(k).p(iter).b(2)=pt_x(iter);
        s(k).p(iter).b(3)=pt_y(iter);
        temp=s(k).p(iter).A(2,:);
        s(k).p(iter).A(2,:)=s(k).p(iter).A(3,:);
        s(k).p(iter).A(3,:)=temp;
        s(k).p(iter).A(:,3)=s(k).p(iter).A(:,3).*-1;
    end
end

%% Save Data
features = adv_package_features(Demo,leadFrames,INDEX);
save(strcat(task_path, '/features.mat'), 'features','handfeature','leadFrames','objs','s');

mainTPGMM(task_path);
indx = findIrrelevant(task_path);

% %update parameter values for final retrain and reproduce
% leadFrames = leadFrames(indx);
% for i=1:length(leadFrames)
%     tempobjs(:,i)=objs(:,find(objs(1,:)==leadFrames(i)));
% end
% objs=tempobjs;
%

% mainTPGMM1(task_path);

end
