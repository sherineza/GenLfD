function [] = mainGetFrames(task_path)

if nargin==0
    clc; clear; close all; clear classes;
    %Adds all subfolders to path
    currentFolder=fileparts(mfilename('fullpath'));
    cd(currentFolder);
    addpath(genpath(currentFolder));
    %load images
    task_path = "tasks\cubeinbox1";
end

demo_files = [dir(fullfile(task_path,'\*.PNG')),...
    ;dir(fullfile(task_path,'\*.jpg'));
    dir(fullfile(task_path,'\*.JPEG'))];
nbDemos = length(demo_files);

%% Detect features
disp("Detecting SURF Features...")
[Demo, INDEX] = get_features(demo_files);
disp("Matching Features across Demonstration images...")
INDEX=  match_feat_demos(Demo, INDEX, 0);
s = feat2frame(Demo,INDEX);
thresh=0.05:0.01:0.2; %for redundant frames
disp("Grouping Redundant Features Together...")
[leadFrames, objs] = group_redundant(s,thresh);%TODO needs fixing

%Hand features
disp("Detecting Hand Features...")
P_hand = gethandfeatures(task_path);
if length(P_hand)~=0
    for demoindx=1:nbDemos
        s(demoindx).p(end+1).b=P_hand(demoindx).b;
        s(demoindx).p(end).A=P_hand(demoindx).A;
    end
    leadFrames=[leadFrames,length(s(1).p)];
    objs(1,end+1)=length(s(1).p);
end

%% Plot Features
xx = round(linspace(1,256,length(leadFrames)));
clrmap = colormap('jet');
colors = min(clrmap(xx,:),.95);figure('WindowState','maximized');
for k=1:nbDemos
    image = imread(strcat(demo_files(k).folder,'\', demo_files(k).name));
    subplot(2,3,k);imshow(image);hold on;
    title(strcat('Grouped Frames in Image ', int2str(k)));
    
    for obj_group=1:length(leadFrames)
        for iter1=1:length(find(objs(:,obj_group)~=0))
            iter=objs(iter1,obj_group);
            plot(s(k).p(iter).b(2), s(k).p(iter).b(3),'.','markersize',20,'color',colors(obj_group,:));
            plot([s(k).p(iter).b(2) s(k).p(iter).b(2)+8*s(k).p(iter).A(2,3)], [s(k).p(iter).b(3) s(k).p(iter).b(3)+8*s(k).p(iter).A(3,3)], 'MarkerSize',20,'color','black');
            if iter1==1
                text(double(s(k).p(iter).b(2)), double(s(k).p(iter).b(3)), int2str(iter),'color','white', "FontSize",15);
            end
        end
    end
end

%% Save Data
save(strcat(task_path,'\parameters.mat'), 'leadFrames', 'objs','s');

%% Train
mainTPGMM(task_path);

end


