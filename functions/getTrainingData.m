function getTrainingData (task)

% Initialise
currentFolder='C:\PhD\taskParaOptimize-master';
cd(currentFolder);
addpath(genpath(currentFolder));

if (narg<1) task='Demo2'; end
task_path = strcat('tasks\',task);
boolDisp = false;

%% Get Training Data
% Get Features from Demo Images
% The frames of references (task parameters) are detected as visual features from the images of the demonstrations. 
demo_files = dir(fullfile(task_path,'\*.jpg'));
nbDemos = length(demo_files);

if isempty(demo_files)
    error('No images were found in the folder.')
else
    [Demo, INDEX] = get_features(demo_files);
    
    if boolDisp
        for i=1:length(Demo)
            figure; imshow(Demo(i).image); hold on;
            plot(Demo(i).corners); hold off;
            title(strcat('Demo Image ',int2str(i)));
        end
    end
end

%% Match Features between Demo Images
% This filters through the features obtained such that only the ones present across demo images remain.
INDEX = match_feat_demos(Demo, INDEX, boolDisp);


s=feat2frame(Demo,INDEX);

if boolDisp
    for k=1:nbDemos
        figure(k);
        imshow(Demo(k).image);hold on;
        title(strcat('Image ', int2str(k)));
        for iter=1:length(s(k).p)
            plot(s(k).p(iter).b(2), s(k).p(iter).b(3),'.','markersize',20,'color','blue');
        end
    end
end

%% Identify Redundant frames
% This filters through the features obtained to group them into "objects" and identify redundant ones. The reason why we identify redundant frames at this stage is to decrease the number of frames and allow the training of the TP-LfD.
thresh=0.05;
[leadFrames, objs] = group_redundant(s,thresh);

if boolDisp
    for k=1:nbDemos
        figure(k);
        imshow(Demo(k).image);hold on;
        title(strcat('Image ', int2str(k)));
        for iter=1:length(s(k).p)
            plot(s(k).p(iter).b(2), s(k).p(iter).b(3),'.','markersize',20,'color','blue');
            if find(leadFrames==iter)~=0
                plot(s(k).p(iter).b(2), s(k).p(iter).b(3),'.','markersize',10,'color','green');
            end
        end
    end
end

%% Draw Demonstration Paths
nbData=200;

for i=1:nbDemos
    [finalX,finalY]=draw_demopath(nbData,Demo(i));
    time=1/nbData:1/nbData:1;
    Data(:,:,i)=[time;finalX';finalY'];
    s(i).Data0=double([time;finalX';finalY']);
    s(i).nbData=nbData;
    s(i).Data=double([time;finalX';finalY']);
end

%% Save Training Data
save(strcat(task_path, '\Data.mat'), 's', 'Data','leadFrames', 'objs');

end