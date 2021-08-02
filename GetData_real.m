clear all;clc;close all; % clearing previous data
% adding subfolders to matlab path
currentFolder=fileparts(mfilename('fullpath'));
cd(currentFolder);
addpath(genpath(currentFolder));
task_path = "tasks/batteryinbox";

demo_files = [dir(fullfile(task_path,'/*.PNG')),...
    ;dir(fullfile(task_path,'/*.jpg'));
    dir(fullfile(task_path,'/*.JPEG'))];
nbDemos = size(demo_files,1);

if isempty(demo_files)
    error('No images were found in the folder.')
end

%% calibrate
if isfile(strcat(task_path, '/calib.mat'))
    % File exists.
    load(strcat(task_path, '/calib.mat'));
else
    % File does not exist. Copy default one and paste it in the task
    % folder. 
    copyfile("tasks/calib.mat",task_path);
    pause(2);
    load(strcat(task_path, '/calib.mat'));
end

[lengthim, widthim,d]= size(imread(strcat(demo_files(1).folder,'/', demo_files(1).name)));
cntr_real=[cam_pos(1),cam_pos(2),cam_posrel(3)];
cntr_im=[widthim/2,lengthim/2];
length_real=2*cam_posrel(3)*tan(pi/6);
width_real=length_real*widthim/lengthim;

%% draw paths

nbData=200; %number of points on the path
% this prompts the user to draw the demonstration path on each of the
% demonstration images and outputs the path as a vector
for i=1:nbDemos
    filename= strcat(demo_files(i).folder,'/', demo_files(i).name);
    [finalX,finalY]=draw_demopath(nbData,filename);
    time=1/nbData:1/nbData:1;

    %convert paths from pixel values to real life values
    pt_x=length_real.*(cntr_im(2)-finalY')./lengthim+cntr_real(1);
    pt_y=width_real.*(cntr_im(1)-finalX')./widthim+cntr_real(2);
    paths(i).path=double([time;pt_x;pt_y]);
end

% save path data
save(strcat(task_path, '/paths.mat'), 'paths');