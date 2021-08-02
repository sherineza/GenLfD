clear all;clc;close all; % clearing previous data
% adding subfolders to matlab path
currentFolder=fileparts(mfilename('fullpath'));
cd(currentFolder);
addpath(genpath(currentFolder));
task_path = "tasks/sortobjs";

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
id=vrep.simxStart('127.0.0.1',19997,true,true,1000,5); % connect simulation to matlab
if id < 0
    system(strcat(task_path,"/RobotScene.ttt")); % open simulation software if not already open
end
while true
    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5); % connect simulation to matlab
    if id >= 0
        returnCode=vrep.simxStartSimulation(id,vrep.simx_opmode_oneshot); % start simulation
        break;
    end
end

% get object handles from simulation
[camerro,vision1] = vrep.simxGetObjectHandle(id,'Vision_sensor0',...
    vrep.simx_opmode_oneshot_wait); % the camera to read images from
[camerro,target] = vrep.simxGetObjectHandle(id,'Target',...
    vrep.simx_opmode_oneshot_wait);% the cobot's end effector
[camerro,UR3_base] = vrep.simxGetObjectHandle(id,'UR3',...
    vrep.simx_opmode_oneshot_wait);%the robot's base position
[camerro,cube] = vrep.simxGetObjectHandle(id,'obj1',...
    vrep.simx_opmode_oneshot_wait);%the robot's base position
[camerro,box] = vrep.simxGetObjectHandle(id,'obj2',...
    vrep.simx_opmode_oneshot_wait);%the robot's base position

% As part of setting up, the operator is required to input the position of the camera as well as the cobot's base, with respect to any global frame of reference. The global frame of reference can be arbitrarily chosen as long as it is consistent.
% This is important to calibrate the cobot and map the path points from camera pixels to real-life coordinates.
% Here, the positions of the camera and the cobot's base are read automatically from the software.

%get position of fixed items
[returnCode, cam_pos]=vrep.simxGetObjectPosition(id,vision1,-1,...
    vrep.simx_opmode_oneshot_wait);
[returnCode, cam_posrel]=vrep.simxGetObjectPosition(id,vision1,UR3_base,...
    vrep.simx_opmode_oneshot_wait);
[returnCode, target_init_pos]=vrep.simxGetObjectPosition(id,target,-1,...
    vrep.simx_opmode_oneshot_wait);
[returnCode, cube_init_pos]=vrep.simxGetObjectPosition(id,cube,-1,...
    vrep.simx_opmode_oneshot_wait);
[returnCode, cube_init_or]=vrep.simxGetObjectOrientation(id,cube,-1,...
    vrep.simx_opmode_oneshot_wait);
[returnCode, box_init_pos]=vrep.simxGetObjectPosition(id,box,-1,...
    vrep.simx_opmode_oneshot_wait);
[returnCode, box_init_or]=vrep.simxGetObjectOrientation(id,box,-1,...
    vrep.simx_opmode_oneshot_wait);

% Demonstration Recording
% Between demonstrations, the operator is required to vary positions of objects in the scene as expected to happen in future scenarios. i.e. if an object is expected to remain fixed all the time, e.g. fixture or table, to need to vary it. However, if an  object is expected to change positions in future scenarios, e.g. conveyor belt surface or parts, then the operator should vary its position between demonstrations.
nbDemosplan=6; % number of demonstrations to be recorded. default:4
nbData=200;
for i=1:nbDemosplan
    Data=[];
    Data0=[];
    % Instruction to move objects in simulation scene
    uiwait(msgbox({strcat("Record Image of Demonstration ", int2str(i)," of ", int2str(nbDemosplan),"?")},'GenLfD','modal'));
    
    % Image
    % read image
    while true
        [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(id,vision1,0,vrep.simx_opmode_oneshot_wait);
        figure(i);imshow(image);
        trigger = questdlg('Are you satisfied with the image?', 'GenLfD', 'Yes, continue','No, repeat','Quit', 'Quit');
        switch trigger 
            case  'Yes, continue'
                break;
            case 'Quit'
                return;
        end
    end
    imwrite(image,strcat(task_path, '/img', int2str(i), '.jpg'));
    
    % 3D points
    % choose point on the path
    counter=1;
    while true
        trigger = questdlg('Record a path point?', 'GenLfD', 'Yes','No','Quit', 'Quit');
        switch trigger
            case  'Yes'
                figure(i); imshow(image);hold on; 
                title('Click on the image to specify the 2D projection of your path point.');
                [xi, yi, but] = ginput(1);
                % convert point from pixel to VREP position
                resolut=single(resolut);
                cntr_real=[cam_pos(1),cam_pos(2),cam_posrel(3)];
                cntr_im=[resolut(1)/2,resolut(2)/2];
                length_real=2*cam_posrel(3)*tan(pi/6);
                width_real=length_real*resolut(1)/resolut(2);
                pt_x=length_real*(cntr_im(2)-yi)/resolut(2)+cntr_real(1);
                pt_y=width_real*(cntr_im(1)-xi)/resolut(1)+cntr_real(2);
                point=[pt_x,pt_y,+0.77];
                
                % send point to VREP and trigger robot to be where the point is
                returnval=vrep.simxSetObjectPosition(id, target, -1, point,...
                    vrep.simx_opmode_oneshot_wait);
                uiwait(msgbox({'Finished adjusting height of gripper?'},'GenLfD','modal'));
                [returnval, targetpos] =vrep.simxGetObjectPosition(id, target, -1, ...
                    vrep.simx_opmode_oneshot_wait);
                [returnval, targetor] =vrep.simxGetObjectOrientation(id, target, -1, ...
                    vrep.simx_opmode_oneshot_wait);
                Data(counter,:)=[targetpos, targetor];
                
                % Gripper action
                while true
                    trigger = questdlg('At this point, would you like the cobot to have a closed gripper or opened gripper?', 'GenLfD', 'Open','Close','Quit','Quit');
                    switch trigger
                        case  'Open'
                            grip(counter)=0;
                            returnval=vrep.simxSetIntegerSignal(id,'gripperclose',grip(counter),vrep.simx_opmode_oneshot_wait);
                            break;
                        case 'Close'
                            grip(counter)=1;
                            returnval=vrep.simxSetIntegerSignal(id,'gripperclose',grip(counter),vrep.simx_opmode_oneshot_wait);
                            break;
                        case 'Quit'
                            return;
                    end
                end
                counter=counter+1;
            case 'No'
                if counter==1
                    f = errordlg('At least one path point should be recorded.')
                    return;
                end
                break;
            case 'Quit'
                return;
        end
    end
    
    % fill in the blank of path points  
    % calculating the nb of points between each 2 points
    disttot=0;
    ratio=[];dif=[];
    for ind=1:length(Data(:,1))-1      
        pt1= [Data(ind,1),Data(ind,2),Data(ind,3)];
        pt2= [Data(ind+1,1),Data(ind+1,2),Data(ind+1,3)];
        dif(ind,:)=pt2-pt1;
        dist=pdist([pt1;pt2],'euclidean');
        ratio=[ratio,dist];
        disttot= dist+disttot;
    end
    ratio=ratio/disttot;
    
    % filling in gaps
    temp=1;
    for ind=1:size(Data,1)-1
        Data0(temp,1:6)=Data(ind,1:6)
        Data0(temp,7)=grip(ind);
%         [returnCode,dummyHandle]=vrep.simxCreateDummy(id,0.008,[[1,0,0];[0,0,0];[1,0,0];[1,0,0]], vrep.simx_opmode_oneshot_wait);
%         [returnval] =vrep.simxSetObjectPosition(id, dummyHandle, -1, Data0(temp,1:3),vrep.simx_opmode_oneshot_wait);
       
        temp=temp+1;
        while temp<(sum(ratio(1:ind))*nbData)
            Data0(temp,1:3)=Data0(temp-1,1:3)+dif(ind,:)./(nbData*ratio(ind));
            Data0(temp,4:6)=Data0(temp-1,4:6)+(Data(ind,4:6)-Data(ind+1,4:6))./(nbData*ratio(ind));
            Data0(temp,7)=grip(ind);
%             [returnCode,dummyHandle]=vrep.simxCreateDummy(id,0.004,[[0,0,0];[0,0,0];[0,0,0];[0,0,0]], vrep.simx_opmode_oneshot_wait);
%             [returnval] =vrep.simxSetObjectPosition(id, dummyHandle, -1, Data0(temp,1:3),vrep.simx_opmode_oneshot_wait);
            temp=temp+1;
        end
    end
    Data0(temp,1:6)=Data(end,:)
    Data0(temp,7)=grip(end);
%     [returnCode,dummyHandle]=vrep.simxCreateDummy(id,0.008,[[1,0,0];[0,0,0];[1,0,0];[1,0,0]], vrep.simx_opmode_oneshot_wait);
%     [returnval] =vrep.simxSetObjectPosition(id, dummyHandle, -1, Data0(temp,1:3),vrep.simx_opmode_oneshot_wait);
    
%     % convert 0-1 of gripper state to continuous integral - NOT ENTIRELY
%     NECESSARY
%     starti=0; % assume default start and end open gripper
%     endi=1; % assume gripper closes after point 1
%     state=0;
%     for t=1:nbData
%         if Data0(t,7)=state
%             endi=endi+1;
%         else
%             for temp=starti:endi
%                 if state==0
%                     Data0(temp,7)=temp/(endi-starti);
%                 else
%                     Data0(temp,7)=1-temp/(endi-starti);
%                 end
%             end
%             %switch state
%             if state==0
%                 state=1;
%             else
%                 state=0;
%             end
%         end
%     end
    
    returnval=vrep.simxSetObjectPosition(id, target, -1, target_init_pos,...
        vrep.simx_opmode_oneshot_wait);
    returnval=vrep.simxSetObjectPosition(id, cube, -1, cube_init_pos,...
        vrep.simx_opmode_oneshot_wait);
    returnval=vrep.simxSetObjectOrientation(id, cube, -1, cube_init_or,...
        vrep.simx_opmode_oneshot_wait);
    returnval=vrep.simxSetObjectPosition(id, box, -1, box_init_pos,...
        vrep.simx_opmode_oneshot_wait);
    returnval=vrep.simxSetObjectOrientation(id, box, -1, box_init_or,...
        vrep.simx_opmode_oneshot_wait);
    
            
            % Visualise the path recorded in 3D

%     for ind=1:nbData
%         [returnval] =vrep.simxSetObjectPosition(id, target, -1, Data0(ind,1:3),...
%             vrep.simx_opmode_oneshot_wait);
%         [returnval] =vrep.simxSetObjectOrientation(id, target, -1, Data0(ind,4:6),...
%             vrep.simx_opmode_oneshot_wait);
%         if Data0(ind,7)==1
%             returnval=vrep.simxSetIntegerSignal(id,'gripperclose',1,vrep.simx_opmode_oneshot_wait);
%         elseif Data0(ind,7)==0
%             returnval=vrep.simxSetIntegerSignal(id,'gripperclose',0,vrep.simx_opmode_oneshot_wait);
%         end
%     end

     %%
    % Visualise the path on the image
    %converting coordinates to pixels
    tempx=-(-cntr_im(1)+(Data0(:,2)-cntr_real(2)).*(resolut(1))./width_real);
    tempy=-(-cntr_im(2)+(Data0(:,1)-cntr_real(1)).*(resolut(2))./length_real);
%     Data0(:,1:2)=[tempx,tempy];
    figure(i); hGreenCurve = plot(tempx, tempy, '-g','MarkerSize', 50);
    
    % add timesteps 
    if length(Data0(:,1))==200
        Data0(:,2:8)=Data0(:,1:7);
        Data0(:,1)=[2/nbData:2/nbData:2]';
    else
        Data0(1:200,2:8)=Data0(2:201,1:7);
        Data0(201,:)=[];
        Data0(:,1)=[2/nbData:2/nbData:2]';
    end
    paths(i).path=Data0'; %path is nbVar x nbData 
end
save(strcat(task_path, '/paths.mat'), 'paths')
save(strcat(task_path, '/calib.mat'), 'cam_pos', 'cam_posrel')

disp("Demonstration Successfully Recorded. Now begins the Automatic Training.  ")
mainGetFrames(task_path);
disp("Training Completed. Use TestRobot.mlx live script to reproduce paths in new scenarios. ")