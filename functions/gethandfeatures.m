function [P_hand] = gethandfeatures (images_path)
%In this function, only one hand per image is returned for simplicity. with
%the highest confidence.
clear classes 

P_hand=[];
if nargin < 1
   images_path= "/images";
end

%code to call python hand detector

pyenv;

% insert(py.sys.path, int32(0),'C:\PhD\GUIv1\venv\Lib\site-packages');
% insert(py.sys.path, int32(0),'C:\PhD\GUIv1\yolo-hand-detection');
% pyenv('Version','executable')

currentFolder=fileparts(fileparts(mfilename('fullpath')));
path_sitepackages= strcat(currentFolder, '/python-handdetector/venv/Lib/site-packages');
path_handdetector= strcat(currentFolder, '/python-handdetector/yolo-hand-detection');
insert(py.sys.path, int32(0),path_sitepackages);
insert(py.sys.path, int32(0),'/Users/shirineelzaatari/Library/Python/3.8/');
insert(py.sys.path, int32(0),path_handdetector);

mod=py.importlib.import_module('returnhand');
py.importlib.reload(mod);

system('python3 python-handdetector/yolo-hand-detection/returnhand.py')
% handlist=py.returnhand.main(images_path);

%convert python list to 3D array
temp=cell(handlist);
for i=1:length(handlist)%number of images
   tempinimage=cellfun(@double,cell(temp{i}));
   tempinimage=reshape(tempinimage,[5,length(tempinimage)/5]);
   tempinimage=tempinimage';
   [maxconf, idx]=max(tempinimage(:,5));
   cx = tempinimage(idx,1) + (tempinimage(idx,3) / 2);
   cy = tempinimage(idx,2) + (tempinimage(idx,4) / 2);
   if length(idx)~=0
       P_hand(i).b = [0;cx;cy];
       P_hand(i).A=[[1,0,0];[0,1, 0];[0,0,1]]; %keep the orientation
   end
end

end