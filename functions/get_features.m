function [feat, feat_mat] = get_features(image_files)

feat=struct();
nbSamples = length(image_files);

for i=1:nbSamples
    %read image
    feat(i).filename= strcat(image_files(i).folder,'/', image_files(i).name);
    feat(i).image = rgb2gray(imread(feat(i).filename));
    %detect SURF points
    feat(i).corners = detectfeatures(feat(i).image);
    %extract features from SURF points
    [feat(i).Features, feat(i).corners] = ...
        extractFeatures(feat(i).image, feat(i).corners);
    %fill the feature matrix that contains the indeces of the features of all the demonstrations
    feat_mat(i,1:length(feat(i).corners),i)=1:length(feat(i).corners);
end

end

