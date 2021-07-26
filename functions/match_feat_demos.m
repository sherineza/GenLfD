function [INDEX] = match_feat_demos(Demo, INDEX, boolDisp)
nbSamples=length(Demo);
for i=1:nbSamples-1
    for j=i+1:nbSamples
        boxPairs = matchFeatures(Demo(j).Features, Demo(i).Features,...
            'Unique',true, 'MatchThreshold', 100, 'MaxRatio', 0.7); %match features between two images
        matchedBoxPoints = Demo(j).corners(boxPairs(:, 1), :);
        matchedScenePoints = Demo(i).corners(boxPairs(:, 2), :);
        INDEX(j,boxPairs(:,2),i)=boxPairs(:,1);
        INDEX(i,boxPairs(:,1),j)=boxPairs(:,2);
        
        for k=1:i-1
            % checking for matches "by substitution". e.g. if feature 1 is
            % demo A is matched with feature 2 and 3 from demo B and C
            % respectively, then feature 2 and 3 from B and C are set as
            % matched as well.
            for n=1:length(Demo(i).corners)
                if INDEX(k,n,i)~=0 && INDEX(j,n,i)~=0
                    if INDEX(k,INDEX(j,n,i),j)==0
                        INDEX(k,INDEX(j,n,i),j)=INDEX(k,n,i);
                    end
                end
            end
        end
        
        if boolDisp %display the feature matches on the live script
            figure;
            showMatchedFeatures(Demo(j).image, Demo(i).image,...
                matchedBoxPoints, matchedScenePoints, 'montage')
            title(strcat('Image',int2str(j),' and Image', int2str(i)))
        end
    end
end
end

