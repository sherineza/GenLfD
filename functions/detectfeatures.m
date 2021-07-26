function [points] = detectfeatures(img)
points = detectSURFFeatures(img,'MetricThreshold',500);

end

