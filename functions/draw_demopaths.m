function [finalX,finalY] = draw_demopaths(nbData,image)
figure;
imshow(image);hold on;
set(gcf, 'Position', get(0,'Screensize'));
hFH = imfreehand();
xy = hFH.getPosition; x = xy(:,1); y = xy(:,2);
delete(hFH);
hold on;
xCoordinates =  xy(:, 1);
yCoordinates = xy(:, 2);
plot(xCoordinates, yCoordinates,  'MarkerSize', 10,'color','green');
xCoordinates = xy(:, 1);
yCoordinates = xy(:, 2);
numberOfKnots =length(xCoordinates);
samplingRateIncrease = 200/numberOfKnots;
newXSamplePoints = linspace(1, numberOfKnots, numberOfKnots * samplingRateIncrease);
yy = [0, xCoordinates', 0; 1, yCoordinates', 1];
pp = spline(1:numberOfKnots, yy);
smoothedY = ppval(pp, newXSamplePoints);
smoothedXCoordinates = smoothedY(1, :);
smoothedYCoordinates = smoothedY(2, :);
%% Trying to make points equidistant
% close the contour, temporarily
x=smoothedXCoordinates;
y=smoothedYCoordinates;
xc = [x(:); x(1)];
yc = [y(:); y(1)];
xc = [x(:)];
yc = [y(:)];
% current spacing may not be equally spaced
dx = diff(xc);
dy = diff(yc);
% distances between consecutive coordinates
dS = sqrt(dx.^2+dy.^2);
dS = [0; dS];     % including start point
% arc length, going along (around) snake
d = cumsum(dS);  % here is your independent variable
perim = d(end);
N = 200;
ds = perim / N;
dSi = ds*(0:N).'; %' your NEW independent variable, equally spaced
dSi(end) = dSi(end)-.005; % appease interp1
xi = interp1(d,xc,dSi);
yi = interp1(d,yc,dSi);
xi(end)=[]; yi(end)=[];
smoothedXCoordinates=xi';smoothedYCoordinates=yi';
hGreenCurve = plot(smoothedXCoordinates, smoothedYCoordinates, '-g','MarkerSize', 30);
plot(smoothedXCoordinates, smoothedYCoordinates,'o');
title('Spline Interpolation Demo', 'FontSize', 20);
intSmoothedXCoordinates = double(int32(smoothedXCoordinates));
intSmoothedYCoordinates = double(int32(smoothedYCoordinates));
diffX = [1, diff(intSmoothedXCoordinates)];
diffY = [1, diff(intSmoothedYCoordinates)];
bothZero = (diffX==0) & (diffY == 0);
finalX = intSmoothedXCoordinates(:);
finalY = intSmoothedYCoordinates(:);
delete(hGreenCurve);
hGreenCurve = plot(finalX, finalY, '-g','MarkerSize', 30);
hold off;
end