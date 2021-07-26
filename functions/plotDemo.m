function plotDemo(img,s,leadFrames,model)
xx = round(linspace(1,64,4));
clrmap = colormap('jet');
clrmap = min(clrmap(xx,:),.95);
colPegs =[ [0, 0.4470, 0.7410];[0.3,0.4,0.5];[00.5,0.7,0.6];[0.1, 0.1, 1];[0.85, 0.32, 0.09];[0.1, 0.5, 0.1];[0.92, 0.69, 0.12]; [1, .01, 0.1];[0.49, 0.18, 0.50]; [0.1, 0.75, 0.75];[0.46, 0.67, 0.18]; [0.75, 0.1, 0.75];[0.30, 0.74, 0.93];[0.75, 0.75, 0.1];[0.63, 0.07, 0.18];[0.75, 0.05, 0.25]];

if nargin <3
    leadFrames=1:length(s.p);
end

figure;
I=imread(strcat(img.folder,'\', img.name));
imshow(I);hold on;

%Plot frames
for mm=1:length(leadFrames)
    
    m=leadFrames(mm);
    plot(s.p(m).b(2), s.p(m).b(3),'.','markersize',30,'color',colPegs(min(16,m),:));
    plot([s.p(m).b(2) s.p(m).b(2)+s.p(m).A(2,3)], [s.p(m).b(3) s.p(m).b(3)+s.p(m).A(3,3)], '-','linewidth',6,'color',colPegs(min(14,m),:));
    if model.orientless(mm)==1
%         plot([s.p(m).b(2) s.p(m).b(2)+35*s.p(m).A(2,3)], [s.p(m).b(3) s.p(m).b(3)+35*s.p(m).A(3,3)], 'linewidth',3,'color','white');
        text(double(s.p(m).b(2)), double(s.p(m).b(3)), int2str(mm),'color','yellow', "FontSize",15);
    else
%         plot([s.p(m).b(2) s.p(m).b(2)+35*s.p(m).A(2,3)], [s.p(m).b(3) s.p(m).b(3)+35*s.p(m).A(3,3)], 'linewidth',3,'color','yellow');
        text(double(s.p(m).b(2)), double(s.p(m).b(3)), int2str(mm),'color','white', "FontSize",15);
    end
end

%Plot trajectories
plot(s.Data(2,1), s.Data(3,1),'.','markersize',6);
plot(s.Data(2,:), s.Data(3,:),'-','linewidth',1.5,'color','green');

%Plot Gaussians
if nargin<3
    plotGMM(s.Data(2:3,1:10:end), s.Sigma(:,:,1:10:end),clrmap(1,:), .5);
end
    
hold off;

end

