function [Cost,path]= GeneratePath(image, s, indx,leadFrames, objs, MuGMR, SigmaGMR, model)
clrmap = colormap('jet');
xx = round(linspace(1,64,1));
clrmap = min(clrmap(xx,:),.95);
colPegs =[[0.2,0.4,0.5];[0.8,0.8,0.1];[0.2,0.2,0.9];[0.4,0.9,0.7];[0.3,0.4,0.1];[0.9,0.4,0.6];[0.3,0.4,0.1]; [0.1, 0.1, 1]...
    ;[0,0,0];[0.85, 0.32, 0.09];[0.1, 0.5, 0.1];[0.92, 0.69, 0.12]; [1, .01, 0.1];[0.49, 0.18, 0.50]; ...
    [0.1, 0.75, 0.75];[0.46, 0.67, 0.18]; [0.75, 0.1, 0.75];[0.30, 0.74, 0.93];[0.75, 0.75, 0.1];[0.63, 0.07, 0.18];[0.25, 0.25, 0.1]];
colors =[[0.2,0.4,0.5];[0.8,0.8,0.1];[0.2,0.2,0.9];[0.4,0.9,0.7];[0.3,0.4,0.1];[0.9,0.4,0.6];[0.3,0.4,0.1]; [0.1, 0.1, 1]...
    ;[0,0,0];[0.85, 0.32, 0.09];[0.1, 0.5, 0.1];[0.92, 0.69, 0.12]; [1, .01, 0.1];[0.49, 0.18, 0.50]; ...
    [0.1, 0.75, 0.75];[0.46, 0.67, 0.18]; [0.75, 0.1, 0.75];[0.30, 0.74, 0.93];[0.75, 0.75, 0.1];[0.63, 0.07, 0.18];[0.25, 0.25, 0.25]];

MuTmp = zeros(model.nbVar-1, model.nbData, model.nbFrames);
SigmaTmp = zeros(model.nbVar-1, model.nbVar-1, model.nbData, model.nbFrames);

%get GMR in main frame
for m=1:length(leadFrames)
    n=leadFrames(m);
    MuTmp(1:2,:,m) = s.p(n).A(2:end,2:end) * MuGMR(1:2,:,n) + repmat(s.p(n).b(2:end),1,length(s.Data));
    for t=1:model.nbData
        SigmaTmp(1:2,1:2,t,m) = s.p(n).A(2:end,2:end) * SigmaGMR(1:2,1:2,t,n) * s.p(n).A(2:end,2:end)';
    end
    if model.nbVar>3
        MuTmp(3:end,:,m) =  MuGMR(3:end,:,n) ;
        for t=1:model.nbData
            SigmaTmp(3:end,3:end,t,m) = SigmaGMR(3:end,3:end,t,(n)) ;
        end
    end
end

%FINDING THE CLOSEST RING POINT TO THE OTHER GAUSSIANS
for t=1:model.nbData
    for m=1:length(indx)
        if model.orientless(indx(m))==1
            closestid=0;%initialise
            closestd=100000;
            for kk=1:length(indx)
                k=indx(kk);
                if model.orientless(k)==0
                    d=sqrt((MuTmp(1,t,k)-s.p(leadFrames(indx(m))).b(2))^2+(MuTmp(2,t,k)-s.p(leadFrames(indx(m))).b(3))^2)-model.Mu_dohnut(t,indx(m))
                    if closestd>d
                        closestd=d;
                        closestid=k;
                    end
                else
                    if k~=indx(m)
                        d=sqrt((s.p(leadFrames(k)).b(2)-s.p(leadFrames(indx(m))).b(2))^2+(s.p(leadFrames(k)).b(3)-s.p(leadFrames(indx(m))).b(3))^2)-model.Mu_dohnut(t,indx(m))
                        if closestd>abs(d)
                            closestd=d;
                            closestid=k;
                        end
                    end
                end
            end
            %find the coordinates of the actual point closest
            %                 MuTmp2(:,t,indx(m))=(closestd/(closestd+model.Mu_dohnut(t,indx(m)))).*(pTmp(indx(m)).b(2:3)-MuTmp(:,t,closestid))+MuTmp(:,t,closestid);
            %                 SigmaTmp2(:,:,t,indx(m))=[model.Sigma_dohnut(t,indx(m)),0;0,model.Sigma_dohnut(t,indx(m))];
            MuTmp2(1:2,t,indx(m))=(closestd/(closestd+model.Mu_dohnut(t,indx(m)))).*(s.p(leadFrames(indx(m))).b(2:3)-MuTmp(1:2,t,closestid))+MuTmp(1:2,t,closestid);
            SigmaTmp2(1:2,1:2,t,indx(m))=[model.Sigma_dohnut(t,indx(m)),0;0,model.Sigma_dohnut(t,indx(m))];
            if model.nbVar>3
                MuTmp2(3:model.nbVar-1,t,indx(m))=MuTmp(3:end,t,closestid);
                SigmaTmp2(3:model.nbVar-1,3:model.nbVar-1,t,indx(m))= SigmaTmp(3:end,3:end,t,closestid);
            end
        else
            MuTmp2(:,t,indx(m))=MuTmp(:,t,indx(m));
            SigmaTmp2(:,:,t,indx(m))=SigmaTmp(:,:,t,indx(m));
        end
    end
end

Cost=0;
%product of Gaussians
for t=1:model.nbData
    SigmaP = zeros(model.nbVar-1);
    MuP = zeros(model.nbVar-1, 1);
    for m=1:length(indx)
        SigmaP = SigmaP + inv(SigmaTmp(:,:,t,(indx(m))));
        MuP = MuP + SigmaTmp(:,:,t,(indx(m)))\ MuTmp(:,t,(indx(m)));
    end
    r(1).Sigma(:,:,t) = inv(SigmaP);
    r(1).Data(:,t) = r(1).Sigma(:,:,t) * MuP;
    
    r(2).Data(:,:)=s.Data(2:3,t); %ground truth
      
    SigmaP = zeros(model.nbVar-1);
    MuP = zeros(model.nbVar-1, 1);
    for m=1:length(leadFrames)
        SigmaP = SigmaP + inv(SigmaTmp(:,:,t,m));
        MuP = MuP + SigmaTmp(:,:,t,m)\ MuTmp(:,t,m);
    end
    r(3).Sigma(:,:,t) = inv(SigmaP);
    r(3).Data(:,t) = r(3).Sigma(:,:,t) * MuP;
    
    %ring Gaussians
    SigmaP = zeros(model.nbVar-1);
    MuP = zeros(model.nbVar-1, 1);
    for m=1:length(indx)
        SigmaP = SigmaP + inv(SigmaTmp2(:,:,t,indx(m)));
        MuP = MuP + SigmaTmp2(:,:,t,indx(m))\ MuTmp2(:,t,indx(m));
    end
    r(5).Sigma(:,:,t) = inv(SigmaP);
    r(5).Data(:,t) = r(5).Sigma(:,:,t) * MuP;
    
    distance = pdist2(r(1).Data(:,t)', r(2).Data(:,t)'); %ground truth - TP-LfD relevant only
    Cost=Cost+distance;
end
Cost=Cost/s.nbData;

figure;imshow(image);hold on;
% %plot frames
% for m=1:length(leadFrames)
%     n=leadFrames(m)
%     plot(s.p(n).b(2), s.p(n).b(3), '.','markersize',30,'color',...
%         colPegs(min(21,m),:));
%     plot([s.p(n).b(2) s.p(n).b(2)+8*s.p(n).A(2,3)], [s.p(n).b(3) s.p(n).b(3)+8*s.p(n).A(3,3)], 'MarkerSize',15,'color','black');
%     
%     text(double(s.p(n).b(2)), double(s.p(n).b(3)), int2str(n),'color','white','fontsize',20);
% end
% plot winning frames

plotGMM(r(5).Data(1:2,1:10:end), r(5).Sigma(1:2,1:2,1:10:end),clrmap(1,:), .5);

%plot track
plot(r(2).Data(2,:),r(2).Data(1,:), 'color','green','linewidth',2);%ground truth
plot(r(3).Data(2,:),r(3).Data(1,:), 'color','red','linewidth',1.0);%all frames
plot(r(5).Data(2,:),r(5).Data(1,:), 'color','white','linewidth',2.0);%ring gaussians
% plot(r(1).Data(1,:),r(1).Data(2,:),'color','red','linewidth',2)
path=r(1);

for n=1:length(indx)
    m=leadFrames(indx(n));
    plot(s.p(m).b(2), s.p(m).b(3), '.','markersize',30,'color','white');
    plot([s.p(m).b(2) s.p(m).b(2)+8*s.p(m).A(2,3)], [s.p(m).b(3) s.p(m).b(3)+8*s.p(m).A(3,3)], 'MarkerSize',30,'color','black');
%     plot([s.p(m).b(2) s.p(m).b(2)+45*s.p(m).A(2,3)], [s.p(m).b(3) s.p(m).b(3)+45*s.p(m).A(3,3)], 'linewidth',4,'color','yellow');
end

end