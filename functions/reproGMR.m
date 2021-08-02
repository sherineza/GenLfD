function r = reproGMR(s, model, SigmaGMR, MuGMR, leadFrames,indx)
model.nbFrames=length(leadFrames);
model.nbVar=size(MuGMR,1)+1;
model.nbData=size(MuGMR,2);
model.nbSamples=length(s);

out = 2:model.nbVar;
r=s;

if nargin ==5
    indx=1:length(leadFrames);
end

for n=1:model.nbSamples
    MuTmp = zeros(length(out), model.nbData, model.nbFrames);
    SigmaTmp = zeros(length(out), length(out), model.nbData, model.nbFrames);
    
    pTmp = s(n).p(leadFrames);
    
    %Linear transformation of the retrieved Gaussians
    for m=1:model.nbFrames
        MuTmp(1:2,:,m) = pTmp(m).A(2:end,2:end) * MuGMR(1:2,:,leadFrames(m)) + repmat(pTmp(m).b(2:end),1,model.nbData);
        for t=1:model.nbData
            SigmaTmp(1:2,1:2,t,m) = pTmp(m).A(2:end,2:end) * SigmaGMR(1:2,1:2,t,leadFrames(m)) * pTmp(m).A(2:end,2:end)';
        end
        if model.nbVar>3
            MuTmp(3:end,:,m) =  MuGMR(3:end,:,leadFrames(m)) ;
            for t=1:model.nbData
                SigmaTmp(3:end,3:end,t,m) = SigmaGMR(3:end,3:end,t,leadFrames(m)) ;
            end
        end
    end
    
    method =3;
    MuTmp2=zeros(length(out),  model.nbData, model.nbFrames);
    SigmaTmp2= zeros(length(out),length(out), model.nbData, model.nbFrames);
    if method ==1
        %FINDING THE CLOSEST RING POINT TO THE OTHER GAUSSIANS
        for t=1:model.nbData
            for m=1:length(leadFrames)
                if model.orientless(indx(m))==1 && length(leadFrames)>1%don't do for any frame
                    closestid=0;%initialise
                    closestd=100000;
                    
                    for k=1:length(leadFrames)
                        if model.orientless(indx(k))==0
                            d=sqrt((MuTmp(1,t,k)-pTmp(m).b(2))^2+(MuTmp(2,t,k)-pTmp(m).b(3))^2)-model.Mu_dohnut(t,indx(m));
                            if closestd>d
                                closestd=d;
                                closestid=k;
                            end
                        else
                            if k~=m
                                d=sqrt((pTmp((k)).b(2)-pTmp((m)).b(2))^2+(pTmp((k)).b(3)-pTmp((m)).b(3))^2)-model.Mu_dohnut(t,indx(k));
                                if closestd>d
                                    closestd=d;
                                    closestid=k;
                                end
                            end
                        end
                    end
                    %find the coordinates of the actual pint closest
                    MuTmp2(1:2,t,m)=(closestd/(closestd+model.Mu_dohnut(t,indx(m)))).*(pTmp(m).b(2:3)-MuTmp(1:2,t,closestid))+MuTmp(1:2,t,closestid);
                    if model.orientless(indx(closestid))==1
                        MuTmp2(1:2,t,m)=pTmp(m).b(2:3)+(model.Mu_dohnut(t,indx(m))/(closestd+model.Mu_dohnut(t,indx(closestid)))).*[(pTmp((closestid)).b(2)-pTmp((m)).b(2));(pTmp((closestid)).b(3)-pTmp((m)).b(3))];
                    end
                    eigenvectorsmall=[(pTmp((closestid)).b(2)-pTmp((m)).b(2)),(pTmp((closestid)).b(3)-pTmp((m)).b(3))];
                    eigenvectorbig=[(pTmp((closestid)).b(3)-pTmp((m)).b(3)),-(pTmp((closestid)).b(2)-pTmp((m)).b(2))];
                    V=[eigenvectorsmall;eigenvectorbig];
                    D=[model.Sigma_dohnut(t,indx(m)),0;0,model.Sigma_dohnut(t,indx(m))*2];
                    SigmaTmp2(1:2,1:2,t,m)=V*D*inv(V);
                    
                    %                 SigmaTmp2(1:2,1:2,t,m)=[model.Sigma_dohnut(t,indx(m)),0;0,model.Sigma_dohnut(t,indx(m))];
                    if model.nbVar>3
                        MuTmp2(3:model.nbVar-1,t,m)=MuTmp(3:end,t,m);
                        SigmaTmp2(3:model.nbVar-1,3:model.nbVar-1,t,m)= SigmaTmp(3:end,3:end,t,m);
                    end
                else
                    MuTmp2(:,t,m)=MuTmp(:,t,m);
                    SigmaTmp2(:,:,t,m)=SigmaTmp(:,:,t,m);
                end
            end
        end
    elseif method==2
        for t=1:model.nbData
            for m=1:model.nbFrames
                d=[];point=[];
                if model.orientless(indx(m))==1 && length(leadFrames)>1%don't do for any frame
                    for k=1:model.nbFrames
                        %calculate distance between each 2 gaussians 
                        if model.orientless(indx(k))==0
                            d(k)=abs(sqrt((MuTmp(1,t,k)-pTmp(m).b(2))^2+(MuTmp(2,t,k)-pTmp(m).b(3))^2)-model.Mu_dohnut(t,indx(m)));
                            %find the closest point on the ring to each gaus
                            point(:,k)=(d(k)/(d(k)+model.Mu_dohnut(t,indx(m)))).*(pTmp(m).b(2:3)-MuTmp(1:2,t,k))+MuTmp(1:2,t,k);
                        else
                            if k~=m
                                d(k)=abs(sqrt((pTmp(k).b(2)-pTmp(m).b(2))^2+(pTmp((k)).b(3)-pTmp((m)).b(3))^2)-model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m)));
                                point(:,k)=(model.Mu_dohnut(t,indx(m)))/(d(k)+model.Mu_dohnut(t,indx(k))+model.Mu_dohnut(t,indx(m))).*(pTmp(k).b(2:3)-pTmp(m).b(2:3))+pTmp(m).b(2:3);
%                                 point(:,k)=pTmp(m).b(2:3)+(model.Mu_dohnut(t,indx(m))/(d(k)+model.Mu_dohnut(t,indx(k)))).*[(pTmp((k)).b(2)-pTmp((m)).b(2));(pTmp((k)).b(3)-pTmp((m)).b(3))];
                            end
                        end
                    end
                    
                    %average the points weighted by the sigma
                    dd=1./d;
                    
                    for k=1:model.nbFrames
                        if model.orientless(indx(k))==1 && k~=m
                            %                             temp(1:2,1:2,t,m)=[model.Sigma_dohnut(t,indx(m)),0;0,model.Sigma_dohnut(t,indx(m))];
                            %                             SigmaTmp2(:,:,t,m)=SigmaTmp2(:,:,t,m)+inv(temp(:,:,t,indx(k)));
                            %                             MuTmp2(:,t,m) = MuTmp2(:,t,m) +temp(:,:,t,indx(k)) \ point(:,k);
                            MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m)+point(:,k)*dd(k)/sum(dd(dd~=Inf));
                        elseif model.orientless(indx(k))==0
                            %                             SigmaTmp2(:,:,t,m)=SigmaTmp2(:,:,t,m)+inv(SigmaTmp(:,:,t,k));
                            %                             MuTmp2(:,t,m) = MuTmp2(:,t,m) + SigmaTmp(:,:,t,k) \ point(:,k);
                            MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m)+point(:,k)*(dd(k)/sum(dd(dd~=Inf)));
                        end
                    end
                    MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m);
                    SigmaTmp2(1:2,1:2,t,m)=[[model.Sigma_dohnut(t,indx(m)),0];[0,model.Sigma_dohnut(t,indx(m))]];
                    %                     SigmaTmp2(:,:,t,m) = inv(SigmaTmp2(:,:,t,m));
                    %                     MuTmp2(:,t,m) = SigmaTmp2(:,:,t,m)  * MuTmp2(:,t,m);
                    if model.nbVar>3
                        MuTmp2(3:model.nbVar-1,t,m)=MuTmp(3:end,t,m);
                        SigmaTmp2(3:model.nbVar-1,3:model.nbVar-1,t,m)= SigmaTmp(3:end,3:end,t,m);
                    end
                else
                    MuTmp2(:,t,m)=MuTmp(:,t,m);
                    SigmaTmp2(:,:,t,m)=SigmaTmp(:,:,t,m);
                end
            end
        end
    elseif method ==3
        for m=1:model.nbFrames
            mode=[];
            for t=1:model.nbData
                d=zeros(1,length(leadFrames));point=zeros(2,length(leadFrames));
                if model.orientless(indx(m))==1 && length(leadFrames)>1%don't do for any frame
                    for k=1:model.nbFrames
                        %calculate distance between each 2 gaussians 
                        if model.orientless(indx(k))==0
                            d(k)=abs(sqrt((MuTmp(1,t,k)-pTmp(m).b(2))^2+(MuTmp(2,t,k)-pTmp(m).b(3))^2)-model.Mu_dohnut(t,indx(m)));
                            %find the closest point on the ring to each gaus
                            point(:,k)=(d(k)/(d(k)+model.Mu_dohnut(t,indx(m)))).*(pTmp(m).b(2:3)-MuTmp(1:2,t,k))+MuTmp(1:2,t,k);
                        else
                            if k~=m
                                fdist=sqrt((pTmp(k).b(2)-pTmp(m).b(2))^2+(pTmp((k)).b(3)-pTmp((m)).b(3))^2);
                                if model.Mu_dohnut(t,indx(k))+model.Mu_dohnut(t,indx(m))<fdist %2 external circles
                                    d(k)=fdist-model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m));
                                    point(:,k)=(model.Mu_dohnut(t,indx(m)))/(fdist).*(pTmp(k).b(2:3)-pTmp(m).b(2:3))+pTmp(m).b(2:3);
                                
                                    mode(t,k)=1;
                                elseif model.Mu_dohnut(t,indx(k))+model.Mu_dohnut(t,indx(m))>=fdist && abs(model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m)))<=fdist %2 intersecting circles
                                    temp1=model.Mu_dohnut(t,indx(m))-(model.Mu_dohnut(t,indx(m))+model.Mu_dohnut(t,indx(k))-fdist)/2;
                                    temp2=sqrt(model.Mu_dohnut(t,indx(m))^2-temp1^2);
                                    if indx(m)<indx(k)
                                        transmat=[0,-1;+1,0];
                                    else
                                        transmat=[0,1;-1,0];
                                    end
                                    
                                    x=(fdist^2-model.Mu_dohnut(t,indx(k))^2+model.Mu_dohnut(t,indx(m))^2)/(2*fdist);
                                    y=sqrt(model.Mu_dohnut(t,indx(m))^2-x^2);
%                                    if length(leadFrames)==3 figure (100);hold on; plot(t, y,'.');end
%                                     d(k)=sqrt(min(min(-fdist+model.Mu_dohnut(t,indx(k))+model.Mu_dohnut(t,indx(m)),model.Mu_dohnut(t,indx(m))*2+fdist-model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m))),model.Mu_dohnut(t,indx(k))*2+fdist-model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m))))^2;
%                                     d(k)=min(x,model.Mu_dohnut(t,indx(m))*2-x);
                                    % d(k)=x;
                                     d(k)=0.00000000000000000001;
                                     point(:,k)=pTmp(m).b(2:3)+x.*(pTmp(k).b(2:3)-pTmp(m).b(2:3))./fdist+(transmat*(pTmp(k).b(2:3)-pTmp(m).b(2:3))).*y./fdist;
                                     
                                     mode(t,k)=2;
                                     %point(:,k)=pTmp(m).b(2:3)+(model.Mu_dohnut(t,indx(m))/(d(k)+model.Mu_dohnut(t,indx(k)))).*[(pTmp((k)).b(2)-pTmp((m)).b(2));(pTmp((k)).b(3)-pTmp((m)).b(3))];
%                                      if length(leadFrames)==3 && mod(t,101)==0
%                                          %plot on image
%                                          plot(pTmp(m).b(2),pTmp(m).b(3),'.','markersize',30,'color','yellow');hold on;
%                                          plot(pTmp(k).b(2),pTmp(k).b(3),'.','markersize',30,'color','yellow');
%                                          tt=linspace(-pi,pi,1000);
%                                          x=model.Mu_dohnut(t,indx(m))*[cos(tt);sin(tt)]+[pTmp(m).b(2);pTmp(m).b(3)];x=[x,x(:,1)];
%                                          patch(x(1,:),x(2,:),[.5 .5 .5],'EdgeColor', [.5 .5 .5], 'facealpha', 0,'edgealpha', 0.5,'LineWidth',10000*model.Sigma_dohnut(t,indx(m)));
%                                          patch(x(1,1:end-1),x(2,1:end-1),[.5 .5 .5]*0.5,'EdgeColor', [.5 .5 .5]*0.5, 'facealpha', 0,'edgealpha', 0.5,'LineWidth',1);
%                                          x=model.Mu_dohnut(t,indx(k))*[cos(tt);sin(tt)]+[pTmp(k).b(2);pTmp(k).b(3)];x=[x,x(:,1)];
%                                          patch(x(1,:),x(2,:),[.5 .5 .5],'EdgeColor', [.5 .5 .5], 'facealpha', 0,'edgealpha', 0.5,'LineWidth',10000*model.Sigma_dohnut(t,indx(k)));
%                                          patch(x(1,1:end-1),x(2,1:end-1),[.5 .5 .5]*0.5,'EdgeColor', [.5 .5 .5]*0.5, 'facealpha', 0,'edgealpha', 0.5,'LineWidth',1);
%                                         
%                                      end
                                elseif abs(model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m)))>fdist
                                    if model.Mu_dohnut(t,indx(m))>model.Mu_dohnut(t,indx(k))
                                        d(k) = abs(model.Mu_dohnut(t,indx(m))-model.Mu_dohnut(t,indx(k))-fdist); %distance between two circles
                                        point(:,k)=pTmp(m).b(2:3)+(pTmp(k).b(2:3)-pTmp(m).b(2:3))*model.Mu_dohnut(t,indx(m))/fdist;
                                    else
                                        d(k) =abs( model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m))-fdist);%distance between two circles
                                        point(:,k)=pTmp(m).b(2:3)+(-pTmp(k).b(2:3)+pTmp(m).b(2:3))*model.Mu_dohnut(t,indx(m))/fdist;
                                    end
                                    mode(t,k)=3;
                                end
                            end
                            
                            
                        end

                    end
                
                     dd=1./d;

                    for k=1:model.nbFrames
                        
                        if k~=m
                            MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m)+point(:,k)*(dd(k)/(sum(dd(find(dd~=dd(m))))));
                        end
                    end
                    MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m);
                    SigmaTmp2(1:2,1:2,t,m)=[[model.Sigma_dohnut(t,indx(m)),0];[0,model.Sigma_dohnut(t,indx(m))]];
                    %                     SigmaTmp2(:,:,t,m) = inv(SigmaTmp2(:,:,t,m));
                    %                     MuTmp2(:,t,m) = SigmaTmp2(:,:,t,m)  * MuTmp2(:,t,m);
                    if model.nbVar>3
                        MuTmp2(3:model.nbVar-1,t,m)=MuTmp(3:end,t,m);
                        SigmaTmp2(3:model.nbVar-1,3:model.nbVar-1,t,m)= SigmaTmp(3:end,3:end,t,m);
                    end
                else
                    MuTmp2(:,t,m)=MuTmp(:,t,m);
                    SigmaTmp2(:,:,t,m)=SigmaTmp(:,:,t,m);
                end
            end
        end
    elseif method ==4
        for t=1:model.nbData
            for m=1:model.nbFrames
                d=[];point=[];
                if model.orientless(indx(m))==1 && length(leadFrames)>1%don't do for any frame
                    for k=1:model.nbFrames
                        %calculate distance between each 2 gaussians 
                        if model.orientless(indx(k))==0
                            [sigma,correlation]=cov2corr(SigmaTmp(1:2,1:2,t,k));
                            d(k)=sum(sigma)/2;
                            %find the closest point on the ring to each gaus
                            point(:,k)=(d(k)/(d(k)+model.Mu_dohnut(t,indx(m)))).*(pTmp(m).b(2:3)-MuTmp(1:2,t,k))+MuTmp(1:2,t,k);
                        else
                            if k~=m
                                fdist=sqrt((pTmp(k).b(2)-pTmp(m).b(2))^2+(pTmp((k)).b(3)-pTmp((m)).b(3))^2);
                                if model.Mu_dohnut(t,indx(k))+model.Mu_dohnut(t,indx(m))<fdist %2 external circles
                                    d(k)=abs(sqrt((pTmp(k).b(2)-pTmp(m).b(2))^2+(pTmp((k)).b(3)-pTmp((m)).b(3))^2)-model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m)));
                                    point(:,k)=(model.Mu_dohnut(t,indx(m)))/(d(k)+model.Mu_dohnut(t,indx(k))+model.Mu_dohnut(t,indx(m))).*(pTmp(k).b(2:3)-pTmp(m).b(2:3))+pTmp(m).b(2:3);
                                elseif model.Mu_dohnut(t,indx(k))+model.Mu_dohnut(t,indx(m))>=fdist && abs(model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m)))<=fdist %2 intersecting circles
                                    if k<m
                                        transmat=[0,-1;+1,0];
                                    else
                                        transmat=[0,1;-1,0];
                                    end
                                    x=(fdist^2-model.Mu_dohnut(t,indx(k))^2+model.Mu_dohnut(t,indx(m))^2)/(2*fdist);
                                    y=sqrt((4*fdist^2*model.Mu_dohnut(t,indx(m))^2-(fdist^2-model.Mu_dohnut(t,indx(k))^2+model.Mu_dohnut(t,indx(m))^2)^2)/(4*fdist^2));
                                    point(:,k)=pTmp(m).b(2:3)+x.*(pTmp(m).b(2:3)-pTmp(k).b(2:3))./fdist+(transmat*(pTmp(m).b(2:3)-pTmp(k).b(2:3))).*y./fdist;
                                elseif abs(model.Mu_dohnut(t,indx(k))-model.Mu_dohnut(t,indx(m)))>fdist
                                    if model.Mu_dohnut(t,indx(m))>model.Mu_dohnut(t,indx(k))
                                        point(:,k)=pTmp(m).b(2:3)+(pTmp(k).b(2:3)-pTmp(m).b(2:3))*model.Mu_dohnut(t,indx(m))/fdist;
                                    else
                                        point(:,k)=pTmp(m).b(2:3)+(-pTmp(k).b(2:3)+pTmp(m).b(2:3))*model.Mu_dohnut(t,indx(m))/fdist;
                                    end
                                end
                                d(k)=model.Sigma_dohnut(t,indx(k));
                            end
                        end
                    end
                    
                    %average the points weighted by the sigma
                    dd=1./d;
                    dd=normalize(-d,'range',[0,1]);
                    
                    for k=1:model.nbFrames
                        if k~=m
                            MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m)+point(:,k)*dd(k)/sum(dd(dd~=Inf));
%                             MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m)+point(:,k)*sum(d)/d(k);
                      
                        end
                    end
                    MuTmp2(1:2,t,m)=MuTmp2(1:2,t,m);
                    SigmaTmp2(1:2,1:2,t,m)=[[model.Sigma_dohnut(t,indx(m)),0];[0,model.Sigma_dohnut(t,indx(m))]];
                    %                     SigmaTmp2(:,:,t,m) = inv(SigmaTmp2(:,:,t,m));
                    %                     MuTmp2(:,t,m) = SigmaTmp2(:,:,t,m)  * MuTmp2(:,t,m);
                    if model.nbVar>3
                        MuTmp2(3:model.nbVar-1,t,m)=MuTmp(3:end,t,m);
                        SigmaTmp2(3:model.nbVar-1,3:model.nbVar-1,t,m)= SigmaTmp(3:end,3:end,t,m);
                    end
                else
                    MuTmp2(:,t,m)=MuTmp(:,t,m);
                    SigmaTmp2(:,:,t,m)=SigmaTmp(:,:,t,m);
                end
            end
        end
    end
    
    %Product of Gaussians (fusion of information from the different coordinate systems)
    for t=1:model.nbData
        SigmaP = zeros(length(out));
        MuP = zeros(length(out), 1);
        for m=1:model.nbFrames % or model.nbFrames to compute for all the frames
            SigmaP = SigmaP + inv(SigmaTmp2(:,:,t,m));
            MuP = MuP + SigmaTmp2(:,:,t,m) \ MuTmp2(:,t,m);
        end
        r(n).Sigma(:,:,t) = inv(SigmaP);
        r(n).Data(1:model.nbVar-1,t) = r(n).Sigma(:,:,t) * MuP;
    end
end

end

