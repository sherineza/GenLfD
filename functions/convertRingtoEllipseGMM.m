function [MuTmp,SigmaTmp] = convertRingtoEllipseGMM (Mu_ring, Sigma_ring, frames, MuTmp,SigmaTmp,model)
   
%FINDING THE CLOSEST RING POINT TO THE OTHER GAUSSIANS
for t=1:model.nbData
    for m=1:model.nbFrames
        if model.orientless(m)==1
            closestid=0; %initialise
            closestd=1000;
            for k=1:model.nbFrames
                if model.orientless(k)==0
                    d=sqrt((MuTmp(1,t,k)-frames(m).b(2))^2+(MuTmp(2,t,k)-...
                        frames(m).b(3))^2)-Mu_ring(t,m);
                    if closestd>d
                        closestd=d;
                        closestid=k;
                    end
                else
                    if k~=m
                        d=sqrt((frames(k).b(2)-frames(m).b(2))^2+...
                            (frames(k).b(3)-frames(m).b(3))^2)-Mu_ring(t,m);
                        if closestd>d
                            closestd=d;
                            closestid=k;
                        end
                    end
                end
            end
            id(m,t)=closestid;
            
            %find the coordinates of the actual point closest
            MuTmp2(:,t,m)=(closestd/(closestd+Mu_ring(t,m))).*...
                (frames(m).b(2:3)-MuTmp(:,t,closestid))+MuTmp(:,t,closestid);
            SigmaTmp2(:,:,t,m)=[Sigma_ring(t,m),0;0,Sigma_ring(t,m)];
        else
            MuTmp2(:,t,m)=MuTmp(:,t,m);
            SigmaTmp2(:,:,t,m)=SigmaTmp(:,:,t,m);
        end
    end
end

MuTmp=MuTmp2;
SigmaTmp=SigmaTmp2;
    
end