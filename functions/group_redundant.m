
function [leadFrames, objs] = group_redundant(s, threshold)
nbSamples=length(s);
% INDX=zeros(30,1); %maybe not needed

%Calculating Difference in angle and distance between every two frames
for n=1:length(s(1).p)-1
    for m=n+1:length(s(1).p)
        for i=1:nbSamples
            d = pdist2(s(i).p(n).b(2:3)',s(i).p(m).b(2:3)');
            a1=rotm2eul(s(i).p(n).A);a2=rotm2eul(s(i).p(m).A);
            a= pdist2(a1,a2);
            [x,y]=pol2cart(a,d);
            Diff(n,m,i,1:2)=[a,d];
        end
    end
end


for thresh=threshold
    Redund=eye(length(s(1).p));
    %Filling the Redundancy matrix
    for n=1:length(s(1).p)-1
        for m=n+1:length(s(1).p)
            %Thresholding std/mean
            if std(Diff(n,m,:,1),1,'all')< thresh*mean(Diff(n,m,:,1),'all') ...
                    && std(Diff(n,m,:,2),1,'all') < thresh*mean(Diff(n,m,:,2),'all')
                Redund(n,m)=1;Redund(m,n)=1;
            end
        end
    end
    
    %Gathering up the redundant frames into objects
    INDX=[];objINDX=[];
    
    for n=1:length(s(1).p)
        %Find the first 1 of each column
        indx=find(Redund(:,n));
        INDX=[INDX,indx(1)];
    end
    INDX=unique(INDX);
    
    method=2;
    if method==1 %original method in which sometimes the obtained lead Frames belong to each other
        %grouping them under same object
        Redund1=Redund-eye(length(s(1).p));
        n=1;
        for m=1:length(INDX)
            indx=find(Redund1(:,INDX(m))==1);
            if length(indx)>0
                %             if INDX(n)<indx(1)
                objINDX(1,n)=INDX(n);
                objINDX(2:length(unique(indx))+1,n)=unique(indx');
                n=n+1;
                %             else
                %                 objINDX(1,n)=INDX(n);
                %                 n=n+1;
                %             end
            else
                objINDX(1,n)=INDX(n);
                n=n+1;
            end
        end
    end
    
    if method==2 %if a point of less index belong to the object, then the lead frame of that object is ignored and the points belonging to it are merged with another object
        %grouping them under same object
        Redund1=Redund-eye(length(s(1).p));
        n=1;
        for m=1:length(INDX)
            indx=find(Redund1(:,INDX(m))==1);
            if length(indx)>0
                if INDX(m)>indx(1) %found point of lesser value => merge objects together
                    tempindx=find(objINDX(1,:)==indx(1));
                    while isempty(tempindx)
                       indx=find(Redund1(:,indx(1))==1);
                       tempindx=find(objINDX(1,:)==indx(1));
                    end
                    indx=find(Redund1(:,INDX(m))==1);
                    objINDX(end+1:end+length(indx),tempindx)=indx;
                    objINDX(1:length(unique(objINDX(:,tempindx))),tempindx)=unique(objINDX(:,tempindx));
                    objINDX(length(unique(objINDX(:,tempindx)))+1:end,tempindx)=objINDX(length(unique(objINDX(:,tempindx)))+1:end,tempindx).*0;
                    if objINDX(1,tempindx)==0
                        objINDX(1:length(unique(objINDX(:,tempindx))),tempindx)=objINDX(2:length(unique(objINDX(:,tempindx)))+1,tempindx);
                    end
                else
                    objINDX(1,n)=INDX(m);
                    objINDX(2:length(unique(indx))+1,n)=unique(indx');
                    n=n+1;
                    %             else
                    %                 objINDX(1,n)=INDX(n);
                    %                 n=n+1;
                    %             end
                end
            else
                objINDX(1,n)=INDX(m);
                n=n+1;
            end
        end
    end
    
    if length(INDX)<20 %justify by experiment concerning the max number of frames the RL can handle
        break
    end
end

leadFrames=objINDX(1,:);
objs=objINDX;
end

