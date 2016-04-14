function [ptcloud,nptcloud,plane,point,outIndices]=groundPlane(ptCloud,ra)
norm=ptCloud.Normal;
n=ptCloud.Count;
initial=0;
in=randi(n,ra,1);
for i=1:ra
[~,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,norm(in(i,1),:),8);
if size(inlierIndices,1)>initial
    initial=size(inlierIndices,1);
    inIndices=inlierIndices;
    outIndices=outlierIndices;
%     plane=p;
end
end
ptcloud=select(ptCloud,inIndices);
nptcloud=select(ptCloud,outIndices);
point = mean(ptcloud.Location,1);
cov= bsxfun(@minus,ptcloud.Location,point);
[V,~] = eig(cov'*cov);
plane = V(:,1);
end
