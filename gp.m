function [ptcloud,nptcloud,plane,point,outlierIndices]=gp(ptCloud)
[~,inlierIndices,~] = pcfitplane(ptCloud,1,[0,0,1],1);
[~,~,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],8);
ptcloud=select(ptCloud,inlierIndices);
nptcloud=select(ptCloud,outlierIndices);
point = mean(ptcloud.Location,1);
cov= bsxfun(@minus,ptcloud.Location,point);
[V,~] = eig(cov'*cov);
plane = V(:,1);
end