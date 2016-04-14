points=zeros(3,3891);
parfor i=1:3891
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','F:\',i);
M1=load(scanName1);
p=M1.SCAN.X_wv;
points(:,i)=[p(1,1) p(2,1) p(3,1)];
end
%%
