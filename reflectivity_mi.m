clearvars
clc
close all
%%
scanName1 = sprintf('%s/SCANS_FOR_MI/Scan_for_MI_%04d.txt','D:\data',0200);
M1 = dlmread(scanName1);
R1 = M1(2:size(M1,1),4);
pts1 = pointCloud(M1(2:size(M1,1),1:3));
pts1.Normal=pcnormals(pts1);
[ptc1,nptc1,plane1,p1,outliers1]=gp(pts1);
xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
scanName2 = sprintf('%s/SCANS_FOR_MI/Scan_for_MI_%04d.txt','D:\data',0215);
M2 = dlmread(scanName2);
R2 = M2(2:size(M2,1),4);
pts2 = pointCloud(M2(2:size(M2,1),1:3));
pts2.Normal=pcnormals(pts2);
[ptc2,nptc2,plane2,p2,outliers2]=gp(pts2);
xlimit2=nptc2.XLimits(1,2)-nptc2.XLimits(1,1);
ylimit2=nptc2.YLimits(1,2)-nptc2.YLimits(1,1);
xlimit=max(xlimit1,xlimit2);
ylimit=max(ylimit1,ylimit2);
%%
Ref1=R1(outliers1);
Ref2=R2(outliers2);
[SI1,SB1]=grid2D(nptc1.Location,xlimit,ylimit);
[SR1,SRB1]=grid_reflectivity(nptc1.Location,Ref1,xlimit,ylimit);
[SR2,SRB2]=grid_reflectivity(nptc2.Location,Ref2,xlimit,ylimit);
