function [TransformedScan,TT,TR] =  scan_alignment(folder,scan1,scan2)
close all
tic;
scanName1 = sprintf('%s/SCANS/Scan%04d.mat',folder,scan1);
M1=load(scanName1);
pts11 = pointCloud(M1.SCAN.XYZ');
%gridStep = 0.5;
%pts11 = pcdownsample(pts1,'gridAverage',gridStep);
pts11.Normal=pcnormals(pts11);
[~,~,plane1,~]=gp(pts11);

xlimit1=pts11.XLimits(1,2)-pts11.XLimits(1,1);
ylimit1=pts11.YLimits(1,2)-pts11.YLimits(1,1);
scanName2 = sprintf('%s/SCANS/Scan%04d.mat',folder,scan2);
M2=load(scanName2);
pts22 = pointCloud(M2.SCAN.XYZ');
%pts22 = pcdownsample(pts2,'gridAverage',gridStep);
pts22.Normal=pcnormals(pts22);
[~,~,plane2,~]=gp(pts22);
%plane2=plane2.Normal';
xlimit2=pts22.XLimits(1,2)-pts22.XLimits(1,1);
ylimit2=pts22.YLimits(1,2)-pts22.YLimits(1,1);
xlimit = roundn(max(xlimit1,xlimit2),1);
ylimit = roundn(max(ylimit1,ylimit2),1);
[~,SB1]=grid2D(pts11.Location,xlimit,ylimit);
[~,SB1R]=grid2DR(pts11.Location,xlimit,ylimit);
[R12,eul1]=find_rotation(plane1,plane2);
t=R12*mean(pts11.Location)'-mean(pts22.Location)';
tf0=0;
ObjectiveFunction1=@(tf)mi2(tf,SB1R,pts22,xlimit,ylimit);
[tf,~,~,~] = patternsearch(ObjectiveFunction1,tf0);
x0=[0;0];
objectiveFunction2=@(x)mi(x,tf,SB1,pts22,xlimit,ylimit);
[x,~,~,~] = patternsearch(objectiveFunction2,x0);
TR=[eul1(1,3); eul1(1,2); tf];
eul=[tf eul1(1,2) eul1(1,3)];
rt=eul2rotm(eul);
TT=[x' t(3,1)];
A=[rt,[0 0 0]';[TT 1]];
tform=affine3d(A);
pts23=pctransform(pts22,tform);
TransformedScan=pts23.Location;
toc;
figure
pcshowpair(pts11,pts22);
set(gca,'color','black');
figure 
pcshowpair(pts11,pts23);
set(gca,'color','black');