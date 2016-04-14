clc
clearvars
close all
%%
%pose=load_pose_applanix('Pose-Applanix.log');
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','F:\',200);
M1=load(scanName1);
pts1 = pointCloud(M1.SCAN.XYZ');
pts11=pts1;
%gridStep = 0.2;
%pts11 = pcdownsample(pts1,'gridAverage',gridStep);
%pts11 = pcdownsample(pts1,'random',gridStep);
pts11.Normal=pcnormals(pts11);
[ptc1,nptc1,plane1,p1]=gp(pts11);
%plane1=plane1.Normal';
% xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
% ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','F:\',205);
M2=load(scanName2);
pts2 = pointCloud(M2.SCAN.XYZ');
pts22=pts2;
%pts22 = pcdownsample(pts2,'gridAverage',gridStep);
%pts22 = pcdownsample(pts2,'random',gridStep);
pts22.Normal=pcnormals(pts22);
[ptc2,nptc2,plane2,p2]=gp(pts22);
%plane2=plane2.Normal';
% xlimit2=nptc2.XLimits(1,2)-nptc2.XLimits(1,1);
% ylimit2=nptc2.YLimits(1,2)-nptc2.YLimits(1,1);
xlimit = 200;%max(xlimit1,xlimit2);
ylimit = 200;%max(ylimit1,ylimit2);
[SI1,SB1]=grid2D(pts11.Location,xlimit,ylimit);
[SI1R,SB1R]=grid2DR(pts11.Location,xlimit,ylimit);
%[SI2,SB2]=grid2D(nptc2.Location,xlimit,ylimit);
tic;
%%
[R12,eul1]=find_rotation(plane1,plane2);
t=R12*mean(pts11.Location)'-mean(pts22.Location)';
%t=R12*p1'-p2';
 %%
% l1=find(M1.SCAN.timestamp_laser>pose.utime, 1, 'last' );
% X_wv1=[pose.pos(l1,:) ,quat2eul(pose.orientation(l1,:))];
% l2=find(M2.SCAN.timestamp_laser>pose.utime, 1, 'last' );
% X_wv2=[pose.pos(l2,:) ,quat2eul(pose.orientation(l2,:))];
% pose1=ssc_tail2tail(X_wv2',X_wv1');
 pose1=ssc_tail2tail(M1.SCAN.X_wv,M2.SCAN.X_wv);

%%
%options = psoptimset('MeshAccelerator','on','ScaleMesh','off');
%options = psoptimset('UseParallel', true,'CompletePoll','on', 'Vectorized', 'off');
tf0=0;
ObjectiveFunction1=@(tf)mi2(tf,SB1R,pts22,xlimit,ylimit);
[tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,tf0);
%%
x0=[0;0];
objectiveFunction2=@(x)mi(x,tf,SB1,pts22,xlimit,ylimit);
[x,fval2,exitFlag2,output2] = patternsearch(objectiveFunction2,x0);
%%
  TR=[eul1(1,3); eul1(1,2); tf];
  eul=[tf eul1(1,2) eul1(1,3)];
  rt=eul2rotm(eul);
  TT=[x(1:2,1)' t(3,1)];
  A=[rt,[0 0 0]';[TT 1]];
  tform=affine3d(A);
  pts23=pctransform(pts2,tform);
 toc;
 %%
 eul2=[-pose1(6,1) pose1(5,1) -pose1(4,1)];
 rt1=eul2rotm(eul2);
 TT1=[-pose1(2,1) pose1(1,1) pose1(3,1)];
 B=[rt1,[0 0 0]';[TT1 1]];
 tf1=affine3d(B);
 pts24=pctransform(pts2,tf1);
%end
 %%
 tic;
 [tf2,pts25] = pcregrigid(pts2,pts1);
 TT2=tf2.T(4,1:3);
 TR2=affine3dtoeul(tf2);
 toc;
 %%
%  figure
%  pcshowpair(pts1,pts2);
 figure
 pcshowpair(pts1,pts23,'MarkerSize',10);
 %set(gca,'color','black');
 figure
 pcshowpair(pts1,pts24);
 figure
 pcshowpair(pts1,pts25);