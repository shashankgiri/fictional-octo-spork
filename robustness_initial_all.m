clc;
clearvars
close all
file_path1=('F:\HP\experiments\resultsall\result_new_');
c=100;
l=200;
k_max=8;
j_max=1;
TX=zeros(c,1);
TY=zeros(c,1);
TZ=zeros(c,1);
RX=zeros(c,1);
RY=zeros(c,1);
RZ=zeros(c,1);
tx=zeros(c,1);
ty=zeros(c,1);
tz=zeros(c,1);
rx=zeros(c,1);
ry=zeros(c,1);
rz=zeros(c,1);
%%
for j=1:j_max

scanName1 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',l+25*(j-1));
M1 = load(scanName1);
I1 = M1.points(2:size(M1.points,1),5);
R1 = M1.points(2:size(M1.points,1),4);
RGB1 = M1.points(2:size(M1.points,1),6:8);
pts1 = pointCloud(M1.points(2:size(M1.points,1),1:3));
pts1.Normal = pcnormals(pts1);
[ptc1,~,plane1,p1,outliers1]=gp(pts1);
xlimit1=pts1.XLimits(1,2)-pts1.XLimits(1,1);
ylimit1=pts1.YLimits(1,2)-pts1.YLimits(1,1);
%%
for k=1:k_max
for i=1:c 
ax=(pi/6)*(1-2*rand);
Tx=3-6*rand;
ay=(pi/6)*(1-2*rand);
Ty=3-6*rand;
az=(pi/6)*(1-2*rand);
Tz=3-6*rand;
Rx=[1 0 0;0 cos(ax) sin(ax);0 -sin(ax) cos(ax)];
Ry=[cos(ay) 0 -sin(ay);0 1 0;sin(ay) 0 cos(ay)];
Rz=[cos(az) sin(az) 0;-sin(az) cos(az) 0;0 0 1];
R=Rz*Ry*Rx;
T=[Tx Ty Tz 1];
A=[R,[0 0 0]';T];
tfr=affine3d(A);
%%
tic
scanName2 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',l+25*(j-1)+k);
M2 = load(scanName2);
I2 = M2.points(2:size(M2.points,1),5);
R2 = M2.points(2:size(M2.points,1),4);
RGB2 = M2.points(2:size(M2.points,1),6:8);
pts2 = pointCloud(M2.points(2:size(M2.points,1),1:3));
pts2.Normal = pcnormals(pts2);
pts2 = pctransform(pts2,tfr);
[ptc2,~,plane2,p2,outliers2] = groundPlane(pts2,4);
xlimit2 = pts2.XLimits(1,2)-pts2.XLimits(1,1);
ylimit2 = pts2.YLimits(1,2)-pts2.YLimits(1,1);
xlimit = roundn(max(xlimit1,xlimit2),1);
ylimit = roundn(max(ylimit1,ylimit2),1);
%%
Ref1 = R1;%(outliers1);
Ref2 = R2;%(outliers2);
int1 = I1;%(outliers1);
int2 = I2;%(outliers2);
rgb1 = RGB1;%(outliers1,:);
rgb2 = RGB2;%(outliers2,:);
[~,SB1] = grid2D(pts1.Location,xlimit,ylimit);
[~,SB1R] = grid2DR(pts1.Location,xlimit,ylimit);
[~,SRB1] = grid_reflectivity(pts1.Location,Ref1,xlimit,ylimit);
[~,SRB1R] = grid_reflectivityR(pts1.Location,Ref1,xlimit,ylimit);
[~,Sib1] = grid_intensity(pts1.Location,int1,xlimit,ylimit);
[~,Sib1R] = grid_intensityR(pts1.Location,int1,xlimit,ylimit);
[~,SCB1] = grid_rgb(pts1.Location,rgb1,xlimit,ylimit);
[~,SCB1R] = grid_rgbR(pts1.Location,rgb1,xlimit,ylimit);
%%
[R12,eul1] = find_rotation(plane1,plane2);
t = R12*mean(pts1.Location)'-mean(pts2.Location)';
%%
tf0=0;
ObjectiveFunction1 = @(tf)mi_all2(tf,SB1R,SRB1R,Sib1R,SCB1R,pts2,Ref2,int2,rgb2,xlimit,ylimit);
[tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,tf0);
%%
x0=[0;0];
objectiveFunction2 = @(x)mi_all(x,tf,SB1,SRB1,Sib1,SCB1,pts2,Ref2,int2,rgb2,xlimit,ylimit);
[x,fval2,exitFlag2,output2] = patternsearch(objectiveFunction2,x0);
%%
TR = [eul1(1,3); eul1(1,2); tf];
eul = [tf eul1(1,2) eul1(1,3)];
rt = eul2rotm(eul);
TT = [x(1:2,1)' t(3,1)];
% A = [rt,[0 0 0]';[TT 1]];
% tform = affine3d(A);
% pts22 = pctransform(pts2,tform);
%%
TX(i,1) = Tx;
TY(i,1) = Ty;
TZ(i,1) = Tz;
RX(i,1) = ax;
RY(i,1) = ay;
RZ(i,1) = az;
tx(i,1) = TT(1,1);
ty(i,1) = TT(1,2);
tz(i,1) = TT(1,3);
rx(i,1) = TR(1,1);
ry(i,1) = TR(2,1);
rz(i,1) = TR(3,1);
toc;
end
file_name1=[file_path1 num2str(l+25*(j-1)) '-' num2str(l+25*(j-1)+k) '.mat'];
save(file_name1,'tx','ty','tz','rx','ry','rz','TX','TY','TZ','RX','RY','RZ','TT','TR');
 end
 end