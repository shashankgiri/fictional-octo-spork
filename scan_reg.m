
M1=load('I:\thesis\mat-files\SCANS\Scan0366.mat');
M2=load('I:\thesis\mat-files\SCANS\Scan0365.mat');
ptXYZ1=pointCloud(M1.SCAN.XYZ');
ptXYZ2=pointCloud(M2.SCAN.XYZ');
% ptXYZ1.Normal=normal_estimation(ptXYZ1.Location,ptXYZ1.Count);
% ptXYZ2.Normal=normal_estimation(ptXYZ2.Location,ptXYZ2.Count);
ptXYZ1.Normal=pcnormals(ptXYZ1);
ptXYZ2.Normal=pcnormals(ptXYZ2);
%%
ax=pi/18;Tx=3;
ay=pi/18;Ty=1;
az=pi/18;Tz=1;
Rx=[1 0 0;0 cos(ax) sin(ax);0 -sin(ax) cos(ax)];
Ry=[cos(ay) 0 -sin(ay);0 1 0;sin(ay) 0 cos(ay)];
Rz=[cos(az) sin(az) 0;-sin(az) cos(az) 0;0 0 1];
R=Rz*Ry*Rx;
T=[Tx Ty Tz 1];
A=[R,[0 0 0]';T];
tfr=affine3d(A);
ptXYZ2r=pctransform(ptXYZ2,tfr);
% ptXYZ2r=ptXYZ2;
%%
% tic;
[xyz1,nxyz1,gr1,ngr1,pr1,plane1,Vr1]=ransac_plane(ptXYZ1.Location,ptXYZ1.Normal,ptXYZ1.Count,10);
ptXYZ1ot=pointCloud(nxyz1,'Normal',ngr1);
[xyz2,nxyz2,gr2,ngr2,pr2,plane2,Vr2]=ransac_plane(ptXYZ2r.Location,ptXYZ2r.Normal,ptXYZ2r.Count,10);
ptXYZ2rot=pointCloud(nxyz2,'Normal',ngr2);
%%
[R12,eul]=find_rotation(plane1,plane2);
t=-mean(ptXYZ1.Location)*R12'+mean(ptXYZ2r.Location);
B=[R12,[0 0 0]';[t,1]];
tf12=affine3d(B);
ptXYZ2rr=pctransform(ptXYZ2r,tf12);
%ptXYZ22rot=pctransform(ptXYZ2rot,tf12);
%%
% tic;[TR, TT] = icp(ptXYZ1ot.Location',ptXYZ22rot.Location');
% C=[TR,[0 0 0]';[TT',1]];
% [ptXYZ222r,M]=ICP_finite(ptXYZ22rot.Location,ptXYZ1ot.Location);
% ptXYZ22r=pointCloud(ptXYZ222r);
% tform=affine3d(C);
tic;[tform,ptXYZ2rott] = pcregrigid(ptXYZ22rot,ptXYZ1ot);
ptXYZ22r = pctransform(ptXYZ2rr,tform); toc;
%%
figure(1)
pcshowpair(ptXYZ1,ptXYZ2r);
set(gca,'color','black');
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on
figure(2)
pcshowpair(ptXYZ1,ptXYZ2rott);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(3)
pcshowpair(ptXYZ1,ptXYZ22r);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');