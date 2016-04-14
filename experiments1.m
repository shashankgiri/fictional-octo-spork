clear M1 M2 pts1 pts2
file_path1=('I:\experiments\results1\data_');
file_path2=('I:\experiments\results1\mean\mean_');
l=200;
a=1;
b=1;
k_max=12;
j_max=12;
dx=zeros(j_max,1);
dy=zeros(j_max,1);
dz=zeros(j_max,1);
rx=zeros(j_max,1);
ry=zeros(j_max,1);
rz=zeros(j_max,1);
% mx=zeros(j_max,1);
% my=zeros(j_max,1);
% mz=zeros(j_max,1);
% mrx=zeros(j_max,1);
% mry=zeros(j_max,1);
% mrz=zeros(j_max,1);
dxt=zeros(j_max,1);
dyt=zeros(j_max,1);
dzt=zeros(j_max,1);
rxt=zeros(j_max,1);
ryt=zeros(j_max,1);
rzt=zeros(j_max,1);
% mxt=zeros(j_max,1);
% myt=zeros(j_max,1);
% mzt=zeros(j_max,1);
% mrxt=zeros(j_max,1);
% mryt=zeros(j_max,1);
% mrzt=zeros(j_max,1);
for k=a:k_max
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',l+5*(k-1));
M2=load(scanName2);
pts2= pointCloud(M2.SCAN.XYZ');
pts2 = pcdenoise(pts2);
pts2.Normal=pcnormals(pts2);
for j=b:j_max
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',l+5*(k-1)+j);
M1=load(scanName1);
pts1= pointCloud(M1.SCAN.XYZ');
pts1 = pcdenoise(pts1);
pts1.Normal=pcnormals(pts1);
[ptcloud1,nptcloud1,plane1]=groundPlane(pts1,4);
[ptcloud2,nptcloud2,plane2]=groundPlane(pts2,4);
%%
tform = pcregrigid(pts1,pts2);
el1=affine3dtoeul(tform);
x=tform.T(4,1);
y=tform.T(4,2);
z=tform.T(4,3);
qx=el1(1,3);
qy=el1(1,2);
qz=el1(1,1);
%for i=1:c
%%
tic;
[R12,eul]=find_rotation(plane1,plane2);
t=-mean(pts2.Location)*R12'+mean(pts1.Location);
B=[R12,[0 0 0]';[t,1]];
tf12=affine3d(B);
%pts2rr=pctransform(pts2r,tf12);
%nptcloud2r=pctransform(nptcloud2,tf12);
%%
dx(j,1)=x;
dy(j,1)=y;
dz(j,1)=z;
rx(j,1)=qx;
ry(j,1)=qy;
rz(j,1)=qz;
dxt(j,1)=tf12.T(4,1);
dyt(j,1)=tf12.T(4,2);
dzt(j,1)=tf12.T(4,3);
rxt(j,1)=eul(1,3);
ryt(j,1)=eul(1,2);
rzt(j,1)=eul(1,1);toc;
% mx(j,1)=sqrt(mean(dx.^2));
% my(j,1)=sqrt(mean(dy.^2));
% mz(j,1)=sqrt(mean(dz.^2));
% mrx(j,1)=sqrt(mean(rx.^2));
% mry(j,1)=sqrt(mean(ry.^2));
% mrz(j,1)=sqrt(mean(rz.^2));
% mxt(j,1)=sqrt(mean(dxt.^2));
% myt(j,1)=sqrt(mean(dyt.^2));
% mzt(j,1)=sqrt(mean(dzt.^2));
% mrxt(j,1)=sqrt(mean(rxt.^2));
% mryt(j,1)=sqrt(mean(ryt.^2));
% mrzt(j,1)=sqrt(mean(rzt.^2));
end
file_name1=[file_path1 num2str(l+5*(k-1)) '-' num2str(l+5*(k-1)+j) '.mat'];
save(file_name1,'dx','dy','dz','rx','ry','rz','dxt','dyt','dzt','rxt','ryt','rzt');
% file_name2=[file_path2 num2str(l+5*(k-1)) '-' num2str(l+5*(k-1)+j) '.mat'];
% save(file_name2,'mx','my','mz','mrx','mry','mrz','mxt','myt','mzt','mrxt','mryt','mrzt');
end