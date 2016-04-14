file_path1=('I:\experiments\results\data_');
file_path2=('I:\experiments\results\mean\mean_');
l=200;
c=1;
k_max=1;
j_max=1;
t1=zeros(c,1);
t2=zeros(c,1);
tx=zeros(c,1);
ty=zeros(c,1);
tz=zeros(c,1);
sx=zeros(c,1);
sy=zeros(c,1);
sz=zeros(c,1);
TX=zeros(c,1);
TY=zeros(c,1);
TZ=zeros(c,1);
RX=zeros(c,1);
RY=zeros(c,1);
RZ=zeros(c,1);
dx=zeros(c,1);
dy=zeros(c,1);
dz=zeros(c,1);
rx=zeros(c,1);
ry=zeros(c,1);
rz=zeros(c,1);
mx=zeros(j_max,1);
my=zeros(j_max,1);
mz=zeros(j_max,1);
mrx=zeros(j_max,1);
mry=zeros(j_max,1);
mrz=zeros(j_max,1);
dxt=zeros(c,1);
dyt=zeros(c,1);
dzt=zeros(c,1);
rxt=zeros(c,1);
ryt=zeros(c,1);
rzt=zeros(c,1);
mxt=zeros(j_max,1);
myt=zeros(j_max,1);
mzt=zeros(j_max,1);
mrxt=zeros(j_max,1);
mryt=zeros(j_max,1);
mrzt=zeros(j_max,1);
%%
for k=1:k_max
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',l+5*(k-1));
M2=load(scanName2);
ptXYZ2=pointCloud(M2.SCAN.XYZ');
ptXYZ2.Normal=pcnormals(ptXYZ2);
for j=1:j_max
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',l+5*(k-1)+j);
M1=load(scanName1);
ptXYZ1=pointCloud(M1.SCAN.XYZ');
ptXYZ1.Normal=pcnormals(ptXYZ1);
[xyz1,nxyz1,gr1,ngr1,pr1,plane1,Vr1]=ransac_plane(ptXYZ1.Location,ptXYZ1.Normal,ptXYZ1.Count,10);
ptXYZ1ot=pointCloud(nxyz1,'Normal',ngr1);
[tform,ptXYZ2r] = pcregrigid(ptXYZ2,ptXYZ1);
el1=affine3dtoeul(tform);
x=tform.T(4,1);
y=tform.T(4,2);
z=tform.T(4,3);
qx=el1(1,3);
qy=el1(1,2);
qz=el1(1,1);
for i=1:c
ax=(pi/9)*(1-2*rand);Tx=1-2*rand;
ay=(pi/9)*(1-2*rand);Ty=1-2*rand;
az=(pi/9)*(1-2*rand);Tz=1-2*rand;
Rx=[1 0 0;0 cos(ax) sin(ax);0 -sin(ax) cos(ax)];
Ry=[cos(ay) 0 -sin(ay);0 1 0;sin(ay) 0 cos(ay)];
Rz=[cos(az) sin(az) 0;-sin(az) cos(az) 0;0 0 1];
R=Rz*Ry*Rx;
T=[Tx Ty Tz 1];
A=[R,[0 0 0]';T];
tfr=affine3d(A);
ptXYZ2r=pctransform(ptXYZ2,tfr);
%%
[xyz2,nxyz2,gr2,ngr2,pr2,plane2,Vr2]=ransac_plane(ptXYZ2r.Location,ptXYZ2r.Normal,ptXYZ2r.Count,10);
ptXYZ2rot=pointCloud(nxyz2,'Normal',ngr2);
%%
[R12,eul]=find_rotation(plane1,plane2);
t=-mean(ptXYZ1.Location)*R12'+mean(ptXYZ2r.Location);
B=[R12,[0 0 0]';[t,1]];
tf12=affine3d(B);
ptXYZ2rr=pctransform(ptXYZ2r,tf12);
ptXYZ22rot=pctransform(ptXYZ2rot,tf12);
%%
tic;[tform1,ptXYZ2rott] = pcregrigid(ptXYZ22rot,ptXYZ1ot);
ptXYZ22r = pctransform(ptXYZ2rr,tform1);
tf1=pcregrigid(ptXYZ22r,ptXYZ1);t1=toc;
tic;[tform2,ptXYZ2rt]=pcregrigid(ptXYZ2r,ptXYZ1);t2=toc;
tf2=pcregrigid(ptXYZ2rt,ptXYZ1);
eul1 = affine3dtoeul(tform1);
eul2 = affine3dtoeul(tf1);
eul3 = affine3dtoeul(tf2);
%%
tx(i,1)=tform1.T(4,1);
ty(i,1)=tform1.T(4,2);
tz(i,1)=tform1.T(4,3);
sx(i,1)=eul1(1,3);
sy(i,1)=eul1(1,2);
sz(i,1)=eul1(1,1);
TX(i,1)=Tx;
TY(i,1)=Ty;
TZ(i,1)=Tz;
RX(i,1)=ax;
RY(i,1)=ay;
RZ(i,1)=az;
dx(i,1)=tf1.T(4,1);
dy(i,1)=tf1.T(4,2);
dz(i,1)=tf1.T(4,3);
rx(i,1)=eul2(1,3);
ry(i,1)=eul2(1,2);
rz(i,1)=eul2(1,1);
dxt(i,1)=tf2.T(4,1);
dyt(i,1)=tf2.T(4,2);
dzt(i,1)=tf2.T(4,3);
rxt(i,1)=eul3(1,3);
ryt(i,1)=eul3(1,2);
rzt(i,1)=eul3(1,1);
%toc;
end
%%
% file_name1=[file_path1 num2str(l+5*(k-1)) '-' num2str(l+5*(k-1)+j) '.mat'];
% save(file_name1,'x','y','z','qx','qy','qz','tx','ty','tz','sx','sy','sz','TX','TY','TZ','RX','RY','RZ','dx','dy','dz','rx','ry','rz','dxt','dyt','dzt','rxt','ryt','rzt');
%%
mx(j,1)=sqrt(mean(dx.^2));
my(j,1)=sqrt(mean(dy.^2));
mz(j,1)=sqrt(mean(dz.^2));
mrx(j,1)=sqrt(mean(rx.^2));
mry(j,1)=sqrt(mean(ry.^2));
mrz(j,1)=sqrt(mean(rz.^2));
mxt(j,1)=sqrt(mean(dxt.^2));
myt(j,1)=sqrt(mean(dyt.^2));
mzt(j,1)=sqrt(mean(dzt.^2));
mrxt(j,1)=sqrt(mean(rxt.^2));
mryt(j,1)=sqrt(mean(ryt.^2));
mrzt(j,1)=sqrt(mean(rzt.^2));
end
% file_name2=[file_path2 num2str(l+5*(k-1)) '-' num2str(l+5*(k-1)+j) '.mat'];
% save(file_name2,'mx','my','mz','mrx','mry','mrz','mxt','myt','mzt','mrxt','mryt','mrzt');
end
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
pcshowpair(ptXYZ1,ptXYZ2rt);
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
%%
% figure(1)
% subplot(3,2,1)
% title('TX')
% xlabel('number of experiments');
% ylabel('Estimated TX')
% hold all
% plot(dx,'o');
% plot(TX,'*');
% plot(x*ones(100,1));
% subplot(3,2,2)
% title('TY')
% xlabel('number of experiments');
% ylabel('Estimated TY')
% hold all
% plot(dy,'o');
% plot(TY,'*');
% plot(y*ones(100,1));
% subplot(3,2,3)
% title('TZ')
% xlabel('number of experiments');
% ylabel('Estimated TZ')
% hold all
% plot(dz,'o');
% plot(TZ,'*');
% plot(z*ones(100,1));
% subplot(3,2,4)
% title('RX')
% xlabel('number of experiments');
% ylabel('Estimated RX')
% hold all
% plot(rx,'o');
% plot(RX,'*');
% plot(qx*ones(100,1));
% subplot(3,2,5)
% title('RY')
% xlabel('number of experiments');
% ylabel('Estimated RY')
% hold all
% plot(ry,'o');
% plot(RY,'*');
% plot(qy*ones(100,1));
% subplot(3,2,6)
% title('RZ')
% xlabel('number of experiments');
% ylabel('Estimated RZ')
% hold all
% plot(rz,'o');
% plot(RZ,'*');
% plot(qz*ones(100,1));
% %%
% figure(2)
% subplot(3,2,1)
% title('TX')
% xlabel('number of experiments');
% ylabel('Direct Estimated TX')
% hold all
% plot(dxt,'o');
% plot(TX,'*');
% plot(x*ones(100,1));
% subplot(3,2,2)
% title('TY')
% xlabel('number of experiments');
% ylabel('Estimated TY')
% hold all
% plot(dyt,'o');
% plot(TY,'*');
% plot(y*ones(100,1));
% subplot(3,2,3)
% title('TZ')
% xlabel('number of experiments');
% ylabel('Estimated TZ')
% hold all
% plot(dzt,'o');
% plot(TZ,'*');
% plot(z*ones(100,1));
% subplot(3,2,4)
% title('RX')
% xlabel('number of experiments');
% ylabel('Estimated RX')
% hold all
% plot(rxt,'o');
% plot(RX,'*');
% plot(qx*ones(100,1));
% subplot(3,2,5)
% title('RY')
% xlabel('number of experiments');
% ylabel('Estimated RY')
% hold all
% plot(ryt,'o');
% plot(RY,'*');
% plot(qy*ones(100,1));
% subplot(3,2,6)
% title('RZ')
% xlabel('number of experiments');
% ylabel('Estimated RZ')
% hold all
% plot(rzt,'o');
% plot(RZ,'*');
% plot(qz*ones(100,1));