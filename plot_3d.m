   basisx=[1 0 0];
   [xyz,ZZ,Nn,Vb]=plane_fit(norm,basisx,n,XYZ);
   x=xyz;
   Nx=Nn;
   xx=ZZ;
   Vbx=Vb;
   basisy=[0 1 0];
   [xyz,ZZ,Nn,Vb]=plane_fit(norm,basisy,n,XYZ);
   y=xyz;
   Ny=Nn;
   yy=ZZ;
   Vby=Vb;
   basisz=[0 0 1];
   [xyz,ZZ,Nn,Vb]=plane_fit(norm,basisz,n,XYZ);
   z=xyz;
   Nz=Nn;
   zz=ZZ;
   Vbz=Vb;
%%
M2=load('I:\thesis\mat-files\SCANS\Scan0366.mat');
X2=M2.SCAN.XYZ(1,:);
Y2=M2.SCAN.XYZ(2,:);
Z2=M2.SCAN.XYZ(3,:);
XYZ2=[X2' Y2' Z2'];
[xyz,norm,a,nn,VV]=normal_estimation('I:\thesis\mat-files',366);
xyz2=xyz;
norm2=norm;
n2=a;
nn2=nn;
VV2=VV;
%%
figure
scatter3(XYZ1(:,1)',XYZ1(:,2)',XYZ1(:,3)','g','.');
hold on
scatter3(xyz22(:,1)',xyz22(:,2)',xyz22(:,3)','r','.');
%% optimize
xLim = [min(X) max(X)];
yLim = [min(Y) max(Y)];
[uu,vv] = meshgrid(xLim,yLim);
ww=-(nnn(1,1)*uu+nnn(2,1)*vv-ppp*nnn)/nnn(3,1);
reOrder = [1 2  4 3];
figure(6)
hold on
title('Estimated Normals to the ground plane')
quiver3(xyz(:,1)',xyz(:,2)',xyz(:,3)',ZZ(:,1)',ZZ(:,2)',ZZ(:,3)');
set(gca,'color','black');
patch(uu(reOrder),vv(reOrder),ww(reOrder),'r');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(7)
hold on
title('Estimated Plane to the normal points')
scatter3(xyz(:,1)',xyz(:,2)',xyz(:,3)','r','.');
set(gca,'color','black');
patch(uu(reOrder),vv(reOrder),ww(reOrder),'b');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(8)
hold on
title('Estimated Plane to the Point Cloud')
scatter3(X',Y',Z','r','.');
set(gca,'color','black');
patch(uu(reOrder),vv(reOrder),ww(reOrder),'b');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
%% normal_estimation
 %%
