%[XYZ1,norm1,n1]=normal_estimation('I:\thesis\mat-files',365);
M1=load('I:\thesis\mat-files\SCANS\Scan0365.mat');
M2=load('I:\thesis\mat-files\SCANS\Scan0366.mat');
ptXYZ1=pointCloud(M1.SCAN.XYZ');
ptXYZ2=pointCloud(M2.SCAN.XYZ');
ptXYZ1.Normal=pcnormals(ptXYZ1);
ptXYZ2.Normal=pcnormals(ptXYZ2);
%[XYZ2,norm2,n2]=normal_estimation('I:\thesis\mat-files',366);
%%
ax=pi/12;Tx=1;
ay=pi/15;Ty=-1;
az=pi/12;Tz=2;
Rx=[1 0 0;0 cos(ax) sin(ax);0 -sin(ax) cos(ax)];
Ry=[cos(ay) 0 -sin(ay);0 1 0;sin(ay) 0 cos(ay)];
Rz=[cos(az) sin(az) 0;-sin(az) cos(az) 0;0 0 1];
R=Rz*Ry*Rx;
XYZR2=XYZ2*R;
T=[1 0 0 Tx;0 1 0 Ty;0 0 1 Tz;0 0 0 1];
nr=length(XYZR2);
XYZR2=[XYZR2';ones(1,nr)];
XYZR2=T*XYZR2;
XYZR2=[XYZR2(1,:);XYZR2(2,:);XYZR2(3,:)]';

%%
r=0.01;
o=1;
Ur=ones(3,1);
Hr=zeros(o,3);
normr=zeros(3,nr);
valid_flag=zeros(nr,1);
 for i=1:nr
 P=XYZR2(i,:);
 E=((XYZR2-repmat(P,nr,1)).^2)*Ur-repmat(r,nr,1);
 for j=1:nr
 if E(j,1)<0
 valid_flag(j)=valid_flag(j)+1;
 end
 end
 K=find(valid_flag);
 valid_flag=zeros(nr,1);
 o=size(K,1);
 for l=1:o
 Hr(l,:)=XYZR2(K(l,1),:); 
 end
 p=[mean(Hr(:,1)) mean(Hr(:,2)) mean(Hr(:,3))];
 q=size(Hr,1);
 cov=((Hr'-repmat(p',1,q))*(Hr'-repmat(p',1,q))')/q;
 [Vr,~] = eig(cov);
 normr(:,i)=Vr(:,1);
 end
 %%
 [xyz1,nxyz1,gr1,ngr1,pr1,Nr1,Vr1]=ransac_plane(XYZ1,norm1,n1,50);
 
 %[xyz2,nxyz2,gr2,pr2,Nr2,Vr2]=ransac_plane(XYZ2,norm2,n2,50);

 [xyz2r,nxyz2r,gr2r,pr2r,Nr2r,Vr2r]=ransac_plane(XYZR2,normr,nr,50);
 %%
 [R12r]=find_rotation(Nr1,Nr2r);
 %[R22r]=find_rotation(Nr2,Nr2r);
  t=-R12r*mean(XYZ1)'+mean(XYZR2)';
  XYZt=XYZR2*R12r+repmat(t',nr,1);
  
  %%
  r=0.01;
  o=1;
  Ut=ones(3,1);
  Ht=zeros(o,3);
  normt=zeros(3,nr);
  valid_flagt=zeros(nr,1);
 for i=1:nr
 Pt=XYZt(i,:);
 Et=((XYZt-repmat(Pt,nr,1)).^2)*Ut-repmat(r,nr,1);
 for j=1:nr
 if Et(j,1)<0
 valid_flagt(j)=valid_flagt(j)+1;
 end
 end
 Kt=find(valid_flagt);
 valid_flagt=zeros(nr,1);
 ot=size(Kt,1);
 for lt=1:ot
 Ht(lt,:)=XYZt(Kt(l,1),:); 
 end
 pt=[mean(Ht(:,1)) mean(Ht(:,2)) mean(Ht(:,3))];
 qt=size(Ht,1);
 covt=((Ht'-repmat(pt',1,qt))*(Ht'-repmat(pt',1,qt))')/qt;
 [Vt,~] = eig(covt);
 normt(:,i)=Vt(:,1);
 end
 %%
%  [xyzt,nxyzt,gt,pt,Nt,Vt]=ransac_plane(XYZt,normt,nr,50);
%  %%
%  plot_plane_fit(XYZR2,xyz2r,Nr2r,pr2r,gr2r,1);
%  hold on
%  plot_plane_fit(XYZ1,xyz1,Nr1,pr1,gr1,3);
%  %%
% xLim = [min(XYZ1(:,1)) max(XYZ1(:,1))];
% yLim = [min(XYZ1(:,2)) max(XYZ1(:,2))];
% [u1,v1] = meshgrid(xLim,yLim);
% w1=-(Nr1(1,1)*u1+Nr1(2,1)*v1-pr1*Nr1)/Nr1(3,1);
% reOrder = [1 2  4 3];
% figure(1)
% hold on
% title('Estimated Plane to the normal points')
% scatter3(XYZ1(:,1)',XYZ1(:,2)',XYZ1(:,3)','r','.');
% set(gca,'color','black');
% patch(u1(reOrder),v1(reOrder),w1(reOrder),'b');
% hold off
% grid on;
% alpha(0.3);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% xLim = [min(XYZR2(:,1)) max(XYZR2(:,1))];
% yLim = [min(XYZR2(:,2)) max(XYZR2(:,2))];
% [ur2,vr2] = meshgrid(xLim,yLim);
% wr2=-(Nr2r(1,1)*ur2+Nr2r(2,1)*vr2-pr2r*Nr2r)/Nr2r(3,1);
% hold on
% title('Estimated Plane to the Point Cloud')
% scatter3(XYZR2(:,1)',XYZR2(:,2)',XYZR2(:,3)','y','.');
% set(gca,'color','black');
% patch(ur2(reOrder),vr2(reOrder),wr2(reOrder),'g');
% hold off
% grid on;
% alpha(0.3);
%%
% xLim = [min(XYZ1(:,1)) max(XYZ1(:,1))];
% yLim = [min(XYZ1(:,2)) max(XYZ1(:,2))];
% [u1,v1] = meshgrid(xLim,yLim);
% w1=-(Nr1(1,1)*u1+Nr1(2,1)*v1-pr1*Nr1)/Nr1(3,1);
% reOrder = [1 2  4 3];
% figure(1)
% %hold on
% %title('Estimated Plane to the normal points')
% %scatter3(XYZ1(:,1)',XYZ1(:,2)',XYZ1(:,3)','r','.');
% %set(gca,'color','black');
% %patch(u1(reOrder),v1(reOrder),w1(reOrder),'b');
% %hold off
% %grid on;
% %alpha(0.3);
% %xlabel('X');
% %ylabel('Y');
% %zlabel('Z');
% xLim = [min(XYZt(:,1)) max(XYZt(:,1))];
% yLim = [min(XYZt(:,2)) max(XYZt(:,2))];
% [ut,vt] = meshgrid(xLim,yLim);
% wt=-(Nt(1,1)*ut+Nt(2,1)*vt-pt*Nt)/Nt(3,1);
% hold on
% title('Estimated Plane to the Point Cloud')
% scatter3(XYZt(:,1)',XYZt(:,2)',XYZt(:,3)','c','.');
% set(gca,'color','black');
% patch(ut(reOrder),vt(reOrder),wt(reOrder),'m');
% hold off
% grid on;
% alpha(0.3);
% %%
% figure(1)
% hold on
% scatter3(XYZtt(:,1)',XYZtt(:,2)',XYZtt(:,3)','c','.');
% set(gca,'color','black');
% hold off
% grid on;
% alpha(0.3);
% figure(1)
% hold on
% title('Estimated Plane to the normal points')
% scatter3(XYZ1(:,1)',XYZ1(:,2)',XYZ1(:,3)','r','.');
% set(gca,'color','black');
% hold off
% grid on;
% alpha(0.3);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% %%
