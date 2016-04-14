%function normal_estimation(folder, scanIndex)
clear D e E H i in K norm QR V XYZ P p;
close all;
%scanName = sprintf('%s/SCANS/Scan%04d.mat',folder,scanIndex);
   M=load('E:\Books\Thesis\mat-files\SCANS\Scan0365');
   X=M.SCAN.XYZ(1,:);
   Y=M.SCAN.XYZ(2,:);
   Z=M.SCAN.XYZ(3,:);
   r=0.01;
   o=1;
   n=length(X);
   U=ones(3,1);
   H=zeros(o,3);
   norm=zeros(3,n);
   XYZ=[X' Y' Z'];
   valid_flag=zeros(n,1);
   for i=1:n
   P=XYZ(i,:);
   E=((XYZ-repmat(P,n,1)).^2)*U-repmat(r,n,1);
   for j=1:n
   if E(j,1)<0
   valid_flag(j)=valid_flag(j)+1;
   end
   end
   K=find(valid_flag);
   valid_flag=zeros(n,1);
   o=size(K,1);
   for l=1:o
    H(l,:)=XYZ(K(l,1),:); 
   end
   p=[mean(H(:,1)) mean(H(:,2)) mean(H(:,3))];
   q=size(H,1);
   R=((H'-repmat(p',1,q))*(H'-repmat(p',1,q))')/q;
   [V,D] = eig(R);
   e=diag(D);
   in=find(min(e));
   norm(:,i)=V(:,in);
   end
   figure(1)
   title('Estimated Normals with Point Cloud')
   hold on
   scatter3(X',Y',Z','r','.');
   set(gca,'color','black');
   quiver3(X,Y,Z,norm(1,:),norm(2,:),norm(3,:),'b');
   set(gca,'color','black');
   hold off
   grid on;
   alpha(0.3);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   figure(2)
   title('Estimated Normals of Point Cloud')
   hold on
   quiver3(X,Y,Z,norm(1,:),norm(2,:),norm(3,:));
   set(gca,'color','black');
   hold off
   grid on;
   alpha(0.3);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   %end