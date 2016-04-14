function norm = normal_estimation(XYZ,n)
clear D e E H i in K norm QR V P p pp RR VV nn N xyz;
close all;
   r=0.01;
   o=1;
   U=ones(3,1);
   H=zeros(o,3);
   norm=zeros(3,n);
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
   [V,~] = eig(R);
   norm(:,i)=V(:,1);
   end
  norm=norm';
end