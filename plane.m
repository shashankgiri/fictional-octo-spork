
%function plane_fit(folder, scanIndex)
close all;
clear H E A A1 B1 C D G K T e valid_flag max_num M k P1 P2 q Q u v w ;
clc
%scanName = sprintf('%s/SCANS/Scan%04d.mat',folder,scanIndex);
iter=4;
N=1;
M=load('E:\Books\Thesis\mat-files\SCANS\Scan0365.mat');
   X=M.SCAN.XYZ(1,:);
   Y=M.SCAN.XYZ(2,:);
   Z=M.SCAN.XYZ(3,:);
   n=length(X);
   Q=ones(n,1);
   XYZ=[X' Y' Z'];
   A1=[XYZ Q]';
   T=XYZ';
   score=zeros(iter,1);
    valid_flag=zeros(n,iter);
P=zeros(3,3);
while N<5
    clear H B1 e E G max_num;
    
for i= 1:iter
     
    A=randi(n,3,1);
    for j=1:3;
       P(j,:)=XYZ(A(j,1),:); 
    end
   P1(i,:)=P(2,:)-P(1,:); 
   P2(i,:)=P(3,:)-P(1,:);
   C(i,:)=cross(P1(i,:),P2(i,:));
   D(i)=dot(P(1,:),C(i,:));
   for k= 1:n
       E(k)=(C(i,:)*T(:,k)-D(i))/sqrt(C(i,1)^2+C(i,2)^2+C(i,3)^2);
       if   -0.1<E(k)<0.1
           score(i,:)=score(i,:)+1;
           valid_flag(k,i)=valid_flag(k,i)+1;
       end
      
   end
end
 max_num=max(score);
[x,~]=ind2sub(size(score),find(score==max_num));
K=find(valid_flag(:,x));
[o,~]=size(K);
for l=1:o
    H(l,:)=XYZ(K(l,1),:);  
    e(:,l)=E(:,K(l,1))';
end
%G=inv(A1*A1')*A1*E';
%g=G';
q=ones(o,1);
B1=[H q];
%for m=1:16
%valid_flag=zeros(n,iter);
%score=zeros(iter,1);
%I=mean(E.^2);
%D=D-repmat(I,1,4);
G=inv(B1'*B1)*B1'*e';
%a=G(1,1);
%b=G(2,1);
%c=G(3,1);
%d=G(4,1);
%for t=1:n;
%S(t)=([a b c -d]*A1(:,t))/sqrt(a^2+b^2+c^2);

N=N+1;
%end
%G(:,m)=inv(A1*A1')*A1*S';
%end
%g=G(4,:);
%%
xLim = [min(X) max(X)];
yLim = [min(Y) max(Y)];
[u,v] = meshgrid(xLim,yLim);
%w=(C(x,1)*u+C(x,2)*v+D(x))/C(x,3);
w=(G(1,1)*u+G(2,1)*v+G(4,1))/G(3,1);
reOrder = [1 2  4 3];
figure(N);
scatter3(X',Y',Z','b','.');
set(gca,'color','black');
hold on
patch(u(reOrder),v(reOrder),w(reOrder),'r');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(N+4)
scatter3(H(:,1),H(:,2),H(:,3),'r','.');
set(gca,'color','black');
hold on
patch(u(reOrder),v(reOrder),w(reOrder),'b');
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
end
