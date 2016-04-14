clear pp RR VV nn;
pp = mean(xyz,1);
RR = bsxfun(@minus,xyz,p);
[VV,~] = eig(RR'*RR);
nn = VV(:,1);
VV = VV(:,2:end);
%%
xLim = [min(X) max(X)];
yLim = [min(Y) max(Y)];
[u,v] = meshgrid(xLim,yLim);
w=-(nn(1,1)*u+nn(2,1)*v+pp*nn)/nn(3,1);
reOrder = [1 2  4 3];
figure(2);
scatter3(xyz(:,1)',xyz(:,2)',xyz(:,3)','r','.');
set(gca,'color','black');
hold on
patch(u(reOrder),v(reOrder),w(reOrder),'b');
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(3)
scatter3(X',Y',Z','b','.');
set(gca,'color','black');
hold on
patch(u(reOrder),v(reOrder),w(reOrder),'r');
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
