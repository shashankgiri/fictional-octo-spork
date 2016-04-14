function plot_plane_fit(XYZ,xyz,N)
xLim = [min(XYZ(:,1)) max(XYZ(:,1))];
yLim = [min(XYZ(:,2)) max(XYZ(:,2))];
[u,v] = meshgrid(xLim,yLim);
w=-(N(1,1)*u+N(1,2)*v+N(1,4))/N(1,3);
reOrder = [1 2  4 3];
% figure(2*i-1)
% hold on
% title('Estimated Normals to the ground plane')
% quiver3(xyz(:,1)',xyz(:,2)',xyz(:,3)',G(:,1)',G(:,2)',G(:,3)');
% set(gca,'color','black');
% patch(u(reOrder),v(reOrder),w(reOrder),'r');
% hold off
% grid on;
% alpha(0.3);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
figure(1)
hold on
title('Estimated Plane to the normal points')
scatter3(xyz(:,1),xyz(:,2),xyz(:,3),'g','.');
set(gca,'color','black');
patch(u(reOrder),v(reOrder),w(reOrder),'b');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(2)
hold on
title('Estimated Plane to the Point Cloud')
scatter3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'g','.');
set(gca,'color','black');
patch(u(reOrder),v(reOrder),w(reOrder),'b');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
end