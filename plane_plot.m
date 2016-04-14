[plane1,~,~] = pcfitplane(ptc1,1,[0,0,1],1);
[plane2,~,~] = pcfitplane(ptc2,1,[0,0,1],1);
xyz=ptc1.Location;
u=plane1.Normal(1,1);
v=plane1.Normal(1,2);
w=plane1.Normal(1,3);
%%

reOrder = [1 2  4 3];
figure
hold on
title('Estimated Plane to the normal points')
scatter3(xyz(:,1),xyz(:,2),xyz(:,3),'r','.');
set(gca,'color','black');
patch(u(reOrder),v(reOrder),w(reOrder),'b');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
%%
plot_normal(pts1.Location,pts1.Normal,1)