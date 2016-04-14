 function plot_normal(XYZ,norm)
   figure
   title('Estimated Normals with Point Cloud')
   hold on
   scatter3(XYZ(:,1)',XYZ(:,2)',XYZ(:,3)','r','.');
   set(gca,'color','black');
   quiver3(XYZ(:,1),XYZ(:,2),XYZ(:,3),norm(:,1),norm(:,2),norm(:,3),'b');
   set(gca,'color','black');
   hold off
   grid on;
   alpha(0.3);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   figure
   title('Estimated Normals of Point Cloud')
   hold on
   quiver3(XYZ(:,1),XYZ(:,2),XYZ(:,3),norm(:,1),norm(:,2),norm(:,3));
   set(gca,'color','black');
   hold off
   grid on;
   alpha(0.3);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
 end