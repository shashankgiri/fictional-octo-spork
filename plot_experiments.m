figure
subplot(3,2,1)
title('TX')
xlabel('number of scans');
ylabel('Estimated TX')
hold all
plot(dx,'o');
plot(dxt,'*');
subplot(3,2,2)
title('TY')
xlabel('number of scans');
ylabel('Estimated TY')
hold all
plot(dy,'o');
plot(dyt,'*');
subplot(3,2,3)
title('TZ')
xlabel('number of scans');
ylabel('Estimated TZ')
hold all
plot(dz,'o');
plot(dzt,'*');
subplot(3,2,4)
title('RX')
xlabel('number of scans');
ylabel('Estimated RX')
hold all
plot((rx*180/pi),'o');
plot((rxt*180/pi),'*');
subplot(3,2,5)
title('RY')
xlabel('number of scans');
ylabel('Estimated RY')
hold all
plot((ry*180/pi),'o');
plot((ryt*180/pi),'*');
subplot(3,2,6)
title('RZ')
xlabel('number of scans');
ylabel('Estimated RZ')
hold all
plot((rz*180/pi),'o');
plot((rzt*180/pi),'*');
