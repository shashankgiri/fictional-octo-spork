figure(1)
subplot(3,2,1)
title('TX')
xlabel('Number of Trails');
ylabel('Estimated TX')
hold all
plot(tx+TX,'o');
plot(TX,'*');
%plot(x*ones(100,1));
subplot(3,2,2)
title('TY')
xlabel('Number of Trails');
ylabel('Estimated TY')
hold all
plot(ty+TY,'o');
plot(TY,'*');
%plot(y*ones(100,1));
subplot(3,2,3)
title('TZ')
xlabel('Number of Trails');
ylabel('Estimated TZ')
hold all
plot(tz+TZ,'o');
plot(TZ,'*');
%plot(z*ones(100,1));
subplot(3,2,4)
title('RX')
xlabel('Number of Trails');
ylabel('Estimated RX');
hold all
plot(rx-RX,'o');
plot(RX,'*');
%plot(qx*ones(100,1));
subplot(3,2,5)
title('RY')
xlabel('Number of Trails');
ylabel('Estimated RY');
hold all
plot(ry-RY,'o');
plot(RY,'*');
%plot(qy*ones(100,1));
subplot(3,2,6)
title('RZ')
xlabel('Number of Trails');
ylabel('Estimated RZ');
hold all
plot(rz-RZ,'o');
plot(RZ,'*');
%plot(qz*ones(100,1));
% %%
% figure(2)
% subplot(3,2,1)
% title('TX')
% xlabel('number of experiments');
% ylabel('Direct Estimated TX')
% hold all
% plot(dxt,'o');
% plot(TX,'*');
% plot(x*ones(100,1));
% subplot(3,2,2)
% title('TY')
% xlabel('number of experiments');
% ylabel('Estimated TY')
% hold all
% plot(dyt,'o');
% plot(TY,'*');
% plot(y*ones(100,1));
% subplot(3,2,3)
% title('TZ')
% xlabel('number of experiments');
% ylabel('Estimated TZ')
% hold all
% plot(dzt,'o');
% plot(TZ,'*');
% plot(z*ones(100,1));
% subplot(3,2,4)
% title('RX')
% xlabel('number of experiments');
% ylabel('Estimated RX')
% hold all
% plot(rxt,'o');
% plot(RX,'*');
% plot(qx*ones(100,1));
% subplot(3,2,5)
% title('RY')
% xlabel('number of experiments');
% ylabel('Estimated RY')
% hold all
% plot(ryt,'o');
% plot(RY,'*');
% plot(qy*ones(100,1));
% subplot(3,2,6)
% title('RZ')
% xlabel('number of experiments');
% ylabel('Estimated RZ')
% hold all
% plot(rzt,'o');
% plot(RZ,'*');
% plot(qz*ones(100,1));