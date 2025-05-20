clc;
clear all;
close all;
d = load('openloop_rider_forward0.05_webots_data.txt');
d1 = load('openloop_rider_backward0.1_webots_data.txt');
d2 = load('openloop_rider_stright_webots_data.txt');
d3 = load('closeloop_pd_rider_stright_webots_data.txtt');
d4 = load('closeloop_pd_rider_backward0.1_webots_data.txtt');
d5 = load('closeloop_pd_rider_forward0.1_webots_data.txtt');


t = d(:,1);
t1 = d1(:,1);
t2 = d2(:,1);
t3 = d3(:,1);
t4 = d4(:,1);
t5 = d5(:,1);
susp = d(:,5);
susp1 = d1(:,5);
susp2 = d2(:,5);
susp3 = d3(:,5);
susp4 = d4(:,5);
susp5 = d5(:,5);
pitch = d(:,3);
pitch1 = d1(:,3);
pitch2 = d2(:,3);
pitch3 = d3(:,3);
pitch4 = d4(:,3);
pitch5 = d5(:,3);

figure 
plot(t,susp,'k',t1,susp1,'b',t2,susp2,'r');
legend('rider forward','rider backward','rider stright');
xlabel('Time(s)');
ylabel('Suspension compression(m)');
title('comparsion of Suspension compression under open-loop controller for different rider position');

figure 
plot(t,pitch,'k',t1,pitch1,'b',t2,pitch2,'r');
legend('rider forward','rider backward','rider stright');
xlabel('Time(s)');
ylabel('pitch angle(rad)');
title('comparsion of pitch angle under open-loop controller for different rider position');
figure 
plot(t3,pitch3,'r',t4,pitch4,'b',t5,pitch5,'k');
legend('rider still','rider backward','rider forward');
xlabel('Time(s)');
ylabel('pitch angle(rad)');
title('comparsion of pitch angle under PD pitch angle controller for different rider position');
