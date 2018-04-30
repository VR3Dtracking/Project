

load("posX.dat");
load("tempsR.dat");
load("PosAccelX.dat");
load("PosKalmanX.dat");
load("PosCalcX.dat");



PosAccelX(find(tempsR==0))=[];
posX(find(tempsR==0))=[];
PosCalcX(find(tempsR==0))=[];
PosKalmanX(find(tempsR==0))=[];
tempsR(find(tempsR==0))=[];

figure;
subplot(3,1,1)

plot(tempsR,posX,'b')
hold on;
plot(tempsR,PosCalcX,'r')
%ylim([-10 100])
ylabel("position")
xlabel("time (second)");
legend("Real Position","Position by scanning")
title("Position derived from the photodiode sensor")
subplot(3,1,3)
plot(tempsR,posX,'b')
hold on;
plot(tempsR,PosKalmanX,'g')
%ylim([-10 100])
ylabel("position")
xlabel("time (second)");
legend("Real Position","Position by Fusion")
title("Position derived from the fusion using Kalman filter")
subplot(3,1,2)
plot(tempsR,posX,'b')
hold on;
plot(tempsR,PosAccelX,'k')
xlabel("time (second)");
ylabel("position")
%ylim([-10 100])
legend("Real Position","Position by accelerometer")
title("Position derived from the accelerometer sensor")