clear all;
close all;

load("scalaire.dat");
load("tempsR.dat");





scalaire(find(scalaire==0))=[];
tempsR(find(scalaire==0))=[];

figure;


plot(tempsR(1:600),scalaire,'LineWidth',4,'Color',[0 0 1])
xlabel('Time','FontSize',18);
ylabel('Scalar product','FontSize',18);