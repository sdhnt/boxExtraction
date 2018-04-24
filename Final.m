close all;
clear all;
load('box_side4.mat')
s4 = ptCloudScene;
load('box_side2.mat')
s2 = ptCloudScene;
% figure
% pcshow(s4)
% figure
% pcshow(s2)
mergeSize=0.001;
tf=pcregrigid(s4,s2,'Metric','pointToPlane',...
        'Extrapolate', true,...
        'Tolerance',[0.00001, 0.00005],...
        'MaxIteration',3000);
    s4=pctransform(s4,tf);
pcshow(pcmerge(s2,s4,mergeSize));