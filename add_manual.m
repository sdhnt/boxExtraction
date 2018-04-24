clear all
close all
load('Planespc.mat')
% plot and check sides
% for i = 1:length(Planespc)    
%     oneFrame = Planespc{i};
%     figure(i);   
%     subplot(1,2,1)
%     pcshow(oneFrame.plane1)
%     title([num2str(oneFrame.frameNum),'-plane1'])
%      subplot(1,2,2)
%     pcshow(oneFrame.plane2)
%     title([num2str(oneFrame.frameNum),'-plane2'])
%     drawnow    
% end

% set sp1 and sp2 manually
%  fig 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
sp1 = [1,3,3,0,4,4,2,3,3,3,4,4,2,1,0,0,0,0,3,3];
sp2 = [4,0,0,0,0,2,0,2,0,0,0,2,4,2,0,0,0,2,2,0];
for i = 1:length(Planespc)
    Planespc{i}.side_plane1 = sp1(i);
    Planespc{i}.side_plane2 = sp2(i);
end
save('Planespc.mat','Planespc');

%     if ismember(frameNum, [])
%         checkframe = 1;
%     elseif ismember(frameNum, [])
%         checkframe = 2;
%     else
%         checkframe = 0;
%     end
