clear all
close all

load('Planespc.mat');
successFrames=Planespc;

side_used = 4; %%The side which is to be used as the large plane for reference

gridSize = 0.0015;
gridStep = 0.02;
mergeSize = 0.001;

ctr = 1;
n_corner = 1;
xprevrms=0;
yprevrms=0;
xprevcorner=0;
yprevcorner=0;
xprevangle=0;
yprevangle=0;
accumTform = [];
rmse=0;

found_ref = 0;
for i = 1:length(successFrames)%% Go through all the verified frames
    
    % find first frame with large side_used and initialize values with it
    if found_ref == 0
        if successFrames{i}.side_plane1 == side_used     
            planeScene = successFrames{i}.plane1;
            planeNorm = successFrames{i}.norms{1};
        elseif successFrames{i}.side_plane2 == side_used   
            planeScene = successFrames{i}.plane2;
            planeNorm = successFrames{i}.norms{2};
        else
            continue
        end
        ptCloudScene = successFrames{i}.box;            
        found_ref = 1;
        
        % initialise variables
        normals{ctr}=successFrames{i}.planeParameters{1};
        corners{n_corner} = successFrames{i}.corners(1,:);
        moving = pcdownsample(ptCloudScene, 'gridAverage', gridSize);
        moving_plane = pcdownsample(planeScene, 'gridAverage', gridSize);
        moving_planeNorm = planeNorm;

        figure
        hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
        title('Updated world scene')
        % Set the axes property for faster rendering
        hAxes.CameraViewAngleMode = 'auto';
        hScatter = hAxes.Children;

        % Visualize the plane scene.            
        figure
        hAxes2 = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
        title('Updated plane scene')
        % Set the axes property for faster rendering
        hAxes2.CameraViewAngleMode = 'auto';
        hScatter2 = hAxes2.Children;
        continue
    end
    if successFrames{i}.side_plane1 == side_used
        planeCurrent = successFrames{i}.plane1;
        ptCloudCurrent = successFrames{i}.box;
        planeNorm = successFrames{i}.norms{1};
    elseif successFrames{i}.side_plane2 == side_used
        planeCurrent = successFrames{i}.plane2;
        ptCloudCurrent = successFrames{i}.box;
        planeNorm = successFrames{i}.norms{2};
    else
        continue
    end
    % Use previous moving point cloud as reference.
    fixed = moving;
    fixed_plane = moving_plane;  
    fixed_planeNorm = moving_planeNorm;
    
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    moving_plane = pcdownsample(planeCurrent, 'gridAverage', gridSize);
    moving_planeNorm = planeNorm;
    
    % Apply ICP registration.
    tform = pcregrigid(moving_plane, fixed_plane, 'Metric','pointToPlane',...
        'Extrapolate', true,...
        'Tolerance',[0.00001, 0.00005],...
        'MaxIteration',300);
    
    moving_planeNorm_new = [moving_planeNorm,1] * tform.T;% For the fused plane
    moving_planeNorm_new = moving_planeNorm_new(1:3);
    
    if dot(moving_planeNorm_new, fixed_planeNorm) < 0  % Ignore the frame if fused in reverse (normals in opp direction)
        moving = fixed;
        moving_plane = fixed_plane;
        moving_planeNorm = fixed_planeNorm;
        continue;
    end
    
    planePrevNext=pctransform(planeCurrent, tform);
    planePrevNext=pcmerge(fixed_plane, planePrevNext, mergeSize); %Merged Successive Frames
    
    ptCloudPrevNext=pctransform(ptCloudCurrent, tform);
    ptCloudPrevNext=pcmerge(fixed, ptCloudPrevNext, mergeSize); %Merged Successive Large Planes
    
    if isempty(accumTform)
        accumTform = tform;
    else
        accumTform = affine3d(tform.T * accumTform.T); %Update the accumulated transform
    end
    planeAligned = pctransform(planeCurrent, accumTform);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);

    % Update the world scene.
    planeScene = pcmerge(planeScene, planeAligned, mergeSize);
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
    
    if(1)
        %%Caluclate errors in normals, corners and RMSE of fitted plane
        
        %% NORMALS ERROR CALCULATION
         ctr=ctr+1;
         normals{ctr}=successFrames{i}.planeParameters{1}*accumTform.T;
         tmpctr=0;
         totalangle=0;
         for m=1:length(normals)-1
            for n=(m+1):(length(normals)) 
            angle=acos( dot(normals{m} , normals{n})) ;%%IN Radians
            totalangle=totalangle+angle; tmpctr=tmpctr+1;    
            end
         end
         avgangle=totalangle/tmpctr;
          
        figure(300);
        subplot(3,1,1);hold on;
        if(xprevangle~=0 &&yprevangle~=0)  
        plot([successFrames{i}.frameNum,xprevangle], [avgangle,yprevangle], 'r-', 'LineWidth', 2);
        end
        xprevangle=successFrames{i}.frameNum; yprevangle=avgangle;      
        title('Avg. Anglular Error vs Num of Frames');

     
     %% Corner Error Average Calculation
        n_corner = n_corner + 1;
        corner1 = successFrames{i}.corners(1,:);
        corner2 = successFrames{i}.corners(2,:);
        dis1 = sum((corner1 - corners{1}).^2);
        dis2 = sum((corner2 - corners{1}).^2);
        if dis1 < dis2
            corners{n_corner}=corner1;
        else 
            corners{n_corner}=corner2;
        end
         tmpctr=0;
         totaldist=0;
         for m=1:length(corners)-1
            for n=(m+1):(length(corners)) 
                totaldist = totaldist + sqrt(sum((corners{m}-corners{n}).^2));
                tmpctr = tmpctr + 1;
            end
         end
         avgdist=totaldist/tmpctr;
     subplot(3,1,2);hold on;
        if(xprevcorner~=0 &&yprevcorner~=0)  
        plot([successFrames{i}.frameNum,xprevcorner], [avgdist,yprevcorner], 'r-', 'LineWidth', 2);
        end
        xprevcorner=successFrames{i}.frameNum; yprevcorner=avgdist;      
        title('Avg. Corner distance error vs Num of Frames')
      
    %% Plane Fitting and RMS 
    
        [model1,inlierIndices,outlierIndices,mse] = pcfitplane(planeScene,0.01);
        rmse=sqrt(mse);
subplot(3,1,3);hold on;
        if(xprevrms~=0 &&yprevrms~=0)  
        plot([successFrames{i}.frameNum,xprevrms], [rmse,yprevrms], 'r-', 'LineWidth', 2);
        end
        xprevrms=successFrames{i}.frameNum; yprevrms=rmse;  
        title('RMS Error vs Frame Number for Fused Cloud');       
    end
    
    %Visualize the successive frame stitching
    figure;
    subplot(1,2,1)
    pcshow(planePrevNext);
    title('Successive Planes Fused');    
    subplot(1,2,2)
    pcshow(ptCloudPrevNext);
    title('Successive Clouds Fused');
    
    
    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    drawnow('update')
     % Visualize the plane scene.
    hScatter2.XData = planeScene.Location(:,1);
    hScatter2.YData = planeScene.Location(:,2);
    hScatter2.ZData = planeScene.Location(:,3);
    hScatter2.CData = planeScene.Color;
    drawnow('update')
end
save('box_side2.mat','ptCloudScene');
