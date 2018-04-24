%% Load the training data
clear all
close all
box = load('assignment_1_box.mat');
box = box.pcl_train;
flag=1;
checkframe=1;
%% Uncomment to load the test file
%box = load('assignment_1_test.mat');
%box = box.pcl_test;
totalcount=0;
% count the number of recognised planes in one frame
successFrames = {};
allFrames = {};
numSuccessFrames = 0;
% display the points as a point cloud and as an image
for frameNum = 1:length(box) % Reading the 50 point-clouds
    frameNum
	% extract a frame
	rgb = box{frameNum}.Color; % Extracting the colour data
	point = box{frameNum}.Location; % Extracting the xyz data
    
    %pc = pointCloud(point, 'Color', rgb);
    %figure(1);
    %pcshow(pc);
    %title('Original Point Cloud');

	%Extract points close to box%%%%%%%%%%%%%%%%%%%%%

	BoxPos = [-0.71,-0.30,0.81];
    for i=1:length(point)
        point(i,:)=((point(i,:)-BoxPos));
        if(abs(point(i,1))>0.25|| abs(point(i,2))>0.25|| abs(point(i,3))>0.25)
            point(i,:)=NaN;
            rgb(i,:)=NaN;
        end
    end
	pc = pointCloud(point, 'Color', rgb); % Creating a point-cloud variable


	%Remove hand%%%%%%%%%%%%%%%%%%%%% hand is 100=-20, 80+-20, 40 +-10
%     figure(1)
	r = rgb(:,1);
	g = rgb(:,2);
	b = rgb(:,3);
	for i=1:length(r)
		if((r(i)<150 && r(i)>40)&&(g(i)<130 && g(i)>35)&&(b(i)<100 && b(i)> 0))%rgb color range of values for hand
			% r(i)=NaN;
			% g(i)=NaN;
			% b(i)=NaN;
			point(i,:)=NaN;
			rgb(i) = NaN;
		end
	end
	%Remove points that don't belong to box
	idx = find(isnan(point(:,1)));
	point(idx,:) = [];
	rgb(idx,:) = [];

	%Remove excess noise%%%%%%%%%%%%%%%%%%%%%%%%
	BW = im2bw(rgb,0.1);
	BW2 = bwareaopen(BW, 10);
	new_rgb_box= bsxfun( @times , rgb , cast( BW2, 'like' , rgb) ) ;
	new_point_box= bsxfun( @times , point , cast( BW2, 'like' , point) ) ;
    %new_rgb_box=rgb; new_point_box=point;

	%Remove points that don't belong to box
	idx = find(sum(new_rgb_box,2)==0);
	new_point_box(idx,:) = [];
	new_rgb_box(idx,:) = [];

%     imshow(BW2);
%     imshow(new_rgb_box);
	pc1 = pointCloud(new_point_box, 'Color', new_rgb_box);

%  	figure(2);
%     clf
%  	pcshow(pc1);
%  	title(frameNum);
%     hold on
    
%     figure(3);
% 	pcshow(pc1);
% 	title('Plane Data');
%     hold on
    allFrames{frameNum}.box = pc1;
    %check if there is enough points left

	if(length(new_point_box) < 3000)
		continue
	end
%%




    checkframe=1; 

    % fit three planes, and only store when it's large enough
	maxDistance = 0.01;
    count = 0;

	[model1,inlierIndices,outlierIndices] = pcfitplane(pc1,...
			maxDistance);
	plane1 = select(pc1,inlierIndices);
	remainPtCloud = select(pc1,outlierIndices);
    if plane1.Count > 1000
        count = count + 1; totalcount=totalcount+1;
        boxPlanes{count} = plane1;
        boxNormals{count} = model1.Parameters;

      origin1 = [mean(plane1.XLimits),mean(plane1.YLimits),mean(plane1.ZLimits)];
      direction1 = model1.Parameters(1:3);
    end
    
    thresh=0.01;
    mergeSize=0.001;
    
    
    
	[model2,inlierIndices,outlierIndices] = pcfitplane(remainPtCloud,...
				maxDistance);
	plane2 = select(remainPtCloud,inlierIndices);
	remainPtCloud = select(remainPtCloud,outlierIndices);
    
%     if(totalcount==1 && plane2.Count>1000)
%          planeA=plane1;
%          %figure;
%          %pcshow(planeA); 
%          flag=0;
%     end
%     if(flag==1)
%         totalcount=0;
%     end
%     
%     if(flag==0)%%Check the transformation
%         [tf,moved,rmse]=pcregrigid(plane1,planeA,'Metric','pointtoPlane','Extrapolate',true);
%         if(rmse<thresh)%%if same plane
%         checkframe=0;    
%         end
%     end
            
         
    if plane2.Count > 1000
        count = count + 1; totalcount=totalcount+1;
        boxPlanes{count} = plane2;
        boxNormals{count} = model2.Parameters;
        origin2 = [mean(plane2.XLimits),mean(plane2.YLimits),mean(plane2.ZLimits)];
        direction2 = model2.Parameters(1:3);
    end
        
% 	[model3,inlierIndices,outlierIndices] = pcfitplane(remainPtCloud,...
% 				maxDistance);
% 	plane3 = select(remainPtCloud,inlierIndices);
% 	remainPtCloud = select(remainPtCloud,outlierIndices);
%     if plane3.Count > 1000
%         count = count + 1; totalcount=totalcount+1;
%         boxPlanes{count} = plane3;
%         boxNormals{count} = model3.Parameters;
% %         origin = [mean(plane3.XLimits),mean(plane3.YLimits),mean(plane3.ZLimits)];
% %         direction = model3.Parameters(1:3);
% %         if dot(origin - BoxPos, direction) < 0
% %             direction = -direction;
% %         end
% %         quiver3(origin(1),origin(2),origin(3),...
% %             direction(1),direction(2),direction(3),...
% %             0.1);
%     end
          
   
    if (count <= 1)
        continue;
    end
    
        % check fit result
%  	figure(frameNum);   
%     subplot(1,2,1)
%  	pcshow(plane1)
%     title([num2str(frameNum),'-plane1'])
%  	 subplot(1,2,2)
%  	pcshow(plane2)
%     title([num2str(frameNum),'-plane2'])
%     drawnow
%  	figure
%  	pcshow(plane3)
%  	title('Third Plane')
%  	figure
%  	pcshow(remainPtCloud)
%  	title('Remaining Point Cloud')
    

   
   if (dot(origin1 - origin2, direction1) < 0)
        direction1 = -direction1;
    end
    if (dot(origin2 - origin1, direction2) < 0)
        direction2 = -direction2;
    end
% % 
% %      quiver3(origin1(1),origin1(2),origin1(3),...
% %         direction1(1),direction1(2),direction1(3),...
% %         0.1);
% %     quiver3(origin2(1),origin2(2),origin2(3),...
% %         direction2(1),direction2(2),direction2(3),...
% %         0.1);
    N1 = model1.Parameters;
    N2 = model2.Parameters;
    N = cross(N1(1:3),N2(1:3));
    N = N/norm(N);
    xx = 0;
    yy = (N2(4)*N1(3) - N1(4)*N2(3))/ N(1);
    zz = (N1(4)*N2(2) - N2(4)*N1(2))/ N(1);
    corners = checkCorner(N,xx,yy,zz,plane1,plane2);
% %     scatter3(corners(:,1),corners(:,2),corners(:,3),'filled','r');
   
    oneFrame.box = pc1;
    oneFrame.numPlanes = count;
    oneFrame.planes = boxPlanes;
    oneFrame.planeParameters = boxNormals; 
    oneFrame.checkframe = checkframe;
    oneFrame.frameNum=frameNum;
    
    numSuccessFrames = numSuccessFrames + 1;
    successFrames{numSuccessFrames} = oneFrame;
    
    
    %planetmp=pcmerge(plane1, plane2,0.001);
    Planespc{numSuccessFrames} = oneFrame;
    %Planespc{numSuccessFrames}.box=pcmerge(planetmp, plane3,0.001);
    Planespc{numSuccessFrames}.box=pcmerge(plane1, plane2,0.001);
    Planespc{numSuccessFrames}.corners = corners;
    Planespc{numSuccessFrames}.plane1=plane1;
    Planespc{numSuccessFrames}.plane2=plane2;
    Planespc{numSuccessFrames}.frameNum = frameNum;
    Planespc{numSuccessFrames}.norms{1} = direction1;
    Planespc{numSuccessFrames}.norms{2} = direction2;
 

%     pause();
end
% save('successFrames.mat','successFrames');
% save('allFrames.mat','allFrames');
save('Planespc.mat','Planespc');
add_manual
