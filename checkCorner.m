function corners = checkCorner(N,xx,yy,zz,plane1,plane2)

possibleCorners = [];
plane = plane1;
p = plane.Location;
pX = plane.XLimits;
pY = plane.YLimits;
pZ = plane.ZLimits;
idx = find((p(:,1) == pX(1)) | (p(:,1) == pX(2)));
possibleCorners = [possibleCorners;p(idx,:)];
idx = find((p(:,2) == pY(1)) | (p(:,2) == pY(2)));
possibleCorners = [possibleCorners;p(idx,:)];
idx = find((p(:,3) == pZ(1)) | (p(:,3) == pZ(2)));
possibleCorners = [possibleCorners;p(idx,:)];

plane = plane2;
p = plane.Location;
pX = plane.XLimits;
pY = plane.YLimits;
pZ = plane.ZLimits;
idx = find((p(:,1) == pX(1)) | (p(:,1) == pX(2)));
possibleCorners = [possibleCorners;p(idx,:)];
idx = find((p(:,2) == pY(1)) | (p(:,2) == pY(2)));
possibleCorners = [possibleCorners;p(idx,:)];
idx = find((p(:,3) == pZ(1)) | (p(:,3) == pZ(2)));
possibleCorners = [possibleCorners;p(idx,:)];

error = [];
for i = 1:length(possibleCorners)
    pnt = possibleCorners(i,:);
    error_xy = abs((pnt(1) - xx) / N(1) - (pnt(2) - yy) / N(2));
    error_xz = abs((pnt(1) - xx) / N(1) - (pnt(3) - zz) / N(3));
    error = [error; max(error_xy,error_xz)];
end

idx_min = find(error == min(error));
idx_min = idx_min(1);
error(idx_min) = max(error);
corner1 = possibleCorners(idx_min,:);
for i = 1:length(error)
    idx = find(error == min(error));
    idx = idx(1);
    corner2 = possibleCorners(idx,:);
    if sqrt(sum((corner2 - corner1) .^2)) > 0.05
        break
    else
        error(idx) = max(error);
    end
end
corners = [corner1;corner2];

