clc, close all, clear all

%%
% Instantiate XL_320 class
xl = XL_320();
%Set the port parameters
s = serial('COM5');
set(s,'Baudrate',1000000);   %set speed of communication at 1'000'000 bps
set(s,'StopBits',1);         %specify number of bits used to indicate end of byte
set(s,'DataBits',8);         %number of data bits to transmit (we use 8 bit data)
set(s,'Parity','none');      %no parity
%Connect serial port
fopen(s);
%%
% Create the face detector object.
faceDetector_L = vision.CascadeObjectDetector();
faceDetector_R = vision.CascadeObjectDetector();
%%
% Create the point tracker object.
pointTracker_L = vision.PointTracker('MaxBidirectionalError', 2);
pointTracker_R = vision.PointTracker('MaxBidirectionalError', 2);
%%
% Create the webcam object.
LeftEye = webcam(2);
RightEye = webcam(3);
%%
% Capture one frame to get its size.
videoFrame_L = imresize(snapshot(LeftEye),[480 480]);
videoFrame_R = imresize(snapshot(RightEye),[480 480]);
frameSize_L = size(videoFrame_L);
frameSize_R = size(videoFrame_R);
%%
% Create the video player object. 
videoPlayer_L = vision.VideoPlayer('Position', [700 100 [frameSize_L(2), frameSize_L(1)]+30]);
videoPlayer_R = vision.VideoPlayer('Position', [150 100 [frameSize_R(2), frameSize_R(1)]+30]);
%%
% Detection and Tracking
% Capture and process video frames from the webcam in a loop to detect and
% track a face. The loop will run until the video player
% window is closed.
elapsedTime = tic;
runLoop = true;
numPts_L = 0;
numPts_R = 0;
i = 0;
flag = 0;
while runLoop
    i = i + 1;
    % Get the next frame.
    videoFrame_L = imresize(snapshot(LeftEye),[480 480]);
    videoFrame_R = imresize(snapshot(RightEye),[480 480]);
    videoFrameGray_L = rgb2gray(videoFrame_L);
    videoFrameGray_R = rgb2gray(videoFrame_R);
%%
    if numPts_L < 10 || numPts_R < 10 
        % Detection mode.
        bbox_L = faceDetector_L.step(videoFrameGray_L);
        bbox_R = faceDetector_R.step(videoFrameGray_R);
%%        
        if ~isempty(bbox_L) && ~isempty(bbox_R)
            % Find corner points inside the detected region.
            points_L = detectMinEigenFeatures(videoFrameGray_L, 'ROI', bbox_L(1, :));
            points_R = detectMinEigenFeatures(videoFrameGray_R, 'ROI', bbox_R(1, :));
%%            
            % Re-initialize the point tracker.
            xyPoints_L = points_L.Location;
            xyPoints_R = points_R.Location;
            numPts_L = size(xyPoints_L,1);
            numPts_R = size(xyPoints_R,1);
            release(pointTracker_L);
            release(pointTracker_R);
            initialize(pointTracker_L, xyPoints_L, videoFrameGray_L);
            initialize(pointTracker_R, xyPoints_R, videoFrameGray_R);
%%            
            % Save a copy of the points.
            oldPoints_L = xyPoints_L;
            oldPoints_R = xyPoints_R;
%%            
            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints_L = bbox2points(bbox_L(1, :));  
            bboxPoints_R = bbox2points(bbox_R(1, :));  
%%            
            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4] 
            % format required by insertShape.
            bboxPolygon_L = reshape(bboxPoints_L', 1, []);
            bboxPolygon_R = reshape(bboxPoints_R', 1, []);
%%            
            % Display a bounding box around the detected face.
            videoFrame_L = insertShape(videoFrame_L, 'Polygon', bboxPolygon_L, 'LineWidth', 3);
            videoFrame_R = insertShape(videoFrame_R, 'Polygon', bboxPolygon_R, 'LineWidth', 3);
%%            
            %% Finding central points of the detected faces            
        centroids_L = mean(xyPoints_L);
        centroids_R = mean(xyPoints_R);
%%            
            % Display detected corners.
            videoFrame_L = insertMarker(videoFrame_L, centroids_L, '+', 'Color', 'white');
            videoFrame_R = insertMarker(videoFrame_R, centroids_R, '+', 'Color', 'white');            
        end
    else
        % Tracking mode.
        [xyPoints_L, isFound_L] = step(pointTracker_L, videoFrameGray_L);
        [xyPoints_R, isFound_R] = step(pointTracker_R, videoFrameGray_R);
        visiblePoints_L = xyPoints_L(isFound_L, :);
        visiblePoints_R = xyPoints_R(isFound_R, :);
        oldInliers_L = oldPoints_L(isFound_L, :);
        oldInliers_R = oldPoints_R(isFound_R, :);
%%                
        numPts_L = size(visiblePoints_L, 1);       
        numPts_R = size(visiblePoints_R, 1);       
%%        
        if numPts_L >= 10 && numPts_R >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform_L, oldInliers_L, visiblePoints_L] = estimateGeometricTransform(...
                oldInliers_L, visiblePoints_L, 'similarity', 'MaxDistance', 4);            
            [xform_R, oldInliers_R, visiblePoints_R] = estimateGeometricTransform(...
                oldInliers_R, visiblePoints_R, 'similarity', 'MaxDistance', 4);            
%%            
            % Apply the transformation to the bounding box.
            bboxPoints_L = transformPointsForward(xform_L, bboxPoints_L);
            bboxPoints_R = transformPointsForward(xform_R, bboxPoints_R);
%%            
            %% Finding central points of the detected faces            
            centroids_L = mean(bboxPoints_L);
            centroids_R = mean(bboxPoints_R);
%% 
flag = 1;
            
            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4] 
            % format required by insertShape.
            bboxPolygon_L = reshape(bboxPoints_L', 1, []);            
            bboxPolygon_R = reshape(bboxPoints_R', 1, []);            
%%            
            % Display a bounding box around the face being tracked.
            videoFrame_L = insertShape(videoFrame_L, 'Polygon', bboxPolygon_L, 'LineWidth', 3);
            videoFrame_R = insertShape(videoFrame_R, 'Polygon', bboxPolygon_R, 'LineWidth', 3);
%%            
            % Display tracked points.
            videoFrame_L = insertMarker(videoFrame_L, centroids_L, '+', 'Color', 'white');
            videoFrame_R = insertMarker(videoFrame_R, centroids_R, '+', 'Color', 'white');
%%            
            % Reset the points.
            oldPoints_L = visiblePoints_L;
            oldPoints_R = visiblePoints_R;
            setPoints(pointTracker_L, oldPoints_L);
            setPoints(pointTracker_R, oldPoints_R);
        end
    end
%%    
    % Display the annotated video frame using the video player object.
    step(videoPlayer_L, videoFrame_L);
    step(videoPlayer_R, videoFrame_R);
    
    %%
%%    
    if(flag == 1)
        
%%Saving the Centural Points
            center(i,1) = centroids_L(1,2);%v1 left y
            center(i,2) = centroids_L(1,1);%u1 left x
            center(i,3) = centroids_R(1,2);%v2 right y
            center(i,4) = centroids_R(1,1);%u2 right x
%% PID Coefficients 
num = [ 240 ];
denum = [ 54.1 108.2 ];
G = tf( num,denum );
H = [ 1 ];
%KP = 0.008276;
KP = 806.32136;
KI = 7.462e-05;
KD = 0.1993;
Gc = pid(KP, KI, KD);
Mc = feedback(Gc*G, H);
%%
% Stop the Timer
t=toc(elapsedTime);
x = 0:t/(i-1):t;
C2 = lsim(Mc,center(:,1),x);
C1 = lsim(Mc,center(:,2),x);
C4 = lsim(Mc,center(:,3),x);
C3 = lsim(Mc,center(:,4),x);
%%
%Control Expression
GP(i,2) = round(512 - C2(i,1));
                   if GP(i,2)>1023
                      GP(i,2) = 1023;
               elseif GP(i,2)<0
                      GP(i,2) = 0;
                   end
%%                   
GP(i,1) = round(512 - C1(i,1));
                   if GP(i,1)>1023
                      GP(i,1) = 1023;
               elseif GP(i,1)<0
                      GP(i,1) = 0;
                   end
%%                   
GP(i,4) = round(512 - C4(i,1));
                   if GP(i,4)>1023
                      GP(i,4) = 1023;
               elseif GP(i,4)<0
                      GP(i,4) = 0;
                   end
%%                   
GP(i,3) = round(512 - C3(i,1));
                   if GP(i,3)>1023
                      GP(i,3) = 1023;
               elseif GP(i,3)<0
                      GP(i,3) = 0;
                   end
%% Moving eyes
xl.sync_write_four([1,3,2,4],xl.Address.GOAL_POSITION,[GP(i,1),GP(i,3),GP(i,2),GP(i,4)],xl.Bytes.GOAL_POSITION,s);
    end
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer_L) && isOpen(videoPlayer_R);
end
%% 
%Returning to Home Location
xl.sync_write_four([1,3,2,4],xl.Address.GOAL_POSITION,[512,512,512,512],xl.Bytes.GOAL_POSITION,s);
%%
%Clean up.
clear LeftEye;
clear RightEye;
release(videoPlayer_L);
release(videoPlayer_R);
release(pointTracker_L);
release(pointTracker_R);
release(faceDetector_L);
release(faceDetector_R);
%%
%Disconnect serial port
fclose(s);
delete(s)
clear s