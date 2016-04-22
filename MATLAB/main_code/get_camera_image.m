function [D, I] = get_camera_image(CameraHandle)
    % Wrapping most of these functions in evalc to silence debug msg
    evalc('pxcAcquireFrame(CameraHandle)');
    [T,D] = evalc('pxcDepthImage(CameraHandle)'); 
    D=permute(D,[2 1]);
    [T,I] = evalc('pxcColorImage(CameraHandle)'); 
    I=permute(I([3,2,1],:,:),[3 2 1]);
    subplot(1,2,1),h1=imshow(I); 
    subplot(1,2,2),h2=imshow(D,[200 750]); colormap('jet');
    pxcReleaseFrame(CameraHandle);
end