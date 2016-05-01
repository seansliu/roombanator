function [D, I, j] = get_camera_image(CameraHandle, j)
    % Wrapping most of these functions in evalc to silence debug msg
    global show_image
    j = j+1;
    evalc('pxcAcquireFrame(CameraHandle)');
    [T,D] = evalc('pxcDepthImage(CameraHandle)'); 
    D=permute(D,[2 1]);
    [T,I] = evalc('pxcColorImage(CameraHandle)'); 
    I=permute(I([3,2,1],:,:),[3 2 1]);
    if show_image
        subplot(2,2,1),h1=imshow(I); colormap('jet');
        title(j);
        subplot(2,2,3),h2=imshow(D,[200 750]);  colormap('jet'); evalc('freezeColors');
    end
    pxcReleaseFrame(CameraHandle);
end