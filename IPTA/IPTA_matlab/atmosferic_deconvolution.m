function filtered_video = atmosferic_deconvolution(video)
% Performs richardson-lucy deconvolution in as a noise filter

% INPUTS
% video:            px_x by px_y by number_of_frames video data

% OUTPUTS
% filtered_video:   deconvoluted video


filtered_video = zeros(size(video));

number_of_frames = size(video, 3);

%atmosferic turbulence
k = 1.9497;
[x,y] = meshgrid(-4:3,-4:3);
PSF = exp(-k*(x.^2 + y.^2).^(5/6));

for frame_index = 1:number_of_frames
    frame = video(:,:,frame_index);
    filtered_video(:,:,frame_index) = deconvlucy(frame, PSF, 10);   
end
