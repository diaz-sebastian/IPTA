function filtered_video = noise_filter(original_video)

% Filters noise video video 
% INPUTS:
% original_video:   Matlab double array containing video data. For a axb
%                   resolution video the matlab array must have axbxf
%                   dimensions where f is the number of frames.

% OUTPUTS:
% filter_video:     Filtered video on matlab double array with dimensions axbxf

% RF3D available at https://webpages.tuni.fi/foi/GCF-BM3D/
filtered_video = RF3D(original_video, -1, -1, ones(8,'single'), ones(8,'single'), 'dct', true, false, 'lc', false);

% [1] M. Maggioni, E. Sanchez-Monge, A. Foi, "Joint removal of random and
%     fixed-pattern noise through spatiotemporal video filtering", IEEE 
%     Transactions on Image Processing, vol.23, no.10, pp. 4282-4296, Oct. 2014
%     http://doi.org/10.1109/TIP.2014.2345261
% 
% [2] M. Maggioni, G. Boracchi, A. Foi, and K. Egiazarian, "Video denoising,
%     deblocking, and enhancement through separable 4-D nonlocal spatiotem-
%     poral transforms," IEEE Transactions on Image Processing, vol. 21,
%     no. 9, pp. 3952-3966, Sep. 2012.  http://doi.org/10.1109/TIP.2012.2199324

end