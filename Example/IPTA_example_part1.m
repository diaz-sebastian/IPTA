clc
clear all
close all

%% Necessary files and directories

% Add path with IPTA matlab functions
addpath('../IPTA/IPTA_matlab/')

% WFS data mask file (2D logical array with active wfs slopes)
load('files/mask_example.mat');

% Open loop WFS data to be proccesed
fits_dir = 'files/fits_example_file.fits';

% Config file
config_dir = 'config.json';

% Output file name
out_file_name = 'files/output_correlation_example_file.mat';

%% Load configuration parameters
fid = fopen(config_dir, 'r'); % Opening the file
config = jsondecode(char(fread(fid,inf))');
fid = fclose(fid);

cross_center = config.correlation.cross_center;
total_frames = config.correlation.total_frames;
fpi = config.correlation.fpi;
delta_init = config.correlation.fpf;
sensor_1 = config.correlation.sensor_1_id;
sensor_2 = config.correlation.sensor_2_id;
number_of_samples = config.correlation.number_of_samples;

%% Calculate temporal cross correlation
processed_wfs_data = prepare_wfs_data(fits_dir, mask);

% Separate wfs data
data_wfs1 = processed_wfs_data(:, sensor_1, 1:number_of_samples);
data_wfs2 = processed_wfs_data(:, sensor_2, 1:number_of_samples);

% Select desired number of samples
data_array_size = size(data_wfs1, 1);
data_wfs1 = reshape(data_wfs1, [data_array_size number_of_samples]);
data_wfs2 = reshape(data_wfs2, [data_array_size number_of_samples]);

% Calculate temporal cross correlation
temporal_correlation_result = temporal_correlation(data_wfs1, data_wfs2, cross_center, total_frames, fpi, delta_init, mask, 2);

% Filter results
filtered_video = atmosferic_deconvolution(noise_filter(temporal_correlation_result));

% Save video 
save(out_file_name,"filtered_video")

%% Show mask
figure(1)

imagesc(mask); axis square
title('mask');
drawnow

%% Show results
figure(2)

for iteration = 1:3
    for frame = 1:size(filtered_video, 3)
        imagesc(filtered_video(:, :, frame)); axis square
        hold on
        plot([0 40], [40 40], 'r')
        title(['Temporal cross-correlation result. Frame ' num2str(frame)])
        drawnow
        pause(0.2)
    
    end
end



