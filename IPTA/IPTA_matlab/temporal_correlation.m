function total_temporal_correlation = temporal_correlation(data_wfs1, data_wfs2, cross_center, total_frames, fpi, delta_init, mask, echo)
    % It calculates temporal coross correlation between pairs of wfs 
    
    % INPUTS

    % data_wfs1, data_wfs2: Data form pair of WFS. They must be matblal
    %               double arrays! Dimensions must be
    %               wfs_data (including masked zeros) x number_of_samples

    % corrss_center: Do we want peaks to cross the baseline or to start on
    %                it? (true or false)
    
    % total_frames: How many frames do we want on the total sequence
    
    % fpi:          (frames per iteration) How many samples do we skip when
    %               corss-correlating. ("fps")
    
    % delta_init:   How many samples do we skip before we start correlating
    %               samples.
    
    % mask:         Mask of valid 

    % echo:         0 - Print nothing
    %               1 - Print progress % on terminal
    %               2 - Show GUI progress bar with cross-correlation
    %               progress

    % OUTPUTS
    % total_temporal_correlation: video sequence with "total_frames" number
    %               of images (on matlab array format). Dimensions are wfs
    %               (X_resolution*2 - 1) x (Y_resolution*2 - 1) x total_frames
    
    if(echo == 2)
        progres_bar = waitbar(0,'Calculating temporal cross-correlation');
    end
    
    %force correct format
    total_frames = double(total_frames);
    fpi          = double(fpi);
    delta_init   = double(delta_init);
    mask         = double(mask);
    

    %constants 
    number_of_samples = size(data_wfs1, 2);
    dimensions = size(data_wfs1, 1)/2;          %how many X and Y solpes
    resolution = sqrt(dimensions);              %WFS resolucion resolution (they are square)
    
    %setup output resolution
    total_temporal_correlation = zeros(2*resolution-1, 2*resolution-1, total_frames);
    
    %get reference frame index
    if(cross_center)
        %sequence center frame
        fixed_frame_index = fix(total_frames/2) + 1; 
    else
        %sequence first frame
        fixed_frame_index = 1;
    end
    
    progress_val_ant = 0;

    total_temporal_correlations = (number_of_samples - (total_frames - 1)*fpi);
    for start_sample_index = 1:delta_init:total_temporal_correlations
        
        %display progress
        progress = start_sample_index/total_temporal_correlations;
        if(echo == 2)
            waitbar(progress, progres_bar,'Calculating temporal cross-correlation');
        elseif(echo == 1)
            progress_val = fix(progress*100);
            if(progress_val ~= progress_val_ant)
                progress_val_ant = progress_val;
                fprintf('\t%d%%', progress_val)
                if(mod(progress_val, 10) == 0)
                    fprintf('\n')
                end
            end
        end
        
        %calculate crosscorelation
        fixed_frame = start_sample_index + (fixed_frame_index - 1)*fpi;
        
        X_1 = data_wfs1(1:dimensions, fixed_frame);
        Y_1 = data_wfs1((dimensions + 1):end, fixed_frame);
    
        X_1 = reshape(X_1,[resolution resolution]);
        Y_1 = reshape(Y_1,[resolution resolution]);
        

        for frame = 1:total_frames
            sample = start_sample_index + (frame - 1)*fpi;

            X_2 = data_wfs2(1:dimensions, sample);
            Y_2 = data_wfs2((dimensions + 1):end, sample);

            X_2 = reshape(X_2,[resolution resolution]);
            Y_2 = reshape(Y_2,[resolution resolution]);
    
            total_temporal_correlation(:, :, frame) = total_temporal_correlation(:, :, frame) + xcorr2(X_1, X_2) + xcorr2(Y_1, Y_2);
        end
       
    end

    kt = total_frames*(number_of_samples - (total_frames - 1)*fpi)/delta_init;
    % -----------------------------------------------------------------------
    % bowl border filter (Guesalaga)
    
    Mxy = xcorr2(mask);
    Mxy = Mxy + 100000*(Mxy < 9);
    Mxy2 = Mxy + 100000*(Mxy < 27);
    Wxy2 = (Mxy2 < 10000); 
    
    for j = 1:total_frames
       total_temporal_correlation(:, :, j)= Wxy2.*total_temporal_correlation(:, :, j)./(kt*Mxy); 
    end
    
    if(echo == 2)
        close(progres_bar)
    end

end