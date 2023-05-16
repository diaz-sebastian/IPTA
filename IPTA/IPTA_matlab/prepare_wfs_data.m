function processed_wfs_data = prepare_wfs_data(file_name, mask)

% Reshapes data to fit mask, it also subtracts the mean on each frame

% INPUTS
% file_name: .fits open loop WFS data file name
% mask:      2D logical array with active wfs slopes

% OUTPUTS
%processed_wfs_data: WFS data ready to be used

total = sum(mask, 'all');
% Load fits file
slopes_data = fitsread(file_name);

% Preproccess slopes
processed_wfs_data = zeros(3200, 2, size(slopes_data,1));
for frame = 1:size(slopes_data,1)
    
    processed_slopes = zeros(3200,2);
    for wfs = 0:1
        inicio = wfs*total*2 + 1;
        medio = inicio + total;
        fin = inicio + total*2;
        
        %load data
        wfs_x_data = slopes_data(frame, inicio:(medio - 1));
        wfs_y_data = slopes_data(frame, medio:(fin - 1));

        %subtracts mean
        wfs_x_data = wfs_x_data - sum(wfs_x_data, 'All')/total;
        wfs_y_data = wfs_y_data - sum(wfs_y_data, 'All')/total;
        
        %reshape (to remove zeros)
        wfs_x = zeros(size(mask)); wfs_y = wfs_x;
        wfs_x(mask) = wfs_x_data;
        wfs_y(mask) = wfs_y_data;

        %reshape to return data
        processed_slopes(:, wfs + 1) = [wfs_x(:) ; wfs_y(:)];

    end

    processed_wfs_data(:, :, frame) = processed_slopes;
end

end