When executing example the working directory must be this folder.

## Matlab

IMPORTANT: Remember to download RF3D form https://webpages.tuni.fi/foi/GCF-BM3D/ by clicking on "_Download zipped RF3D MATLAB software_".
First unzip the downloaded file, then select the folowing files:
- RE3D.m
- RF3D_8_mex.mexa64
- RF3D_8_mex.mexw64
- RF3D_8_mex.mexmaci64

And copy them to the folder .../IPTA/IPTA_matlab 

Signal Processing Toolbox https://www.mathworks.com/products/signal.html?s_tid=AO_PR_info and Image Processing Toolbox are required https://www.mathworks.com/products/image.html?s_tid=AO_PR_info

## Python
This libraries are requiered:

- Numpy
- Scikit-image
- Some other libraries may be required, depends on your system

# How to execute IPTA example

1. Execute IPTA_example_part1.m on matlab, these will calculate and create the temporal cross correlation form the example.fits file. It will be saved on correlation.mat.
2. Execute IPTA_example_part2.py with python. These will estimate the wind profile form the coorelation.mat file. The estimated profiles will be saved on estimations.json.

DISCLAIMER: this version of IPTA asumes that the baseline is located horizontally and on the left side (see plot example on IPTA_example_part1.m).
