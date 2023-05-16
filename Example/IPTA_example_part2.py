import numpy as np
import scipy
import json
import sys

# add IPTA path to use IPTA python functions
sys.path.append('../IPTA/IPTA_python/')

from IPTA import *

if __name__ == '__main__':

	#Load config file
	with open("config.json", "r") as infile:
			config = json.load(infile)

	#load cross-correlation results
	video = np.array(scipy.io.loadmat("files/output_correlation_example_file.mat")['filtered_video'])

	#estimate wind velocity
	results = IPTA(video, config, [])

	#show results
	for layer in results:
		print(f"h {layer[0]}[m]\t V(dx/dt, dy/dt): {(layer[1], layer[2])}[m/s]")
	
	#show as speed and angle (calculated for this example!)
	print('\nSpeed and angle for this example')
	for layer in results:
		speed = round(np.sqrt(layer[1]**2 + layer[2]**2),2)
		angle = round(90 - np.arctan(layer[1]/layer[2])*180/np.pi,2) #check this!
		print(f"h {layer[0]}[m]\tspeed: {speed}[m/s]\tangle: {angle}°")

	
	#Example is id19.fits form database