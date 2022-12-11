# Phase1-RoverSimulator

## The Simulator
The first step is to download the simulator build that's appropriate for linux operating system. 
                       
You can test out the simulator by opening it up and choosing "Training Mode".  Use the mouse or keyboard to navigate around the environment and see how it looks.

## Dependencies
You'll need Python 3 and Jupyter Notebooks installed to do this project.  The best way to get setup with these if you are not already is to use Anaconda

## Recording Data
some test data for you in the folder called `test_dataset`.  In that folder you'll find a csv file with the output data for steering, throttle position etc. and the pathnames to the images recorded in each run.
There are also a few saved images in the folder called `calibration_images` to do some of the initial calibration steps with.       
                    
The first step of this project is to record data on your own.  To do this, you should first create a new folder to store the image data in.
Then launch the simulator and choose "Training Mode" then hit "r".  Navigate to the directory  
you want to store data in, select it, and then drive around collecting data.  Hit "r" again to stop data collection.  

## Data Analysis
Included in the IPython notebook called `Project.ipynb`  are the functions for performing the various steps of this project. To see what's in the notebook and execute the code there, start the jupyter notebook server at the command line like this: jupyter notebook 

This command will bring up a browser window in the current directory where you can navigate to wherever `Project.ipynb` is and select it.  Run the cells in the notebook from top to bottom to see the various data analysis steps.  

## Navigating Autonomously
The file called `drive_rover.py` is what you will use to navigate the environment in autonomous mode.  This script calls functions from within `perception.py` and `decision.py`.  The functions defined in the IPython notebook are all included in `perception.py` and we filled in the function called `perception_step()` with the appropriate processing steps and update the rover map. `decision.py` includes another function called `decision_step()`, which includes an  implementation of conditionals to make driving decisions based on the rover's state and the results of the `perception_step()` analysis.

the driver file should run on the terminal using the command: python drive_rover.py

Then launch the simulator and choose "Autonomous Mode".  The rover should drive itself.


*Note: running the simulator with different choices of resolution and graphics quality may produce different results! *
