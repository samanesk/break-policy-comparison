By Saman Eskandarzadeh.
website: http://www.samaneskandarzadeh.com

# Introduction

This repo includes the implementation of the models in the following paper:
Eskandarzadeh, S., & Fahimnia, B. (2022). Rest break policy comparison for heavy vehicle drivers in Australia. Transportation research part E: logistics and transportation review, 159.

# Installation and setup
The codes are written in Python 3. To run the code just need to pull the repo into a local machine. The code to run requires the following python libraries:
argparse, time, datetime, traceback, gurobipy, pandas. 

The code can also be run from the command line. The command-line parameters are in the main.py.

# Code Structure
There are four py files: main.py, model_params.py, data_prep_functions.py, and models.py

To run the code, file main.py should be run. The code reads data files in the folder Data and then writes the optimal duties(tours in the paper) 
in the folder Output. 

There are four sample data files:
duties: this file should contain the composition of current tours run by the parcel company
jobs: this file should contain all the jobs 
location_geocoded: this file contains the location of all jobs
duration_peak_matrix_full. this file contain the travel time between every pair of locations in the location file

   




