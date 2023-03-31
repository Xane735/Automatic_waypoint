#Python Code to convert multiple .waypoints files in one directory to .cvs files in another directory
#install pandas library before running (pip install pandas)
#enter the directory where .waypoints files are stored and also enter where the .cvs files should be stored


import os
import pandas as pd

# set the directory where the .waypoints files are located
waypoints_dir = ''

# set the directory where the output .csv files will be saved
output_dir = ''

# loop through all .waypoints files in the directory
for filename in os.listdir(waypoints_dir):
    if filename.endswith('.waypoints'):
        # read the file into a pandas dataframe
        df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, names=['waypoint_number', 'ishome','confirmation', 'dont_know_1','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','dont_know_2'])

        # select specific columns from the dataframe
        selected_columns = df[['waypoint_number', 'ishome','confirmation', 'dont_know_1','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','dont_know_2']]

        # construct the output filename
        output_filename = os.path.splitext(filename)[0] + '.csv'

        # write the selected columns to a new .csv file
        selected_columns.to_csv(os.path.join(output_dir, output_filename), index=False)
