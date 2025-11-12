#!/usr/bin/env python3
import os
import csv
import numpy as np
import math
from PIL import Image
import matplotlib.pyplot as plt

# This script is used to plot and save exp results
def main():
    HOME = os.path.expanduser('~')
    
    # add folder path and save paths, any other variables needed
    folder_path = HOME+"/.ros/"

    # Read error
    err_data = []
    row_size = 0
    file_name = folder_path + "error.csv"
    with open(file_name, 'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        for row in plots:
            err_data.append(row)

    # Process list data
    err_data = err_data[2:]
    err_data = [list( map(float,i) ) for i in err_data]
    err_data = np.array(err_data)
    # print(err_data[0,1:2])
    print(err_data.shape)
    # compute error norm
    err_norm = []
    for row in range(len(err_data)):
        sum = 0
        for err in err_data[row]:
            sum += float(err)**2
        err_norm.append(math.sqrt(sum))
    
    # create plot axes
    fig, axes = plt.subplots(nrows=2, ncols=1)
    ax1 = axes[0]
    ax2 = axes[1]
    fig.tight_layout(pad=4.0)

    # Plot error norm   
    ax1.plot(err_norm, 'b',linewidth=1.5)
    ax1.margins(x = 0.0, y=0.0)
    ax1.set_ylabel('Error norm')
    ax1.set_xlabel('Control loop iteration #')
    ax1.grid()
    ax1.legend(['Error norm'], loc='upper center', bbox_to_anchor=(0.5, -.35),
          fancybox=True, shadow=True, ncol=1)

    # Plot feature error  
    for col in range(len(err_data[0])):
        ax2.plot(err_data[1:, col], linewidth=1.5)   
    ax2.margins(x = 0.0, y=0.0)
    ax2.set_ylabel('Feature errors (px)')
    ax2.set_xlabel('Control loop iteration #')
    ax2.grid()
    ax2.legend(['x','y'],loc='upper center',
    bbox_to_anchor=(0.5, -0.35), fancybox=True, shadow=True, ncol=len(err_data[0]))
    
    # Save plot
    save_file = folder_path + 'feature_error.png'
    plt.savefig(save_file, dpi=300)
    Image.open(save_file).convert("RGB").save('feature_error.jpg','JPEG')

    # Read model error
    model_error = []
    file_name = folder_path + "model_error.csv"
    with open(file_name,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        for row in plots:
            model_error.append(row)

    # Process list data

    model_error = [list( map(float,i) ) for i in model_error]
    model_error = np.array(model_error)

    # Read velocities
    velocity = []
    row_size = 0
    file_name = folder_path + "velocities.csv"
    with open(file_name,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        for row in plots:
            velocity.append(row)
            row_size = len(row)

    # Process list data
    velocity = velocity[2:row_size]
    velocity = [list( map(float,i) ) for i in velocity]
    velocity = np.array(velocity)
    
    # Create axes for plots
    fig, axes = plt.subplots(nrows=2, ncols=1)
    ax1 = axes[0]
    ax2 = axes[1]
    fig.tight_layout(pad=4.0)

    # Plot model error
    ax1.plot(model_error, 'b',linewidth=1.5)
    ax1.margins(x = 0.0, y=0.0)
    ax1.set_ylabel('Model error')
    ax1.set_xlabel('Iteration #')
    ax1.grid()
    ax1.legend(['Model error'], loc='upper center', bbox_to_anchor=(0.5, -.75),
          fancybox=True, shadow=True, ncol=1)

    # Plot velocities
    
    ax2.margins(x = 0.0, y=0.0)
    ax2.set_ylabel('Actuator velocities (rad/s)')
    ax2.set_xlabel('Iteration #')
    ax2.grid()

    for col in range(row_size):
        ax2.plot(velocity[col], 'b',linewidth=1.5)
        
    
    # ax2.legend(['Actuator velocities'], loc='upper center', bbox_to_anchor=(0.5, -.75),
        #   fancybox=True, shadow=True, ncol=1)

    # Save plot
    save_file = folder_path + 'plot.png'
    plt.savefig(save_file, dpi=300)
    Image.open(save_file).convert("RGB").save('plot.jpg','JPEG')


if __name__ == "__main__":
    main()