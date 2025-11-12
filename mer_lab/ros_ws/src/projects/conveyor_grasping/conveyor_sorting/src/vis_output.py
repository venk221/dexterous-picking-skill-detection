import glob

import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np

def main():
    data = open('/tmp/output.csv').read()
    metrics = data.split('\n')[1:-1]
    parsed = []
    for metric in metrics:
        sm = metric.split(',')
        parsed.append({
            'pre': float(sm[0]),
            'sweeping': np.array([
                [float(num) for num in sm[1:4]],
                [float(num) for num in sm[4:7]]
            ]),
            'post': float(sm[7])
        })
    score_fig, score_ax = plt.subplots(1, 1)
    print(metrics)
    dep_files = sorted(glob.glob('/tmp/*top.png'))
    col_files = [filepath.replace('top', 'rgb') for filepath in dep_files]
    # We should make sure the number of depth/color files matches the size of parsed
    if len(dep_files) != len(parsed):
        print(f'ERROR: Found {len(dep_files)} topographical map images but have {len(parsed)} rows in the output file')
        return 1

    fix, ax_list = plt.subplots(len(dep_files), 2)
    if len(dep_files) == 1:
        ax_list = [ax_list]
    for i in range(len(dep_files)):
        depImg = cv.imread(dep_files[i])
        colImg = cv.imread(col_files[i])
        ax_list[i][0].imshow(colImg)
        ax_list[i][1].imshow(depImg / np.max(depImg))
        delta = parsed[i]['sweeping'][1, :2] - parsed[i]['sweeping'][0, :2]
        ax_list[i][0].arrow(parsed[i]['sweeping'][0, 0], parsed[i]['sweeping'][0, 1],
                            delta[0], delta[1], head_width=5)

    # Convert the score data into an array for plotting
    score_data = [row['pre'] for row in parsed]
    # Add in the last 'post'
    score_data.append(parsed[-1]['post'])
    score_ax.plot(score_data)

    plt.show()

if __name__ == '__main__':
    main()
