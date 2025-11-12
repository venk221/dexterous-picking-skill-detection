import sys
import os
import argparse

import matplotlib.pyplot as plt
import numpy as np

def get_run_scores(score_line):
    ret = []
    scores = score_line.split(' ')
    ret.append(float(scores[0]))
    i = 1
    while i < len(scores):
        # This score is post-run
        ret.append(float(scores[i]))

        i += 2

    return ret

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filepath", type=str)
    parser.add_argument("secondary", type=str, nargs=argparse.REMAINDER)
    args = parser.parse_args()

    score_file = open(os.path.join(args.filepath, "scores.csv")).read()
    score_file = score_file.split('\n')

    scores = []
    for score_line in score_file[:-1]:
        scores.append(get_run_scores(score_line))

    # We will have multiple plots for visualization
    # First plot, we display all values as multiple lines
    fig, ax = plt.subplots(1, 1)
    for score in scores:
        print(score)
        ax.plot(score)

    # Next plot is the average score at each interval
    scores = np.array(scores, dtype=float)
    print(np.average(scores, 0))
    av_f, av_ax = plt.subplots(1, 1)
    av_ax.plot(np.average(scores, 0), label=os.path.basename(args.filepath))

    ax.plot(np.average(scores, 0))

    # If we are given a secondary data folder, plot that alongside this in the average
    if args.secondary:
        for sec in args.secondary:
            sec_score_file = open(os.path.join(sec, "scores.csv")).read()
            sec_score_file = sec_score_file.split('\n')

            sec_scores = []
            for score_line in sec_score_file[:-1]:
                sec_scores.append(get_run_scores(score_line))
            # print(np.array(sec_scores))
            av_ax.plot(np.average(np.array(sec_scores, dtype=float), 0), label=os.path.basename(sec))
    av_ax.legend()

    plt.show()

if __name__ == '__main__':
    main()
