import numpy as np
import matplotlib.pyplot as plt

def read3DPoints(filepath):
    lines = open(filepath).read().split('\n')[:-1]

    pts = np.zeros((len(lines), 3))

    for lineIdx, line in enumerate(lines):
        pts[lineIdx, :] = np.array([float(pt) for pt in line.split(',')])

    return pts

def main():
    sourceFile = read3DPoints('/tmp/source.csv')
    targetFile = read3DPoints('/tmp/target.csv')
    sweepingAction = read3DPoints('/tmp/sweeping.csv')
    association = [int(val) for val in open('/tmp/association.csv').read()[:-1].split('\n')]

    print('Read all data')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(sourceFile[:, 0], sourceFile[:, 1], sourceFile[:, 2])
    ax.scatter(targetFile[:, 0], targetFile[:, 1], targetFile[:, 2])

    for sourceIdx in range(len(association)):
        sourcePt = sourceFile[sourceIdx]
        targetPt = targetFile[association[sourceIdx]]
        ax.plot([sourcePt[0], targetPt[0]], [sourcePt[1], targetPt[1]], [sourcePt[2], targetPt[2]], color='red')
        ax.scatter([sweepingAction[0, 0]], [sweepingAction[0, 1]], [sweepingAction[0, 2]], color='green')
        ax.plot(sweepingAction[:, 0], sweepingAction[:, 1], sweepingAction[:, 2], color='green')

    plt.show()

if __name__ == '__main__':
    main()
