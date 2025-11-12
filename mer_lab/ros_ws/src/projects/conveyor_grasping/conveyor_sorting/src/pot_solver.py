#!/home/fadi/scrap_ws/src/projects/conveyor_sorting/pot_env/bin/python3

import rospy

import ot
import numpy as np
from sklearn.cluster import DBSCAN

from conveyor_sorting.srv import OT, OTResponse

cluster = False
eps = 0.0
min_samples = 0.5

def geometry_to_arr(geometry_pts):
    ret = np.zeros((2, len(geometry_pts)), dtype=np.float32)

    for idx, pt in enumerate(geometry_pts):
        ret[:, idx] = np.array([pt.x, pt.y], dtype=np.float32)

    return ret

def pot_handler(req):
    source = req.source
    target = req.target
    cost = np.array(req.cost).reshape((len(source), len(target)))

    # Cluster the input if required
    if cluster:
        source_arr = geometry_to_arr(source)
        clusters = DBSCAN(eps=eps, min_samples=min_samples).fit(source_arr)

        # Check how many points we have that are not noise
        num_clusters = np.max(clusters) + 1
        num_valid_points = np.sum(clusters != -1)

        # Separate into clusters
        clustered_pts = []
        for i in range(num_clusters):
            clustered_pts.append(np.where(clusters == i))

        # Sub-sample these
        target_arr = geometry_to_arr(target)
        subsampled_indices = np.random.choice(list(range(len(target_arr))), num_valid_points)

        # Compute cost array
        cost_arr = np.array(cost, dtype=np.float32).reshape((len(source), len(target)))
        clustered_cost_arr = np.zeros((num_clusters, num_valid_points), dtype=np.float32)
        for cIdx, pts in enumerate(clustered_pts):
            for tIdx, target in enumerate(subsampled_indices):
                clustered_cost_arr[cIdx, tIdx] = np.mean(cost_arr[pts, target])

        # Do conversions to get the input in the right format
        x = np.array([len(pts) for pts in clustered_pts], dtype=np.float32)
        y = np.ones(len(subsampled_indices), dtype=np.float32)
        # We now have the OT problem
        res, log = ot.emd(x, y, clustered_cost_arr, log=True)
    else:
        # Compute the optimal transport
        cost_arr = np.array(cost, dtype=np.float32).reshape((len(source), len(target)))
        res, log = ot.emd(np.ones(len(source)), np.ones(len(target)), cost, log=True)
        
        # Convert the returned matrix into a 1D association array and return that
        source_to_target = np.where(np.array(res) == 1.0)[1]

    return OTResponse(source_to_target, log['cost'])

def main():
    global cluster
    global eps
    global min_samples

    rospy.init_node('pot_solver_node')

    # Get params
    cluster = rospy.get_param('conveyor_sorting/sweeper/ot_sweeper/cluster')
    eps = rospy.get_param('conveyor_sorting/sweeper/ot_sweeper/eps')
    min_samples = rospy.get_param('conveyor_sorting/sweeper/ot_sweeper/min_samples')

    pot_service = rospy.Service('compute_optimal_transport', OT, pot_handler)

    print('Optimal transport node initialized, waiting for requests.')
    rospy.spin()

if __name__ == '__main__':
    main()
