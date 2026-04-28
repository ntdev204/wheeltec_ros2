
import numpy as np
import math

import os
import pickle

import seaborn as sns
import matplotlib.pylab as plt
from tabulate import tabulate


def getPaths(results):
    paths = []
    for i in range(len(results)):
        if (i % 2) == 0:
            paths.append(results[i].path)
        else:
            for result in results[i]:
                paths.append(result.path)
    return paths


def getTimes(results):
    times = []
    for i in range(len(results)):
        if (i % 2) == 0:
            times.append(results[i].planning_time.nanosec/1e09 + results[i].planning_time.sec)
        else:
            for result in results[i]:
                times.append(result.smoothing_duration.nanosec/1e09 + result.smoothing_duration.sec)
    return times


def getMapCoordsFromPaths(paths, resolution):
    coords = []
    for path in paths:
        x = []
        y = []
        for pose in path.poses:
            x.append(pose.pose.position.x/resolution)
            y.append(pose.pose.position.y/resolution)
        coords.append(x)
        coords.append(y)
    return coords


def getPathLength(path):
    path_length = 0
    x_prev = path.poses[0].pose.position.x
    y_prev = path.poses[0].pose.position.y
    for i in range(1, len(path.poses)):
        x_curr = path.poses[i].pose.position.x
        y_curr = path.poses[i].pose.position.y
        path_length = path_length + math.sqrt((x_curr-x_prev)**2 + (y_curr-y_prev)**2)
        x_prev = x_curr
        y_prev = y_curr
    return path_length

def getSmoothness(pt_prev, pt, pt_next):
    d1 = pt - pt_prev
    d2 = pt_next - pt
    delta = d2 - d1
    return np.dot(delta, delta)

def getPathSmoothnesses(paths):
    smoothnesses = []
    pm0 = np.array([0.0, 0.0])
    pm1 = np.array([0.0, 0.0])
    pm2 = np.array([0.0, 0.0])
    for path in paths:
        smoothness = 0.0
        for i in range(2, len(path.poses)):
            pm0[0] = path.poses[i].pose.position.x
            pm0[1] = path.poses[i].pose.position.y
            pm1[0] = path.poses[i-1].pose.position.x
            pm1[1] = path.poses[i-1].pose.position.y
            pm2[0] = path.poses[i-2].pose.position.x
            pm2[1] = path.poses[i-2].pose.position.y
            smoothness += getSmoothness(pm2, pm1, pm0)
        smoothnesses.append(smoothness)
    return smoothnesses

def arcCenter(pt_prev, pt, pt_next):
    cusp_thresh = -0.7

    d1 = pt - pt_prev
    d2 = pt_next - pt

    d1_norm = d1 / np.linalg.norm(d1)
    d2_norm = d2 / np.linalg.norm(d2)
    cos_angle = np.dot(d1_norm, d2_norm)

    if cos_angle < cusp_thresh:
        d2 = -d2
        pt_next = pt + d2

    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-4:
        return (float('inf'), float('inf'))

    mid1 = (pt_prev + pt) / 2
    mid2 = (pt + pt_next) / 2
    n1 = (-d1[1], d1[0])
    n2 = (-d2[1], d2[0])
    det1 = (mid1[0] + n1[0]) * mid1[1] - (mid1[1] + n1[1]) * mid1[0]
    det2 = (mid2[0] + n2[0]) * mid2[1] - (mid2[1] + n2[1]) * mid2[0]
    center = np.array([(det1 * n2[0] - det2 * n1[0]) / det, (det1 * n2[1] - det2 * n1[1]) / det])
    return center

def getPathCurvatures(paths):
    curvatures = []
    pm0 = np.array([0.0, 0.0])
    pm1 = np.array([0.0, 0.0])
    pm2 = np.array([0.0, 0.0])
    for path in paths:
        radiuses = []
        for i in range(2, len(path.poses)):
            pm0[0] = path.poses[i].pose.position.x
            pm0[1] = path.poses[i].pose.position.y
            pm1[0] = path.poses[i-1].pose.position.x
            pm1[1] = path.poses[i-1].pose.position.y
            pm2[0] = path.poses[i-2].pose.position.x
            pm2[1] = path.poses[i-2].pose.position.y
            center = arcCenter(pm2, pm1, pm0)
            if center[0] != float('inf'):
              turning_rad = np.linalg.norm(pm1 - center);
              radiuses.append(turning_rad)
        curvatures.append(np.average(radiuses))
    return curvatures

def plotResults(costmap, paths):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)
    data = np.where(data <= 253, 0, data)

    plt.figure(3)
    ax = sns.heatmap(data, cmap='Greys', cbar=False)
    for i in range(0, len(coords), 2):
        ax.plot(coords[i], coords[i+1], linewidth=0.7)
    plt.axis('off')
    ax.set_aspect('equal', 'box')
    plt.show()


def averagePathCost(paths, costmap, num_of_planners):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)

    average_path_costs = []
    for i in range(num_of_planners):
        average_path_costs.append([])

    k = 0
    for i in range(0, len(coords), 2):
        costs = []
        for j in range(len(coords[i])):
            costs.append(data[math.floor(coords[i+1][j])][math.floor(coords[i][j])])
        average_path_costs[k % num_of_planners].append(sum(costs)/len(costs))
        k += 1

    return average_path_costs


def maxPathCost(paths, costmap, num_of_planners):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)

    max_path_costs = []
    for i in range(num_of_planners):
        max_path_costs.append([])

    k = 0
    for i in range(0, len(coords), 2):
        max_cost = 0
        for j in range(len(coords[i])):
            cost = data[math.floor(coords[i+1][j])][math.floor(coords[i][j])]
            if max_cost < cost:
                max_cost = cost
        max_path_costs[k % num_of_planners].append(max_cost)
        k += 1

    return max_path_costs


def main():
    benchmark_dir = os.getcwd()
    print("Read data")
    with open(os.path.join(benchmark_dir, 'results.pickle'), 'rb') as f:
        results = pickle.load(f)

    with open(os.path.join(benchmark_dir, 'methods.pickle'), 'rb') as f:
        smoothers = pickle.load(f)
    planner = smoothers[0]
    del smoothers[0]
    methods_num = len(smoothers) + 1

    with open(os.path.join(benchmark_dir, 'costmap.pickle'), 'rb') as f:
        costmap = pickle.load(f)

    paths = getPaths(results)
    path_lengths = []

    for path in paths:
        path_lengths.append(getPathLength(path))
    path_lengths = np.asarray(path_lengths)
    total_paths = len(paths)

    path_lengths.resize((int(total_paths/methods_num), methods_num))
    path_lengths = path_lengths.transpose()

    times = getTimes(results)
    times = np.asarray(times)
    times.resize((int(total_paths/methods_num), methods_num))
    times = np.transpose(times)

    average_path_costs = np.asarray(averagePathCost(paths, costmap, methods_num))
    max_path_costs = np.asarray(maxPathCost(paths, costmap, methods_num))

    smoothnesses = getPathSmoothnesses(paths)
    smoothnesses = np.asarray(smoothnesses)
    smoothnesses.resize((int(total_paths/methods_num), methods_num))
    smoothnesses = np.transpose(smoothnesses)

    curvatures = getPathCurvatures(paths)
    curvatures = np.asarray(curvatures)
    curvatures.resize((int(total_paths/methods_num), methods_num))
    curvatures = np.transpose(curvatures)

    planner_table = [['Planner',
                      'Time (s)',
                      'Path length (m)',
                      'Average cost',
                      'Max cost',
                      'Path smoothness (x100)',
                      'Average turning rad (m)']]
    planner_table.append([planner,
                          np.average(times[0]),
                          np.average(path_lengths[0]),
                          np.average(average_path_costs[0]),
                          np.average(max_path_costs[0]),
                          np.average(smoothnesses[0]) * 100,
                          np.average(curvatures[0])])
    for i in range(1, methods_num):
        planner_table.append([smoothers[i-1],
                              np.average(times[i]),
                              np.average(path_lengths[i]),
                              np.average(average_path_costs[i]),
                              np.average(max_path_costs[i]),
                              np.average(smoothnesses[i]) * 100,
                              np.average(curvatures[i])])

    print(tabulate(planner_table))
    plotResults(costmap, paths)

    exit(0)


if __name__ == '__main__':
    main()
