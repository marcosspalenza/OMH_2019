import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt 
from pso_mh import PSO
from clstr_search_mh import ClusteringSearch

class GraphMap:
    def __init__(self, capacity, vertices, depot_idx, demands):
        self.car_capacity = float(capacity)
        self.depot = depot_idx[0]
        self.vertices = [[float(x), float(y)] for sid, x, y in vertices]
        self.demands = [float(w) for sid, w in demands]

    def plot_graph(self, routes):
        if routes != None:
            for v in routes.keys():
                loc = 0
                cm = plt.cm.get_cmap("tab20c")
                loc = 0
                seq = []
                seq.append(self.vertices[0])
                for r_ in routes[v]:
                    seq.append(self.vertices[r_])
                    loc = r_
                seq.append(self.vertices[0])
                plt.plot(np.array(seq)[:, 0],np.array(seq)[:, 1], linestyle='-', color = cm(v), marker='o')

        plt.title('VRP Graph') 
        plt.show()

class OptTask:
    def __init__(self, fname_, comment_, opt_type_, dimensions_, w_metric_):
        self.fname = fname_
        self.comment = comment_
        self.opt_type = opt_type_
        self.dimm = dimensions_ 
        self.metric = w_metric_

    def __init__(self, tasklst):
        self.fname = tasklst[0][0]
        self.vehicles = int(tasklst[1][2].split(",")[0])
        self.optimal = float(tasklst[1][3][:-1])
        self.opt_type = tasklst[2][0]
        self.dimm = int(tasklst[3][0])
        self.metric = tasklst[4][0]

    def to_string(self):
        return " {} \n {} \n {} \n {} \n {} \n {}".format(self.fname, self.vehicles, self.optimal, self.opt_type, self.dimm, self.metric)

def read_file(filename):
    problem = []
    with open(filename, "r") as d_rdb:
        problem = d_rdb.read()
    problem = [content for content in problem.split("\n") if content != ""]

    opttask = OptTask([p.split(":")[1:] for p in problem[:5]])
    capacity = problem[5].split(":")[1].strip()
    # problem[6] - NODE_COORD_SECTION 
    # ID X Y
    nodes = [problem[i].strip().split() for i in range(7, opttask.dimm+7)]
    # problem[opttask.dimm+7] - DEMAND_SECTION 
    # ID W
    demands = [problem[i].strip().split() for i in range(opttask.dimm+8, (opttask.dimm*2)+8)]
    # problem[(opttask.dimm*2)+7+1] - DEPOT_SECTION 
    # ID
    depot = [int(p)-1 for p in problem[(opttask.dimm*2)+9: ] if p.strip() not in ["-1", "EOF"]]

    graph = GraphMap(capacity, nodes, depot, demands)
    print(opttask.to_string())

    return graph, opttask.vehicles, opttask.optimal

def main():
    parser = argparse.ArgumentParser(usage="%prog [options] <filename>", description="Particle Swarm Optimization for Vehicle Routing Problem.")
    parser.add_argument('data_path', help='Dataset source.')
    args = parser.parse_args()

    for filename in os.listdir(args.data_path):
        print(filename)
        if ".vrp" in filename:
            print(filename)
            locations, vehicles, optimal = read_file(args.data_path+filename)
            swarm = PSO(locations, timer_=300, n_particles_=500, vehicles_=vehicles)
            best, pop = swarm.execute(apply_clustering=False)

            data = np.array([p.vehicle_allocation for p in population])

            csearch = ClusteringSearch(graph_=locations, population_=data, best_=best, threshold_=0.7)

            csearch.execute()

if __name__ == "__main__":
    main()