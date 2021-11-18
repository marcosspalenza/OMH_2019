import time
import itertools
import numpy as np
import random
from sklearn.metrics import pairwise_distances
from collections import defaultdict
from clstr_search_mh import ClusteringSearch

class Particle:
    def __init__(self):
        self.vehicle_allocation = []
        self.pbest = []
        self.pbest_cost = np.inf
        self.alloc_cost = 0
        self.velocity = []
    
class PSO:
    def __init__(self, graph_, timer_=300, vmax_=4, n_particles_=50, epochs_=10, vehicles_=4):
        self.__graph = graph_
        self.__n_particles = n_particles_
        self.__vmax = vmax_
        self.__vehicles = vehicles_
        self.__epochs = epochs_
        self.__timer = timer_
        self.__best = None
        self.__distances = pairwise_distances(self.__graph.vertices, metric="euclidean")

    def execute(self, apply_clustering = True):
        epoch = 0
        temporizer = time.time()
        population = self.__generate_particles__()
        
        timer4best = 0
        cycle4best = 0

        e = 0
        t = 0
        bestid = [pid for pid, _ in sorted(enumerate([p.alloc_cost for p in population]), key = lambda x :x[1])][0]
        self.__best = population[bestid]
        while (time.time() - temporizer) < self.__timer:
            e = e + 1
            bestid = [pid for pid, _ in sorted(enumerate([p.alloc_cost for p in population]), key = lambda x :x[1])][0]

            if population[bestid] != self.__best:
                timer4best = time.time() - temporizer
                cycle4best = e
                self.__best = population[bestid]
            
            if e % 1000 == 0:
               print("___________________"+str(e)+" : "+str(time.time() - temporizer))
               print(self.__best.alloc_cost)

            for p in population:
                self.__update_velocity__(p)
                self.__update_position__(p)

            if apply_clustering and e % 50 == 0:
                data = np.array([p.vehicle_allocation for p in population])
                csearch = ClusteringSearch(graph_=self.__graph, population_=data, best_=self.__best.vehicle_allocation)
                clstr_solution = csearch.execute(method="aprop", threshold_=0.7)
            
                ptcl = Particle()
                ptcl.vehicle_allocation = clstr_solution
                self.__evaluate_routes__(ptcl)

                if ptcl.alloc_cost < population[bestid].alloc_cost:
                    timer4best = time.time() - temporizer
                    cycle4best = e
                    population[bestid].vehicle_allocation = clstr_solution
                    self.__evaluate_routes__(population[bestid])
        
        print(self.__best.alloc_cost)
        print("Time: "+str(timer4best))
        print("Epoch: "+str(cycle4best)+" - Total : "+str(e))
        print(self.__best.vehicle_allocation)
        # self.__graph.plot_graph(self.__list2solution__(self.__best.vehicle_allocation))
        return self.__best.vehicle_allocation, population


    def __list2solution__(self, solution, depot=0):
        vehicle = 0
        route = defaultdict(set)
        for rt in solution:
            if rt != depot:
                route[vehicle].add(rt)
            elif len(list(route[vehicle])) > 1:
                vehicle = vehicle + 1
        return route

    def __solution2list__(self, solution):
        route = []
        route.append(self.__graph.depot)
        for vehicle in solution.keys():
            [route.append(r) for r in solution[vehicle]]
            route.append(self.__graph.depot)
        return route

    def __generate_particles__(self):
        population = []
        for _ in range(self.__n_particles):
            p = Particle()
            routes = [i for i in range(1, len(self.__graph.vertices))]
            np.random.shuffle(routes)
            vehicle = 1
            weight = 0
            p_route = defaultdict(set)
            for r in routes:
                weight = weight + self.__graph.demands[r]
                if weight < self.__graph.car_capacity:
                    p_route[vehicle].add(r)
                else:
                    if vehicle < self.__vehicles:
                        vehicle = vehicle + 1
                    weight = self.__graph.demands[r]
                    p_route[vehicle].add(r)
            p.vehicle_allocation = self.__solution2list__(p_route)
            self.__evaluate_routes__(p)
            p.velocity = [0 for _ in range(len(p.vehicle_allocation))]
            population.append(p)
        return population

    def __evaluate_weights__(self, particle):
        weight = []
        depotid = [lid for lid, local_ in enumerate(particle.vehicle_allocation) if local_ == self.__graph.depot]
        start = 0
        for did in depotid[1:]:
            weight.append(sum([self.__graph.demands[w] for w in particle.vehicle_allocation[start:did]]))
            start = did
        return weight

    def __update_position__(self, particle):
        bias = np.random.random()
        indices = [pid for pid, p in enumerate(particle.velocity) if p > bias]

        tests = []
        
        opt = float(particle.alloc_cost)+1
        while len(indices) > 1 and opt > particle.alloc_cost:
            opt = float(particle.alloc_cost)
            old_cost = particle.alloc_cost

            np.random.shuffle(indices)

            idx1, idx2 = np.random.randint(0, len(indices)), np.random.randint(1, len(particle.vehicle_allocation)-1)

            i1, i2 = indices[idx1], idx2 # indices[idx2]

            if i1 == i2:
                i2 = np.random.randint(1, len(particle.vehicle_allocation)-1)

            item1, item2 = particle.vehicle_allocation[i1], particle.vehicle_allocation[i2]

            particle.vehicle_allocation[i1] = item2
            particle.vehicle_allocation[i2] = item1

            indices = indices[:idx1-1] + indices[idx1:]

            if True in [vehicle_weight > self.__graph.car_capacity for vehicle_weight in self.__evaluate_weights__(particle)]:
                particle.vehicle_allocation[i1] = item1
                particle.vehicle_allocation[i2] = item2
            else:
                self.__evaluate_routes__(particle)
                if particle.alloc_cost < old_cost:
                    improve = old_cost - particle.alloc_cost
                else:
                    particle.vehicle_allocation[i1]=item1
                    particle.vehicle_allocation[i2]=item2
                    particle.alloc_cost = old_cost

    def __update_velocity__(self, particle, inertia = 1, acc_c1 = 0.1, acc_c2 = 0.1):
        w = np.multiply(inertia,particle.velocity)
        local_ = np.multiply((acc_c1 * np.random.random()), np.subtract(particle.pbest, particle.vehicle_allocation))
        global_= np.multiply((acc_c2 * np.random.random()), np.subtract(self.__best.vehicle_allocation, particle.vehicle_allocation))
        new_velocity =  w +  local_ + global_
        for vid, v_ in enumerate(new_velocity):
            if v_ > self.__vmax:
                new_velocity[vid] = self.__vmax
            elif v_ < 0.0:
                new_velocity[vid] = 0.0
        particle.velocity = new_velocity

    def __evaluate_routes__(self, particle):
        cost = 0
        depotid = [lid for lid, local_ in enumerate(particle.vehicle_allocation) if local_ == self.__graph.depot]
        start = 0

        for did in depotid[1:]:
            route = particle.vehicle_allocation[start:did]
            local_ = self.__graph.depot
            for r_ in route:
                cost = cost + self.__distances[local_, r_]
                local_ = r_
            cost = cost + self.__distances[local_, self.__graph.depot]
            start = did
        particle.alloc_cost = cost
        if particle.alloc_cost < particle.pbest_cost:
            particle.pbest_cost = particle.alloc_cost
            particle.pbest = particle.vehicle_allocation
