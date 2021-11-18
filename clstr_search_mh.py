import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import AgglomerativeClustering, AffinityPropagation
from sklearn.metrics import pairwise_distances
from sklearn.preprocessing import MinMaxScaler

class ClusteringSearch:
    def __init__(self, graph_, population_, best_, metric_="euclidean"):
        self.__population = population_
        self.__graph = graph_
        self.__best = best_
        self.__metric = metric_
        self.__distances = pairwise_distances(self.__graph.vertices, metric=metric_)

    def execute(self, method="agglo", threshold_=0.5, use_nn=False):
        population_distance = pairwise_distances(self.__population, metric=self.__metric)
        distances = MinMaxScaler().fit_transform(population_distance)
        
        group = []
        clstr = None

        if method == "agglo":
            clstr = AgglomerativeClustering(n_clusters=None, affinity="precomputed", linkage="complete", compute_full_tree=True, distance_threshold=threshold_)
        else:
            clstr = AffinityPropagation(affinity="precomputed", damping=threshold_)

        groups = clstr.fit_predict(distances)

        clusters = []
        for g1 in np.unique(groups):
            cluster = [gid for gid, g2 in enumerate(groups.tolist()) if g1 == g2]
            clusters.append(cluster)

        for c in clusters:
            if len(c) < 5:
                continue
            min_centroid = [sorted([(c_, pairwise_distances(self.__population[c_].reshape(1,-1), self.__population[c].mean(axis=0).reshape(1,-1))) for c_ in c], key = lambda x :x[1])[0][0]]
            max_centroid = [sorted([(c_, pairwise_distances(self.__population[c_].reshape(1,-1), self.__population[c].mean(axis=0).reshape(1,-1))) for c_ in c], key = lambda x :x[1])[-1][0]]
            
            particle = self.__population[min_centroid].tolist()[0]
            cost0 = 1
            cost1 = 0
            counter = 0
            while cost0 > cost1:
                cost0 = self.evaluate_routes(particle)
                particle = self.local_search(particle)
                cost1 = self.evaluate_routes(particle)
                counter = counter + 1

            if self.evaluate_routes(particle) < self.evaluate_routes(self.__best):
                self.__best = particle

            particle = self.__population[max_centroid].tolist()[0]
            cost0 = 1
            cost1 = 0
            improve = []
            counter = 0
            while cost0 > cost1:
                cost0 = self.evaluate_routes(particle)
                particle = self.local_search(particle)
                cost1 = self.evaluate_routes(particle)
                counter = counter + 1

            if self.evaluate_routes(particle) < self.evaluate_routes(self.__best):
                self.__best = particle

            if use_nn:
                particle = self.nearest_route(self.__best)
                if self.evaluate_routes(particle) < self.evaluate_routes(self.__best):
                    self.__best = particle

        return self.__best

    def evaluate_routes(self, allocation):
        cost = 0
        depotid = [lid for lid, local_ in enumerate(allocation) if local_ == self.__graph.depot]
        start = 0
        for did in depotid[1:]:
            route = allocation[start:did]
            local_ = self.__graph.depot
            for r_ in route:
                cost = cost + self.__distances[local_, r_]
                local_ = r_
            cost = cost + self.__distances[local_, self.__graph.depot]
            start = did
        return cost

    def nearest_route(self, allocation):
        depotid = [lid for lid, local_ in enumerate(allocation) if local_ == self.__graph.depot]
        start = 0
        for did in depotid[1:]:
            route = allocation[start:did]
            cost0 = self.evaluate_routes(allocation)
            pos = route[0]
            best = []
            if len(route) > 2:
                for r in route[1:]:
                    best.append(pos)
                    smaller = sorted([(rid, self.__distances[pos, r]) for rid, r in enumerate(route) if r not in best], key = lambda x :x[1])
                    pos = route[smaller[0][0]]
                best.append(pos)
                allocation[start:did] = best
            start = did
        return allocation

    def local_search(self, allocation):
        depotid = [lid for lid, local_ in enumerate(allocation) if local_ == self.__graph.depot]
        start = 0
        for did in depotid[1:]:
            route = allocation[start:did]
            cost0 = self.evaluate_routes(allocation)
            if len(route) < 2:
                continue
            improve = True
            while improve:
                pos1 = np.random.randint(1,len(route))
                pos2 = np.random.randint(1,len(route))
                x1, x2 = route[pos1], route[pos2]
                route[pos1] = x2
                route[pos2] = x1
                allocation[start:did] = route
                cost1 = self.evaluate_routes(allocation)
                improve = cost0 > cost1
                if not improve:
                    route[pos1] = x1
                    route[pos2] = x2
                    allocation[start:did] = route
                else:
                    cost0 = cost1
            start = did + 1
        return allocation