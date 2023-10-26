#!/usr/bin/python3
import sys

from CS312Graph import *
import time


class ArrayPriorityQueue:
    def __init__(self, nodes):
        self.queue = []
        for node in nodes:
            self.insert(node, sys.maxsize)

    def insert(self, node, key):
        self.queue.append((node, key))

    def delete_min(self):
        if not self.queue:
            return None
        min_val = sys.maxsize
        min_node = None
        min_index = 0
        for i in range(len(self.queue)):
            node = self.queue[i]
            if min_val > node[1]:
                min_val = node[1]
                min_node = node[0]
                min_index = i

        return self.queue.pop(min_index)

    def decrease_key(self, node, new_key):
        for i in range(len(self.queue)):
            if self.queue[i][0] == node:
                if new_key < self.queue[i][1]:
                    self.queue[i] = (node, new_key)
                break


class PriorityArray:
    def __init__(self, nodes):
        self.arr = []
        self.dictionary = {}
        for node in nodes:
            self.insert(node, sys.maxsize)

    def insert(self, node, key):
        self.arr.append((node, key))
        self.dictionary[node] = len(self.arr) - 1

    def get_distance(self, node):
        index = self.dictionary[node]
        return self.arr[index][1]

    def delete_min(self):
        minVal = sys.maxsize
        minNode = None
        for node in self.arr:
            if node[1] < minVal:
                minVal = node[1]
                minNode = node[0]
        index = self.dictionary[minNode]
        return self.arr.pop(index)

    def decrease_key(self, node, new_key):
        index = self.dictionary[node]
        self.arr[index] = (node, new_key)

    def is_empty(self):
        return len(self.arr) == 0


class Heap:
    def __init__(self, nodes):
        self.heap = []
        self.dictionary = {}
        for node in nodes:
            self.insert(node, sys.maxsize)

    def insert(self, node, key):
        self.dictionary[node] = len(self.heap)
        self.heap.append((node, key))
        self.bubble_up(len(self.heap) - 1)

    def get_distance(self, node):
        index = self.dictionary[node]
        return self.heap[index][1]

    def delete_min(self):
        if not self.heap:
            return None

        min_node, min_key = self.heap[0]
        last_node, last_key = self.heap.pop()
        del self.dictionary[min_node]

        if self.heap:
            self.heap[0] = (last_node, last_key)
            self.dictionary[last_node] = 0
            self.bubble_down(0)

        # return min_node, min_key
        return min_node

    def decrease_key(self, node, new_key):
        if node in self.dictionary:
            index = self.dictionary[node]
            current_key = self.heap[index][1]
            if new_key < current_key:
                self.heap[index] = (node, new_key)
                self.bubble_up(index)

    def bubble_up(self, index):
        while index > 0:
            parent = (index - 1) // 2
            if self.heap[index][1] < self.heap[parent][1]:
                self._swap(index, parent)
                index = parent
            else:
                break

    def bubble_down(self, index):
        while True:
            left_child = 2 * index + 1
            right_child = 2 * index + 2
            smallest = index

            if left_child < len(self.heap) and self.heap[left_child][1] < self.heap[smallest][1]:
                smallest = left_child

            if right_child < len(self.heap) and self.heap[right_child][1] < self.heap[smallest][1]:
                smallest = right_child

            if smallest != index:
                self._swap(index, smallest)
                index = smallest
            else:
                break

    def _swap(self, index, parent):
        self.heap[index], self.heap[parent] = self.heap[parent], self.heap[index]
        self.dictionary[self.heap[index][0]] = index
        self.dictionary[self.heap[parent][0]] = parent

    def is_empty(self):
        return len(self.heap) == 0


class NetworkRoutingSolver:
    def __init__(self):
        self.previous = {}
        self.distances = {}
        pass

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def dijkstras(self, srcIndex, queue):
        self.distances = {node: sys.maxsize for node in self.network.nodes}
        self.previous = {node: None for node in self.network.nodes}
        self.distances[self.network.nodes[srcIndex]] = 0
        while True:
            curr = queue.delete_min()
            if curr is None:
                break
            for edge in curr[0].neighbors:
                distance = self.distances[curr[0]] + edge.length
                if distance < self.distances[edge.dest]:
                    self.distances[edge.dest] = distance
                    self.previous[edge.dest] = (curr[0], edge.length)
                    queue.decrease_key(edge.dest, distance)

    def getShortestPath(self, destIndex):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL
        #       NEED TO USE
        path_edges = []
        total_length = 0
        node = self.network.nodes[destIndex]
        print(self.previous[node])
        curr = node
        edges_left = 3
        while self.previous[curr] is not None:
            edge = self.previous[curr]
            path_edges.append((curr.loc, edge[0].loc, '{:.0f}'.format(edge[1])))
            curr = self.previous[curr][0]
        return {'cost': self.distances[node], 'path': path_edges}

    def computeShortestPaths(self, srcIndex, use_heap=False):
        self.source = srcIndex
        t1 = time.time()
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)
        queue = None
        if use_heap:
            queue = Heap(self.network.nodes)
        else:
            queue = ArrayPriorityQueue(self.network.nodes)
        queue.decrease_key(self.network.nodes[srcIndex], 0)
        self.dijkstras(srcIndex, queue)
        t2 = time.time()
        return (t2 - t1)
