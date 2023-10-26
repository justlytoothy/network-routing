#!/usr/bin/python3
import sys

from CS312Graph import *
import time


class ArrayPriorityQueue:
    def __init__(self, nodes):
        self.dictionary = {}
        for node in nodes:
            self.insert(node, sys.maxsize)

    # O(1)
    def insert(self, node, key):
        self.dictionary[node.node_id] = (node, key)  # O(1) to get length and assign new map entry

    # O(|V|)
    def delete_min(self):
        if not self.dictionary:
            return None
        min_val = sys.maxsize
        min_index = -1
        for node in self.dictionary.items():  # for loop runs at worst once for every vertex in graph O(|V|)
            if min_val > node[1][1]:
                min_val = node[1][1]
                min_index = node[0]
        if min_index == -1:
            return None
        return self.dictionary.pop(min_index)[0]        # O(1) for popping at index from dictionary

    # O(1)
    def decrease_key(self, node, new_key):
        self.dictionary[node.node_id] = (node, new_key)  # O(1) to assign value at index


class BinaryHeap:
    def __init__(self, nodes):
        self.heap = []
        self.dictionary = {}
        for node in nodes:
            self.insert(node, sys.maxsize)

    # O(log(|V|)
    def insert(self, node, key):
        self.heap.append((node, key))  # O(1) to append
        index = len(self.heap) - 1
        self.dictionary[node] = index  # O(1) to set value in map

        self.bubble_up(index)  # O(log(|V|) for bubble_up

    # O(log(|V|)
    def delete_min(self):
        if not self.heap:               # O(1) to check if heap not empty
            return None

        min_node, min_key = self.heap[0]

        last_node, last_key = self.heap.pop()           # O(1) to pop from end
        if self.heap:
            self.heap[0] = (last_node, last_key)        # O(1) to assign to front
            self.dictionary[last_node] = 0
        else:
            del self.dictionary[last_node]              # O(1) to delete

        self.trickle_down(0)                            # O(log(|V|)) for trickle_down

        return min_node

    # O(log(|V|)
    def decrease_key(self, node, new_key):
        if node not in self.dictionary:         # O(1) to check for node in dictionary
            return

        index = self.dictionary[node]           # O(1) Search in dictionary
        current_key = self.heap[index][1]       # O(1) access list at index

        if new_key < current_key:
            self.heap[index] = (node, new_key)  # O(1) assign value
            self.bubble_up(index)               # O(log(|V|)

    # O(log(|V|)
    def bubble_up(self, index):
        while index > 0:        # O(log|V|) because index gets roughly halved every iteration and starts as |V|
            parent_index = (index - 1) // 2
            if self.heap[index][1] < self.heap[parent_index][1]:
                index = self.swap(index, parent_index)      # O(1) to swap
            else:
                break

    # O(log(|V|)
    def trickle_down(self, index):
        while True:                                                    # O(log(|V|) worst case loop once per level of tree, max tree height is log(|V|), meaning O(log(|V|)
            left_child_index = 2 * index + 1
            right_child_index = 2 * index + 2
            smallest = index

            if (left_child_index < len(self.heap) and                       # O(1) for whole if block
                    self.heap[left_child_index][1] < self.heap[smallest][1]):
                smallest = left_child_index

            if (right_child_index < len(self.heap) and                       # O(1) for whole if block
                    self.heap[right_child_index][1] < self.heap[smallest][1]):
                smallest = right_child_index

            if smallest != index:
                index = self.swap(index, smallest)                  # O(1) to swap
            else:
                break

    # O(1)
    def swap(self, child, parent):
        self.heap[child], self.heap[parent] = self.heap[parent], self.heap[child]
        self.dictionary[self.heap[child][0]] = child            # O(1) access and modify map entry and list entry
        self.dictionary[self.heap[parent][0]] = parent          # O(1) access and modify map entry and list entry
        return parent


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
            # if type(curr) is tuple:
            #     curr = curr[0]
            for edge in curr.neighbors:
                distance = self.distances[curr] + edge.length
                if distance < self.distances[edge.dest]:
                    self.distances[edge.dest] = distance
                    self.previous[edge.dest] = (curr, edge.length)
                    queue.decrease_key(edge.dest, distance)

    def getShortestPath(self, destIndex):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL
        #       NEED TO USE
        path_edges = []
        node = self.network.nodes[destIndex]
        curr = node
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
            queue = BinaryHeap(self.network.nodes)
        else:
            queue = ArrayPriorityQueue(self.network.nodes)
        queue.decrease_key(self.network.nodes[srcIndex], 0)
        self.dijkstras(srcIndex, queue)
        t2 = time.time()
        return (t2 - t1)
