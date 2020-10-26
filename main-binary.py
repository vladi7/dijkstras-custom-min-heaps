import math
import heapq #binary heap, O(log n) push and O(log n) pop
import time
#https://github.com/arlieu/interview-guide/blob/master/algorithms/graphing/dynamic_programming/dijkstra.py
class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        self.dist = math.inf
        self.visited = False
        self.previous = None

    def __lt__(self, vertex2):
        return self.id < vertex2.id

    def __le__(self, vertex2):
        return self.id <= vertex2.id

    def __gt__(self, vertex2):
        return self.id > vertex2.id

    def __ge__(self, vertex2):
        return self.id >= vertex2.id

    def addNeighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def getNeighbors(self):
        return self.adjacent.keys()

    def getId(self):
        return self.id

    def getWeight(self, neighbor):
        return self.adjacent[neighbor]

    def setdist(self, dist):
        self.dist = dist

    def getdist(self):
        return self.dist

    def setPrevious(self, prev):
        self.previous = prev

    def setVisited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])


class Graph:
    def __init__(self):
        self.vertices = {}
        self.nVertices = 0

    def __iter__(self):
        return iter(self.vertices.values())

    def addVertex(self, node):
        self.nVertices += 1
        newVertex = Vertex(node)
        self.vertices[node] = newVertex
        return newVertex

    def getVertex(self, n):
        if n in self.vertices:
            return self.vertices[n]

        return None

    def addEdge(self, source, dest, cost=0):
        if source not in self.vertices:
            self.addVertex(source)

        if dest not in self.vertices:
            self.addVertex(dest)

        self.vertices[source].addNeighbor(self.vertices[dest], cost)
        self.vertices[dest].addNeighbor(self.vertices[source], cost)

    def getVertices(self):
        return self.vertices.keys()

    def setPrevious(self, current):
        self.previous = current

    def getPrevious(self, current):
        return self.previous
    
    def resetVertices(self):
        for id, vertex in self.vertices.items():
            vertex.dist = math.inf
            vertex.visited = False
            vertex.previous = None

def dijkstra(aGraph, start):
    start.setdist(0)
    unvisited = []
    for v in aGraph:
       if not v.visited:
            heapq.heappush(unvisited, (v.getdist(),v)) 
    while len(unvisited):
        uv = heapq.heappop(unvisited)
        current = uv[1]
        current.setVisited()

        for next in current.adjacent:
            if next.visited:
                continue

            newDist = current.getdist() + current.getWeight(next)

            if newDist < next.getdist():
                next.setdist(newDist)
                next.setPrevious(current)
        while len(unvisited):
            uv = heapq.heappop(unvisited)
    
        for v in aGraph:
            if not v.visited:
                heapq.heappush(unvisited, (v.getdist(),v)) 


if __name__ == '__main__':
    start = time.time()

    #dists = [{'zip1': 64149, 'zip2': 64150, 'dist': 4}, {'zip1': 64150, 'zip2': 64151, 'dist': 2}, {'zip1': 64151, 'zip2': 64152, 'dist': 3}, {'zip1': 64150, 'zip2': 64153, 'dist': 2}, {'zip1': 64152, 'zip2': 64153, 'dist': 4}, {'zip1': 64150, 'zip2': 64154, 'dist': 7}, {'zip1': 64154, 'zip2': 64155, 'dist': 6}, {'zip1': 64151, 'zip2': 64154, 'dist': 6}, {'zip1': 64152, 'zip2': 64156, 'dist': 4}, {'zip1': 64153, 'zip2': 64156, 'dist': 4}, {'zip1': 64151, 'zip2': 64157, 'dist': 5}, {'zip1': 64156, 'zip2': 64157, 'dist': 6}, {'zip1': 64156, 'zip2': 64158, 'dist': 5}, {'zip1': 64158, 'zip2': 64159, 'dist': 6}, {'zip1': 64159, 'zip2': 64153, 'dist': 5}, {'zip1': 64158, 'zip2': 64160, 'dist': 5}, {'zip1': 64160, 'zip2': 64161, 'dist': 4}, {'zip1': 64161, 'zip2': 64162, 'dist': 7}, {'zip1': 64162, 'zip2': 64163, 'dist': 3}, {'zip1': 64163, 'zip2': 64164, 'dist': 6}, {'zip1': 64164, 'zip2': 64165, 'dist': 8}, {'zip1': 64165, 'zip2': 64162, 'dist': 7}, {'zip1': 64164, 'zip2': 64166, 'dist': 4}, {'zip1': 64166, 'zip2': 64167, 'dist': 9}, {'zip1': 64167, 'zip2': 64157, 'dist': 11}]
    #requests = [{'id': 1, 'type': 2, 'zip': 64167, 'vehicle': None, 'dist': 0}, {'id': 2, 'type': 3, 'zip': 64160, 'vehicle': None, 'dist': 0}, {'id': 3, 'type': 1, 'zip': 64150, 'vehicle': None, 'dist': 0}, {'id': 4, 'type': 1, 'zip': 64152, 'vehicle': None, 'dist': 0}, {'id': 5, 'type': 2, 'zip': 64157, 'vehicle': None, 'dist': 0}, {'id': 6, 'type': 3, 'zip': 64159, 'vehicle': None, 'dist': 0}, {'id': 7, 'type': 3, 'zip': 64158, 'vehicle': None, 'dist': 0}, {'id': 8, 'type': 2, 'zip': 64152, 'vehicle': None, 'dist': 0}, {'id': 9, 'type': 1, 'zip': 64164, 'vehicle': None, 'dist': 0}, {'id': 11, 'type': 1, 'zip': 64166, 'vehicle': None, 'dist': 0}, {'id': 12, 'type': 2, 'zip': 64154, 'vehicle': None, 'dist': 0}, {'id': 13, 'type': 2, 'zip': 64151, 'vehicle': None, 'dist': 0}, {'id': 14, 'type': 3, 'zip': 64167, 'vehicle': None, 'dist': 0}, {'id': 15, 'type': 1, 'zip': 64165, 'vehicle': None, 'dist': 0}, {'id': 16, 'type': 2, 'zip': 64153, 'vehicle': None, 'dist': 0}, {'id': 17, 'type': 2, 'zip': 64152, 'vehicle': None, 'dist': 0}, {'id': 18, 'type': 3, 'zip': 64164, 'vehicle': None, 'dist': 0}, {'id': 20, 'type': 1, 'zip': 64165, 'vehicle': None, 'dist': 0}, {'id': 21, 'type': 2, 'zip': 64163, 'vehicle': None, 'dist': 0}, {'id': 22, 'type': 3, 'zip': 64150, 'vehicle': None, 'dist': 0}, {'id': 23, 'type': 3, 'zip': 64160, 'vehicle': None, 'dist': 0}, {'id': 24, 'type': 2, 'zip': 64153, 'vehicle': None, 'dist': 0}, {'id': 25, 'type': 1, 'zip': 64161, 'vehicle': None, 'dist': 0}]
    #vehicles = [{'id': 1, 'type': 1, 'zip': 64149, 'free': True}, {'id': 2, 'type': 1, 'zip': 64149, 'free': True}, {'id': 3, 'type': 2, 'zip': 64149, 'free': True}, {'id': 4, 'type': 3, 'zip': 64149, 'free': True}, {'id': 5, 'type': 3, 'zip': 64150, 'free': True}, {'id': 6, 'type': 3, 'zip': 64151, 'free': True}, {'id': 7, 'type': 2, 'zip': 64152, 'free': True}, {'id': 8, 'type': 3, 'zip': 64152, 'free': True}, {'id': 9, 'type': 1, 'zip': 64153, 'free': True}, {'id': 10, 'type': 2, 'zip': 64153, 'free': True}, {'id': 11, 'type': 3, 'zip': 64154, 'free': True}, {'id': 12, 'type': 2, 'zip': 64155, 'free': True}, {'id': 13, 'type': 2, 'zip': 64155, 'free': True}, {'id': 14, 'type': 1, 'zip': 64156, 'free': True}, {'id': 15, 'type': 1, 'zip': 64156, 'free': True}, {'id': 16, 'type': 2, 'zip': 64156, 'free': True}, {'id': 17, 'type': 1, 'zip': 64157, 'free': True}, {'id': 18, 'type': 3, 'zip': 64157, 'free': True}, {'id': 19, 'type': 1, 'zip': 64158, 'free': True}, {'id': 20, 'type': 1, 'zip': 64158, 'free': True}, {'id': 21, 'type': 2, 'zip': 64158, 'free': True}, {'id': 22, 'type': 3, 'zip': 64158, 'free': True}, {'id': 23, 'type': 3, 'zip': 64158, 'free': True}, {'id': 24, 'type': 2, 'zip': 64159, 'free': True}, {'id': 25, 'type': 3, 'zip': 64159, 'free': True}, {'id': 26, 'type': 1, 'zip': 64160, 'free': True}, {'id': 27, 'type': 2, 'zip': 64161, 'free': True}, {'id': 28, 'type': 2, 'zip': 64161, 'free': True}, {'id': 29, 'type': 1, 'zip': 64162, 'free': True}, {'id': 30, 'type': 3, 'zip': 64162, 'free': True}, {'id': 31, 'type': 3, 'zip': 64162, 'free': True}, {'id': 32, 'type': 1, 'zip': 64163, 'free': True}, {'id': 33, 'type': 2, 'zip': 64163, 'free': True}, {'id': 34, 'type': 2, 'zip': 64163, 'free': True}, {'id': 35, 'type': 2, 'zip': 64163, 'free': True}, {'id': 36, 'type': 3, 'zip': 64163, 'free': True}, {'id': 37, 'type': 3, 'zip': 64163, 'free': True}, {'id': 38, 'type': 3, 'zip': 64163, 'free': True}]
    dists = [{
			"zip1": 64149,
			"zip2": 64150,
			"dist": 4
		},
		{
			"zip1": 64150,
			"zip2": 64151,
			"dist": 2
		},
		{
			"zip1": 64151,
			"zip2": 64152,
			"dist": 3
		},
		{
			"zip1": 64150,
			"zip2": 64153,
			"dist": 2
		},
		{
			"zip1": 64152,
			"zip2": 64153,
			"dist": 4
		},
		{
			"zip1": 64150,
			"zip2": 64154,
			"dist": 7
		},
		{
			"zip1": 64154,
			"zip2": 64155,
			"dist": 6
		},
		{
			"zip1": 64151,
			"zip2": 64154,
			"dist": 6
		},
		{
			"zip1": 64152,
			"zip2": 64156,
			"dist": 4
		},
		{
			"zip1": 64153,
			"zip2": 64156,
			"dist": 4
		},

		{
			"zip1": 64151,
			"zip2": 64157,
			"dist": 5
		},

		{
			"zip1": 64156,
			"zip2": 64157,
			"dist": 6
		},
		{
			"zip1": 64156,
			"zip2": 64158,
			"dist": 5
		},
		{
			"zip1": 64158,
			"zip2": 64159,
			"dist": 6
		},

		{
			"zip1": 64159,
			"zip2": 64153,
			"dist": 5
		},
		{
			"zip1": 64159,
			"zip2": 64153,
			"dist": 5
		},
		{
			"zip1": 64158,
			"zip2": 64160,
			"dist": 5
		},

		{
			"zip1": 64160,
			"zip2": 64161,
			"dist": 4
		},
		{
			"zip1": 64161,
			"zip2": 64162,
			"dist": 7
		},

		{
			"zip1": 64162,
			"zip2": 64163,
			"dist": 3
		},
		{
			"zip1": 64163,
			"zip2": 64164,
			"dist": 6
		},
		{
			"zip1": 64164,
			"zip2": 64165,
			"dist": 8
		},
		{
			"zip1": 64165,
			"zip2": 64162,
			"dist": 7
		},
		{
			"zip1": 64164,
			"zip2": 64166,
			"dist": 4
		},
		{
			"zip1": 64166,
			"zip2": 64167,
			"dist": 9
		},


		{
			"zip1": 64160,
			"zip2": 64157,
			"dist": 2
		},
		{
			"zip1": 64167,
			"zip2": 64157,
			"dist": 11
		}

	]
    vehicles = [{
			"id": 1,
			"type": 2,
			"zip": 64160, 'free': True
		}]
    requests = [{
			"id": 1,
			"type": 2,
			"zip": 64153,
			"vehicle": None,
			"dist": 0
		}
	]

	
    # adding vertices(zip codes) and edges(dists between the zip codes)
    g = Graph()

    for dist in dists:
        if not dist['zip1'] in g.getVertices():
            g.addVertex(dist['zip1'])
        if not dist['zip2'] in g.getVertices():
            g.addVertex(dist['zip2'])
        g.addEdge(dist['zip1'], dist['zip2'], dist['dist'])
    
    #opening a file to write the output in
    my_file=open("binary.txt","w")

    #process each request one by one
    for request in requests:
        #get the vehicles that are free and of requested type
        freeVehicles = [v for v in vehicles if v['type'] == request['type'] and v['free']]
        
        #getting the vertex of the requested zip code and passing it to Djikstra
        #set all dists in graph to infinity
        if len(freeVehicles) > 0:
            g.resetVertices() 
            vertex = g.getVertex(request['zip']) 
            dijkstra(g, vertex) 
            #assign calculated dists to each availible vehicle
            for av in freeVehicles:
                av['dist'] = g.getVertex(av['zip']).getdist()
            #sort free vehicles by the dist
            freeVehicles.sort(key=lambda element: element['dist'])
        #set free property to false for the first vehicle in the list
        #get the first element of the sorted list and print the completed request
        if len(freeVehicles) > 0: 
            vehicles[vehicles.index(freeVehicles[0])]['free'] = False 
            request['vehicle'] = freeVehicles[0]['id'] 
            request['dist'] = freeVehicles[0]['dist'] 
            [print(element,'   ',request[element]) for element in request] 
            print()
            [my_file.write((element+'   '+str(request[element]) + '   ')) for element in request]
            my_file.write("\n")
    my_file.close()
    end = time.time()
    print(end - start)
