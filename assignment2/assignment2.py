import sys
import math
import time

class Edge:
    def __init__(self, startNode, endNode, weight):
        self.start = startNode
        self.end = endNode
        self.weight = weight
    

class Node:
    #Initialize Node
    def __init__(self, name, edges):
        self.name = name
        self.edges = edges
        self.distance = -math.inf
        self.num_longest = 0
    
    #Method allows sort to sort list of Node objects by name quickly with more readable syntax than lambda functions
    def __lt__(self, other):
        return self.name < other.name

def construct_graph():
    graph_nodes = []

    lineNumber = 0
    for line in sys.stdin:
        if lineNumber != 0:
            start, end, weight = line.split(" ")[0], line.split(" ")[1], line.split(" ")[2]
            node = graph_nodes[int(start) - 1]
            node.edges.append(Edge(int(start), int(end), int(weight)))
        else:
            num_nodes = line.split(" ")[0]
            for i in range(0, int(num_nodes)):
                graph_nodes.append(Node(i, []))
            lineNumber += 1
    
    return graph_nodes



def find_longest_path(nodeList):
    nodeList[0].distance = 0
    nodeList[0].num_longest = 1
    largest_dist_index = 1

    for node in nodeList:
        for edge in node.edges:
            current_distance = node.distance
            next_node = nodeList[edge.end - 1]
            
            #If node has been visted 
            if(next_node.distance != -math.inf):
                
                #Check to see if distance is less than current path + edge weight
                if(next_node.distance < current_distance + edge.weight):
                    next_node.distance = current_distance + edge.weight
                #If distance is the same then this is an equally possible best route so add number of longest through node to the value next_node already holds
                elif next_node.distance == current_distance + edge.weight:
                    next_node.num_longest += node.num_longest
            
            #If node hasn't been visited update node distance to be current + edge weight  
            #Also update the current num_long of paths to be same as previous node as this might be the only time that node is used  
            else:
                next_node.distance = current_distance + edge.weight
                next_node.num_longest = node.num_longest
            
            #Continually updates the node that holds the longest so it isn't presumed its the last node in the list
            if(next_node.distance > nodeList[largest_dist_index - 1].distance):
                largest_dist_index = next_node.name

    long_path = nodeList[largest_dist_index].distance
    num_paths = nodeList[largest_dist_index].num_longest
    return [long_path, num_paths]  

def main():

    
    #Construct Graph
    graph = construct_graph()

    #Call to get longest path number followed by the number of paths [longest path, number of paths]
    results = find_longest_path(graph)

    #Outputs the values in results
    print(f"Longest path: {results[0]} \nNumber of longest paths: {results[1]}\n")


if __name__ == "__main__":
    start_time = time.time()
    main()
    print("Completed in: --%s-- seconds." % (time.time() - start_time))