import csv
import sys

def readCSV(fileName):
    """Reads a list of comma seperated parameters from a given text/csv file

    :param fileName: Name of the file to be read
    :return [status, parameterList, length]
            Status -  Boolean value Success / Fail status of reading the file (True = Success)
            parameterList - List of paramters 
                        Each item in the list will be, a list with two or more elements [parameter name, value0, value1, ...]
            length - Number of parameters

    Example Input:
        # This is an example input file
        sampling_resolution: 10
        robot_diameter: 0.4
        
    Output:
        [True,[[sampling_resolution, 10],[robot_diameter, 0.4]], 2]
    """


    status = False
    datalist = []

    try:
        with open(fileName) as csvFile:
            for line in csvFile:
                # Ignore commented lines
                line = line.partition('#')[0]
                line = line.rstrip()
                # Ignore whitespaces and split by comma delimeters
                lineItems = line.replace(' ','').split(',')
                # Ignore empty lines
                if(len(lineItems) == 1 and lineItems[0] == ''): continue
            
                try:
                    lineValues = [float(i) for i in lineItems] 
                    datalist.append(lineValues)
                except:
                    print "Error in line : ", line, ". Contains non numeric characters"
            # Successfully read the File
            status = True
        
    except:
        print "File reading error in ", fileName, " : ", sys.exc_info()[0]

    return status, datalist, len(datalist)    


def writePath(path):
    try:
        with open('path.csv', mode='w') as output:
            output.write("# path.csv file for V-REP kilobot motion planning scene.\n")
            output.write("# Path is represented as a sequence of nodes of the graph.\n")

            # A valid path consists of a list of more than one node. 
            if len(path) > 1 :
                output.write(','.join([str(node) for node in path]))
                print "Path calculated successfully. Output written to path.csv"
            # A path with only one node indicates the robot is stuck at starting position (i.e. Failed to find path)        
            else:
                output.write( str(path[0]) )
                output.write("\n# Failed to find a path")
                print "Failed to calculate a path"
    except:
            print "Output file writing error: ", sys.exc_info()[0]

def writeNodes(nodes, costToGo):
    try:
        with open('nodes.csv', mode='w') as output:
            output.write("# nodes.csv file for V-REP kilobot motion planning scene.\n")
            output.write("# Each line below is of the form\n")
            output.write("# ID,x,y,heuristic-cost-to-go\n")
            output.write("# where ID is the unique integer ID number of the node (1 through N),\n")
            output.write("# (x,y) is the location of the node in the plane, and heuristic-cost-to-go\n")
            output.write("# is an optimistic estimate of the path length from that node to the\n")
            output.write("# goal node, as needed by A* search.\n")

            for node in nodes:
                nodeID = node[0]
                nodeCostToGo = costToGo[nodeID]
                nodeData = node + [nodeCostToGo]
                output.write(','.join([str(data) for data in nodeData]))
                output.write('\n')
    except:
            print "Output file writing error: ", sys.exc_info()[0]

def writeEdges(nodes, edgeCosts):
    try:
        with open('edges.csv', mode='w') as output:
            output.write("# edges.csv file for V-REP kilobot motion planning scene.\n")
            output.write("# Each line below is of the form\n")
            output.write("# ID1,ID2,cost,\n")
            output.write("# where ID1 and ID2 are the IDs of the nodes connected by the edge and\n")
            output.write("# cost is the cost of traversing that edge (in either direction).e\n")

            edges = []
            for nodeA in nodes:
                for nodeB in nodes:
                    nodeA_ID = nodeA[0]
                    nodeB_ID = nodeB[0]
                    # Skip for same node
                    if nodeA_ID == nodeB_ID:
                        continue
                    # Skip is no edge exists
                    elif (edgeCosts[nodeA_ID][nodeB_ID] == -1 and edgeCosts[nodeB_ID][nodeA_ID] == -1):
                        continue
                    else:
                        cost = edgeCosts[nodeA_ID][nodeB_ID]
                        # Check if the edge is already included
                        if ([nodeA_ID, nodeB_ID, cost] not in edges) and ([nodeB_ID, nodeA_ID, cost] not in edges):
                            edges.append([nodeA_ID, nodeB_ID, cost])

            for edge in edges:
                output.write(','.join([str(data) for data in edge]))
                output.write('\n')
    except:
            print "Output file writing error: ", sys.exc_info()[0]