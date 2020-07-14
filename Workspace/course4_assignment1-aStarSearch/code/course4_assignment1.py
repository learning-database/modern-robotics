import csv
import sys

################# Instructions to Run the code ##################
# Make sure you have python 2 installed with following libraries.
#       CSV Library
#
# Run the python file.
#      Example: In linux, open a terminal and type "python course4_assignment1.py"          
#      
# The code will automatically generate a CSV file named path.csv in the execution directory.

def readCSV(fileName, parameters = -1):
    """Reads a list of comma seperated data (numbers) from a given text/csv file

    :param fileName: Name of the file to be read
    :param parameters: Number of values per one row (Optional parameter)
                       If not provided, will read all the values in a row, by default.
    :return [status, datalist, length]
            Status -  Boolean value Success / Fail status of reading the file (True = Success)
            dataList - List of values. 
                        Each item in the list will be, a list of values in one row in the input file
            length - Number of items in the dataList

    Example Input:
        # This is an example input file
        1,2,3
        4,5,6
        
    Output:
        [True,[[1,2,3],[4,5,6]], 2]
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
                # Check number of parameters per line, and append to output list
                if(parameters == -1 or parameters == len(lineItems)):
                    try:
                        lineValues = [float(i) for i in lineItems] 
                        datalist.append(lineValues)
                    except:
                        print "Error in line : ", line, ". Contains non numeric characters"
            # Successfully read the File
            status = True
        
    except:
        print "File reading error in ", fileName, " : ", sys.exc_info()[0]
        if(parameters != -1):
            print "Required number of parameters: ", parameters

    return status, datalist, len(datalist)    

def findPath():
    """Returns the optimal path in a graph, using A* algorithm

    The files nodes.csv and edges.csv should be present in the root directory.

    :return [status, path]
            Status -  Boolean value Success / Fail status of finding the optimal path
            path - Node IDs in the path, as a list

    Example Input:
        Refer the nodes.csv and edges.csv files provided with V-REP Scene5_example
        http://hades.mech.northwestern.edu/images/5/5c/V-REP_scenes.zip

    Output:
        (True, [1, 3, 4, 7, 10, 12])
    """

    # csv file value indexes
    ID1_INDEX = 0
    ID2_INDEX = 1
    EDGE_COST_INDEX = 2
    COST_TO_GO_INDEX = 3

    # Read Node list and costs-to-go values
    status, nodeData, nodeCount = readCSV('nodes.csv')
    if (not status) : return False, []
    # Read edge costs
    status, edgeData, edgeCount = readCSV('edges.csv', 3)
    if(not status) : return False, []

    # Generate cost-to-go List
    costToGo = [ 0.0 for i in range(nodeCount + 1) ]
    for node in nodeData:
        costToGo[ int(node[ID1_INDEX]) ] = node[COST_TO_GO_INDEX]
    # Generate Edge Cost 2D List
    edgeCosts = [ [ -1.0 for i in range(nodeCount+1) ] for j in range(nodeCount+1) ]  # -1 cost : No edge exists
    for edge in edgeData:
        edgeCosts[ int(edge[ID1_INDEX]) ][ int(edge[ID2_INDEX]) ] = edge[EDGE_COST_INDEX]
        edgeCosts[ int(edge[ID2_INDEX]) ][ int(edge[ID1_INDEX]) ] = edge[EDGE_COST_INDEX]
    # private constants
    NODE_ID_INDEX = 0
    EST_TOTAL_COST_INDEX = 1

    # Initialize past Costs List
    pastCost = [ float('inf') for i in range(nodeCount + 1) ]
    pastCost[1] = 0
    openNodesList = [ [1, costToGo[1] + pastCost[1]] ]  # [ID, estimated_total_cost]
    closedNodesList = []
    parentNodeList = [ 0 for i in range(nodeCount + 1) ]

    while len(openNodesList) > 0 :
        currentNode = int( openNodesList.pop(0)[NODE_ID_INDEX] )
        closedNodesList.append(currentNode)
        # Last node (goal node ID = nodeCount)
        if(currentNode == nodeCount):
            path = []
            while( currentNode >= 1):
                path.insert(0, currentNode)
                currentNode = parentNodeList[currentNode]
            return True, path
        
        for neighbour in range (1, nodeCount + 1):
            if( (neighbour not in closedNodesList) and (edgeCosts[currentNode][neighbour] != -1) ):
                tentative_past_cost = pastCost[currentNode] + edgeCosts[currentNode][neighbour]
                if tentative_past_cost < pastCost[neighbour]:
                    pastCost[neighbour] = tentative_past_cost
                    parentNodeList[neighbour] = currentNode

                    # Update estimated total cost
                    totalCost = pastCost[neighbour] + costToGo[neighbour]
                    openNodesIDList = [ openNodesList[i][NODE_ID_INDEX] for i in range(len(openNodesList)) ]
                    if neighbour not in openNodesIDList:
                        openNodesList.append([ neighbour,  totalCost])
                    else:
                        openNodesList [ openNodesIDList.index(neighbour) ][1] = totalCost
                    # Sort the openNodesList according to Estimated Total Cost Value of nodes
                    openNodesList.sort(key=lambda x: x[EST_TOTAL_COST_INDEX])
    # No path found              
    return False, []

# Find Path (A* Algorithm)
success, path = findPath()
try:
    with open('path.csv', mode='w') as output:
        output.write("# path.csv file for V-REP kilobot motion planning scene.\n")
        output.write("# Path is represented as a sequence of nodes of the graph.\n")
                
        if success:
            output.write(','.join([str(node) for node in path]))
            print "Path calculated successfully. Output written to path.csv"
        else:
            output.write("1\n# Failed to find a path")
            print "Failed to calculate a path"
except:
        print "Output file writing error: ", sys.exc_info()[0]
