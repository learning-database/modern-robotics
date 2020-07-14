def searchGraph(nodes, edgeCosts, costToGo):
    """Returns the optimal path in a graph, using A* algorithm

    :return [status, path]
            Status -  Boolean value Success / Fail status of finding the optimal path
            path - Node IDs in the path, as a list

    Example Input:
        Refer the nodes.csv and edges.csv files provided with V-REP Scene5_example
        http://hades.mech.northwestern.edu/images/5/5c/V-REP_scenes.zip

    Output:
        (True, [1, 3, 4, 7, 10, 12])
    """

    # private constants
    NODE_ID_INDEX = 0
    EST_TOTAL_COST_INDEX = 1

    nodeCount = len(nodes)

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