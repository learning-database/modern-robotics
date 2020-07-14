from __future__ import division
import math
import csv_parser
import aStar
import numpy as np

def sampleSpace(diagonalCoordinates, obstacles, resolution):
    """ Samples a rectangular space defined by the given main diagonal of the rectangle.

    :param diagonalCoordinates: The coordinates at the two ends of the main diagonal of the rectangular space
                                    [diagonalStartX, diagonalStartY, diagonalEndX, diagonalEndY]
                                    Ex: [0, 0, 50, 50] ~ The rectangular space bounded by points
                                    (0, 0), (0, 50), (50, 50), (50, 0)
    :param obstacles: List of obstacles. Each element on the list is a list of two elements
                      Example element of obstacles list - [0, 0, 10]
                                       ~ The obstacle at coordinate (0, 0) with a diameter of 10 meters
    :return [nodes]: The sampled coordinates of the given space as a list
                     A single element of the list is a sublist containing the sampleId, x, y positions 
                     of the relavent sample point
    """

    xCoordinateMin = min(diagonalCoordinates[0], diagonalCoordinates[2])
    xCoordinateMax = max(diagonalCoordinates[0], diagonalCoordinates[2]) + 0.001

    yCoordinateMin = min(diagonalCoordinates[1], diagonalCoordinates[3])
    yCoordinateMax = max(diagonalCoordinates[1], diagonalCoordinates[3]) + 0.001

    # NOTE: A slack of 0.001 is added to CoodinateMax values to mitigate effect of 
    # floating point operation errors when using numpy.arrange method

    samples = []
    for x in np.arange(xCoordinateMin, xCoordinateMax, resolution):
        for y in np.arange(yCoordinateMin, yCoordinateMax, resolution):
            # Check if the current x,y coordinate is in collision with an obstacle
            point = [x, y]
            # Checking for collisions with obstacles
            if not collisionPoint(obstacles, point):
                samples.append(point)
    return samples

def collisionPoint(obstacles, point):
    """ Checks if a given point is in collision with a given obstacle set
    :param obstacles: List of obstacles. Each obstacle is a list of obstacle coordinate and diameter
                    Element of obstacles: [obstacleX, obstacleY, diameter]
    :param point: [pointX, pointY]
    :return true if point is in collision. false if not in collision
    """

    for obstacle in obstacles:
        #Distance from point to obstacle center
        distance = eucledianDistance( obstacle[:2], point)
        obstacleRadius = obstacle[2] / 2
        if (distance < obstacleRadius):
            return True
    return False

def createGraph(obstacles, nodes, proximityThreshold):
    """ Given the geometric coordinates to a set of nodes and obstacles, returns the graph
    :param obstacle: Obstacle coordinate and diameter
                    [obstacleX, obstacleY, diameter]
    :param nodes: List of nodes
                    [nodeID, pointX, pointY]
    :param proximityThreshold: The function will only create edges between nodes, at 
                    a distance less than the proximityThreshold (in meters)
    :return edge cost matrix (2D List)
    """
    nodeCount = len(nodes)
    # Generate Edge Cost 2D List
    edgeCosts = [ [ -1.0 for i in range(nodeCount+1) ] for j in range(nodeCount+1) ]  # -1 cost : No edge exists

    for nodeA in nodes:
        for nodeB in nodes:
            nodeA_ID = nodeA[0]
            nodeB_ID = nodeB[0]
            
            # If node IDs are equal, cost is zero
            if nodeA_ID == nodeB_ID:
                edgeCosts[nodeA_ID][nodeB_ID] = 0
                edgeCosts[nodeB_ID][nodeA_ID] = 0
            # Check if an edge already exists
            elif ( (edgeCosts[nodeA_ID][nodeB_ID] != -1) and (edgeCosts[nodeB_ID][nodeA_ID] != -1) ):
                # Edge already exists
                continue
            else:
                # Check node proximity
                nodeAPoint = nodeA[1:]
                nodeBPoint = nodeB[1:]
                distance = eucledianDistance( nodeAPoint, nodeBPoint)
                if( distance <= proximityThreshold):
                    # Check for collision
                    lineSegment = nodeAPoint + nodeBPoint
                    if not collisionLine(obstacles, lineSegment):
                        edgeCosts[nodeA_ID][nodeB_ID] = distance
                        edgeCosts[nodeB_ID][nodeA_ID] = distance
    return edgeCosts

def calculateCostToGo(goal, nodes):
    """ Given the geometric coordinates to a set of nodes and a goal, returns the cost-to-go matrix
    :param goalPosition:  Goal coordinates.
                    [goalX, goalY]
    :param nodes: List of nodes
                    [nodeID, pointX, pointY]
    :return cost to go matrix
    """
    nodeCount = len(nodes)
    # Generate cost-to-go List
    costToGo = [ 0.0 for i in range(nodeCount + 1) ]
    for node in nodes:
        costToGo[node[0]] = eucledianDistance(goal, node[1:])
    return costToGo        

def collisionLine(obstacles, line):
    """ Checks if a given line segment is in collision with a given set of obstacle
    :param obstacles: List of obstacles. Each obstacle is a list of obstacle coordinate and diameter
                    Element of obstacles: [obstacleX, obstacleY, diameter]
    :param line: Coordinates at the two ends of the line segment
                    [Ax, Ay, Bx, By]
    :return true if point is in collision. false if not in collision
    """

    for obstacle in obstacles:
        # Values
        OriginX = obstacle[0]
        OriginY = obstacle[1]
        DIAMETER = obstacle[2]
        Ax = line[0]
        Ay = line[1]
        Bx = line[2]
        By = line[3]

        # parametric equation of the given line is,
        # x = t * Ax + (1-t) * Bx
        # y = t * Ay + (1-t) * By
        # Where t is a parametric value from 0 to 1
        # Plugging this in the equation of the circle, we get,
        # t2 * (d DOT d) + 2t*( f DOT d ) + ( f DOT f - r^2 ) = 0
        # where d is a vector from point A to B
        # and f is a vector from circle origin to point A
        # and r is circle radius

        d = [Bx-Ax, By-Ay]
        f = [Bx-OriginX, By-OriginY]
        r = DIAMETER / 2

        a = d[0] * d[0] + d[1] * d[1] # d.Dot( d )
        b = -2 * ( f[0] * d[0] + f[1] * d[1] ) # 2*f.Dot( d )      
        c = ( f[0] * f[0] + f[1] * f[1] ) - r * r # f.Dot( f ) - r*r 

        discriminant = b*b - 4*a*c
        if discriminant < 0:
            # No intersection between circle and line (infinite line)
            continue
        else:
            # Finding the solution to intersecting point
            discriminant = math.sqrt( discriminant )
            t1 = (-b - discriminant) / (2*a)
            t2 = (-b + discriminant) / (2*a)

            # t is supposed to be within the range [0,1]
            if ( (t1 >= 0 and t1 <= 1) or (t2 >= 0 and t2 <= 1) ):
                # Collision
                return True
            else:
                # Intersection points are out of the bounds of the line segment
                continue
    # No collision detected
    return False

def eucledianDistance (pointA, pointB):
    """ Calculates and returns eucledian distance between two points
    """
    xDistance = pointA[0] - pointB[0]
    yDistance = pointA[1] - pointB[1]
    return math.sqrt( math.pow(xDistance, 2) + math.pow(yDistance, 2) )

def find_path(obstacles, startPosition, goalPosition, robotDiameter, sampleResolution, proximityThreshold, motionSpace = []):
    """ Finds the path from given startPosition to goalPosition, that avoids the given obstacles

    :param obstacles: List of obstacles. Each element on the list is a list of two elements
                      Example element of obstacles list - [0, 0, 10]
                                       ~ The obstacle at coordinate (0, 0) with a diameter of 10 meters
    :param startPosition: Start coordinates. Ex: [0, 0]
    :param goalPosition:  Goal coordinates. Ex: [50, 50]
    :param robotDiameter: Diameter of the robot in meters
    :sampleResolution: Sampling resolution of the motion space, in meters
                      Ex: sampleResolution = 0.01 will sample the space at every 1cm distance
    :proximityThreshold: Maximum distance between sample points, for connecting edges between sample points
    :motionSpace: (Optional) The motion space to plan the path in.
                  - If provided, the path will only be searched in this region
                      Ex: [0, 0, 100, 100] ~ Motion space is within the rectangle where
                      (0, 0) and (100,100) are at the two ends of the main diagonal
                  - If not provided, the path will be calculated within a rectangle, where
                    the start and goal positions are at the two ends of the main diagonal
    :return [status, path]
            Status -  Boolean value Success / Fail status of finding the optimal path
            path - Node IDs in the path, as a list

    Example Input:
        Refer the obstacles.csv file provided with V-REP Scene5_example
        http://hades.mech.northwestern.edu/images/5/5c/V-REP_scenes.zip

    Output:
        TODO 
    """

    if(len(motionSpace) != 4):
        # Custom motion space not given
        # Paths will be only searched within the rectangle bounded by start and goal positions
        motionSpace = startPosition + goalPosition

    #Inflate obstacles by robot radius
    for obstacle in obstacles:
        obstacle[2] = obstacle[2] + (robotDiameter / 2)
    # Sample motionSpace
    samples = sampleSpace(motionSpace, obstacles, sampleResolution)

    nodes = []
    nodeID = 1
    ### Create Nodes list with Node IDs
    # Check whether start and goal positions are in sampled space and remove them
    if startPosition in samples:
        samples.remove(startPosition)
    if goalPosition in samples:
        samples.remove(goalPosition)
    # Add start position to begining of sample list
    samples.insert(0, startPosition)
    # Add goal position to end of sample list
    samples.append(goalPosition)
    
    for sample in samples:
        nodes.append( [nodeID] + sample)
        nodeID = nodeID + 1

    # Calculate cost-to-go
    costToGo = calculateCostToGo(goalPosition, nodes)
    # Save nodes list to csv
    csv_parser.writeNodes(nodes, costToGo)
    # Calculate edge-costs
    edgeCosts = createGraph(obstacles, nodes, proximityThreshold)
    # Save edges list to csv
    csv_parser.writeEdges(nodes, edgeCosts)
    # Find path
    status, path = aStar.searchGraph(nodes, edgeCosts, costToGo)
    return status, path




