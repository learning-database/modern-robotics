import prm_planner
import csv_parser

# Read obstacle values
status, obstacles, obstacleCount = csv_parser.readCSV('obstacles.csv')
if not status:
    print "Cannot Plan path"

# Read path planning parameters 
# 1. Start position
# 2. Goal position
# 3. Robot Diameter
# 4. Sample Resolution
# 5. Proximity Threshold
# 6. Optional- motionSpace
status, parameterData, parameterCount = csv_parser.readCSV('parameters.csv')
if not status:
    print "Cannot Plan path"
else:
    startPosition = parameterData[0]
    goalPosition = parameterData[1]
    robotDiameter = parameterData[2][0]
    sampleResolution = parameterData[3][0]
    proximityThreshold = parameterData[4][0]

    print "Path Start Position: ", startPosition
    print "Path Goal Position: ", goalPosition
    print "Robot Diameter: ", robotDiameter
    print "Space Sample Resolution: ", sampleResolution
    print "Node proximity Threshold for neigbour search: ", proximityThreshold
    status, path = prm_planner.find_path(obstacles, startPosition, goalPosition, robotDiameter, sampleResolution, proximityThreshold)
    # Write path to csv
    csv_parser.writePath(path)
    

