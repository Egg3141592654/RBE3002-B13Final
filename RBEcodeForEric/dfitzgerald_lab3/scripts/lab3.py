#! /usr/bin/env python

import rospy

# message types we may need
from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Quaternion
# from kobuki_msgs.msg import BumperEvent
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float64
# from nav_msgs.msg import Odometry

# Occupancy grid stuff we may use
from nav_msgs.msg import OccupancyGrid  # http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
from nav_msgs.msg import MapMetaData  # http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html
from nav_msgs.msg import GridCells  # http://docs.ros.org/api/nav_msgs/html/msg/GridCells.html
from map_msgs.msg import OccupancyGridUpdate  # http://docs.ros.org/api/map_msgs/html/msg/OccupancyGridUpdate.html
from geometry_msgs.msg import Point  # http://docs.ros.org/api/geometry_msgs/html/msg/Point.html   

# usefull methods
from tf.transformations import euler_from_quaternion

# start and end goal
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

# paths
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

# usefull python
import math
import numpy
import random
import copy

print ("\n"*50) + "LAB3 PYTHON SCRIPT STARTED..."


################################################ CLASSES ################################################

# Occupancy Grid Parameters
G_GridCols = 1
G_GridRows = 1
G_GridTotalCells = 1
G_GridOriginX = 0
G_GridOriginY = 0
G_GridResolution = 0.0
G_OccupancyGrid = OccupancyGrid()
G_Heuristic=[]
G_PrintDebug=True
G_Pathpoints=[]
G_RobotRadius=0.23

class Cell:
    col = 0
    row = 0
    
    def __init__(self, COL, ROW):
        self.col = int(COL)
        self.row = int(ROW)
        if (COL < 0 or ROW < 0 or COL >= G_GridCols or ROW >= G_GridRows):
            raise Exception("CELL POSITION OUT OF BOUNDS: " + self.toString())

    def toString(self):
        return "[" + str(self.col) + "," + str(self.row) + "]"    

    def toPoint(self):
        x = (self.col + 0.5) * G_GridResolution + G_GridOriginX 
        y = (self.row + 0.5) * G_GridResolution + G_GridOriginY
        return Point(x, y, 0.0)
    
    @staticmethod
    def makeFromPoint(point):
        #use floor or int??
        col = math.floor((point.x - G_GridOriginX) / G_GridResolution)
        row = math.floor((point.y - G_GridOriginY) / G_GridResolution)    
        return Cell(col, row)

    def getIndex(self):
        global G_GridCols
        myIndex= self.row * G_GridCols + self.col
        if Cell.isValidIndex(myIndex):
            return myIndex
        else:
            raise Exception("CELL INDEX OUT OF BOUNDS: "+str(myIndex)+" for C" + self.toString())
    
    @staticmethod
    def getColOfIndex(index):
        global G_GridCols
        return index % G_GridCols 
    
    @staticmethod
    def getRowOfIndex(index):
        global G_GridRows
        return math.floor(index / G_GridCols)    

    @staticmethod
    def makeFromIndex(index):
        col = index % G_GridCols
        row = math.floor(index / G_GridCols)
        return Cell(col, row)  
      
    def __eq__(self, other):
        return ((self.col == other.col) and (self.row == other.row))

    def distanceFrom(self, otherCell):
        xDiff=self.col-otherCell.col
        yDiff=self.row-otherCell.row
        return math.sqrt(math.pow(xDiff,2)+math.pow(yDiff,2))

    def distanceFromGoal(self):
        return self.distanceFrom(G_Goal)

    def euclidianDistanceFromCellWithIndex(self, index):
        xDiff=self.col-Cell.getColOfIndex(index)
        yDiff=self.row-Cell.getRowOfIndex(index)
#         print "\t\t\t xDiff: "+str(xDiff)+ "\tyDiff: "+str(yDiff)
        return math.sqrt(math.pow(xDiff,2)+math.pow(yDiff,2))

    def costDistanceFromCellWithIndex(self, index):
        colDiff=abs(self.col-Cell.getColOfIndex(index))
        rowDiff=abs(self.row-Cell.getRowOfIndex(index))
        maxDiff=max([colDiff,rowDiff])
        if (colDiff != rowDiff):
            return maxDiff
        else:
            return 1.414213*maxDiff

    @staticmethod
    def costDistanceBetweenIndeces(indexA, indexB):
        #TODO: make this faster!!!
        return Cell.makeFromIndex(indexA).costDistanceFromCellWithIndex(indexB)

############# NEIGHBORHOOD #########
    @staticmethod
    def isValidIndex(index):
        return (index>=0 and index<G_GridTotalCells)
    
    @staticmethod
    def getIndecesOfNeighborsOfCellWithIndex(index):
        #get the indeces of the 8-connected neighbors of the cell with this index
        indexAbove=index+G_GridCols
        indexBelow=index-G_GridCols
        indexLeft=index-1
        indexRight=index+1
        indexAboveLeft=indexAbove-1
        indexAboveRight=indexAbove+1
        indexBelowLeft=indexBelow-1
        indexBelowRight=indexBelow+1
        
        neighborIndexes=filter(Cell.isValidIndex,[indexAbove,indexBelow,indexLeft,indexRight,indexAboveLeft,indexAboveRight,indexBelowLeft,indexBelowRight])
        
        return neighborIndexes

    def getIndecesOfNeighbors(self):
        return Cell.getIndecesOfNeighborsOfCellWithIndex(self.getIndex())
        
    def getNeighbors(self):
        return [Cell.makeFromIndex(index) for index in self.getIndecesOfNeighbors()]

#############UNOCCUPIED#########

    #the cell is either unoccipied or unexplored
    @staticmethod
    def isUnoccupiedIndex(index):
        return (G_OccupancyGrid.data[index]<50)

    def isUnoccupied(self):
        return Cell.isUnoccupiedIndex(self.getIndex())

    @staticmethod
    def getIndecesOfUnoccupiedNeighborsOfCellWithIndex(index): 
        return filter(Cell.isUnoccupiedIndex, Cell.getIndecesOfNeighborsOfCellWithIndex(index))

    def getIndecesOfUnoccupiedNeighbors(self):
        return filter(Cell.isUnoccupiedIndex, self.getIndecesOfNeighbors())

    def getUnoccupiedNeighbors(self):
        return [Cell.makeFromIndex(index) for index in self.getIndecesOfUnoccupiedNeighbors()]

############# FREE #########

    #the cell 
    @staticmethod
    def isFreeIndex(index):
        return (0<=G_OccupancyGrid.data[index]<50)

    def isFree(self):
        return isFreeIndex(self.getIndex())
    
    @staticmethod
    def getIndecesOfFreeNeighborsOfCellWithIndex(index): 
        return filter(Cell.isFreeIndex, Cell.getIndecesOfNeighborsOfCellWithIndex(index))

    def getIndecesOfFreeNeighbors(self):
        return filter(Cell.isFreeIndex, self.getIndecesOfNeighbors())

    def getFreeNeighbors(self):
        return [Cell.makeFromIndex(index) for index in self.getIndecesOfFreeNeighbors()]
    
    ############# FREE #########
    
    @staticmethod
    def isOccupiedIndex(index):
        return (G_OccupancyGrid.data[index]>=50)

    def isOccupied(self):
        return isOccupiedIndex(self.getIndex())                   
    
    @staticmethod
    def getIndecesOfOccupiedNeighborsOfCellWithIndex(index): 
        return filter(Cell.isOccupiedIndex, Cell.getIndecesOfNeighborsOfCellWithIndex(index))

    def getIndecesOfOccupiedNeighbors(self):
        return filter(Cell.isOccupiedIndex, self.getIndecesOfNeighbors())

    def getOccupiedNeighbors(self):
        return [Cell.makeFromIndex(index) for index in self.getIndecesOfOccupiedNeighbors()]

    ############# UNEXPLORED #########

    @staticmethod
    def isUnexploredIndex(index):
        return (G_OccupancyGrid.data[index]<0)

    def isUnexplored(self):
        return isUnexploredIndex(self.getIndex())                   
    
    @staticmethod
    def getIndecesOfUnexploredNeighborsOfCellWithIndex(index): 
        return filter(Cell.isUnexploredIndex, Cell.getIndecesOfNeighborsOfCellWithIndex(index))

    def getIndecesOfUnexploredNeighbors(self):
        return filter(Cell.isUnexploredIndex, self.getIndecesOfNeighbors())

    def getUnexploredNeighbors(self):
        return [Cell.makeFromIndex(index) for index in self.getIndecesOfUnexploredNeighbors()]


################################################ GLOBALS ################################################ 

# testing

tabs = ""


G_Start = Cell(0, 0)
G_Goal = Cell(0, 0)


################################################ TESTS ################################################

def testAll():
    print "\n"*1
    print ("#"*10) + "TESTS" + ("#"*10)
    
    testGridToPointConversion()
    testPointToCellConversion()
    
    testCellToIndexConversion()
    testIndexToCellConversion()
    
    testDistanceFromGoal()
    testDistanceFromGoalByIndex()
    
    testDistanceFromGoalByIndexForExtrema()
    
    print "DONE TESTING"
    print "\n"*2

def testGridToPointConversion():
    print "TESTING GRID POINT CONVERSION"
    
    global G_GridRows
    global G_GridCols
    global G_GridResolution
    global G_GridOriginX
    global G_GridOriginY
    
    G_GridRows = 10
    G_GridCols = 10
    G_GridResolution = 0.5
    G_GridOriginX = -2.5
    G_GridoriginY = -2.5
    
    testPoint = Point(1.25, 1.75, 0.0)  # should be in cell [7,8]
    testCell = Cell.makeFromPoint(testPoint)
    testPointConverted = testCell.toPoint()
    
    printt(1,"Original Point: " + pointToString(testPoint))
    printt(1,"Cell from point: " + testCell.toString())
    printt(1, "Point from cell: " + pointToString(testPointConverted))
    
    if pointsHaveSamePosition(testPoint, testPointConverted):  # testPoint.x==testPointConverted.x and testPoint.y==testPointConverted.y
        printt(1,"PASS - Same!")
    else:
        raise Exception("FAIL - When converting Point" + pointToString(testPoint) + "->Cell" + testCell.toString() + "->Point" + pointToString(testPointConverted) + " got different point back!")
 
def testPointToCellConversion():
    print "TESTING POINT GRID CONVERSION"
    
    global G_GridRows
    global G_GridCols
    global G_GridResolution
    global G_GridOriginX
    global G_GridOriginY
    
    G_GridRows = 10
    G_GridCols = 10
    G_GridResolution = 0.5
    G_GridOriginX = -2.5
    G_GridoriginY = -2.5
    
    testCell = Cell(7, 8)
    testPoint = testCell.toPoint()
    testConvertedCell = Cell.makeFromPoint(testPoint)
    
    printt(2,"Original Cell: " + testCell.toString())
    printt(2, "Point from cell: " + pointToString(testPoint))
    printt(2, "Cell from point: " + testConvertedCell.toString())
    
    if (testCell == testConvertedCell):  # testPoint.x==testConvertedPoint.x and testPoint.y==testConvertedPoint.y
        printt(1,"PASS - Same!")
    else:
        raise Exception("FAIL - When converting Cell->Point->Cell got different cell back!")
    
def testCellToIndexConversion():
     print "TESTING CELL INDEX CONVERSION"
     
     global G_GridRows
     global G_GridCols
     global G_GridTotalCells
     
     G_GridRows = 20
     G_GridCols = 10
     G_GridTotalCells=G_GridRows*G_GridCols
     
     testCell = Cell(7, 8)
     index = testCell.getIndex()
     testCellConverted = Cell.makeFromIndex(index)
     
     printt(2,"Index of Cell " + testCell.toString() + " which is index " + str(index) + " which makes cell " + testCellConverted.toString())
     
     if (index == 87 and testCell == testCellConverted):
         printt(1,"PASSED - index correct and resulted in the original cell")
     else:
         raise Exception("FAIL - index from cell is incorrect or did not convert to the original cell")
     
def testIndexToCellConversion():     
     print "TESTING INDEX CELL CONVERSION"
     
     global G_GridRows
     global G_GridCols
     global G_GridTotalCells
     
     G_GridRows = 20
     G_GridCols = 10
     G_GridTotalCells=G_GridRows*G_GridCols
     
     index = 87
     testCell = Cell.makeFromIndex(index)
     indexConverted = testCell.getIndex()
     
     printt(2,"Cell with index " + str(index) + " is " + testCell.toString() + " which gets index " + str(indexConverted))
     
     if (testCell == Cell(7, 8) and index == indexConverted):
         printt(1,"PASSED - cell correct and resulted in the original index")
     else:
         raise Exception("FAIL - cell from index is incorrect or did not convert to the original index")
  
def testDistanceFromGoal():
    print "TESTING CELL DISTANCE FROM GOAL"
    global G_Goal
    G_Goal=Cell(1,5)
    otherCell=Cell(4,9)
    distFromGoal=otherCell.distanceFromGoal()
    if (distFromGoal == 5.0):
       printt(1,"PASSED - cell has correct distance from goal: "+str(distFromGoal))
    else:
       raise Exception("FAIL - cell has incorrect distance from goal: "+str(distFromGoal))
 
def testDistanceFromGoalByIndex():
    print "TESTING CELL DISTANCE FROM GOAL BY INDEX"
    global G_Goal
    global G_GridCols
    global G_GridRows
    global G_GridTotalCells
    
    G_GridCols=480
    G_GridRows=512
    G_GridTotalCells=G_GridRows*G_GridCols
    
    goaltestCol=1
    goaltestRow=2
    G_Goal=Cell(goaltestCol,goaltestRow)
    testCol=1
    testRow=2
    otherCell=Cell(testCol,testRow)
    indexOfOtherCell=otherCell.getIndex()
    printt(2,"Index of other cell: "+str(indexOfOtherCell))
    printt(2,"Other Cell Reconstructed By Index: "+Cell.makeFromIndex(indexOfOtherCell).toString())
    actualDistFromGoal=G_Goal.euclidianDistanceFromCellWithIndex(indexOfOtherCell)
    expectedDistance=math.sqrt(math.pow(testCol-goaltestCol,2)+math.pow(testRow-goaltestRow,2))
    if (actualDistFromGoal == expectedDistance):
       printt(1,"PASSED - cell has correct distance from goal by index: (Expected="+str(expectedDistance)+" Actual: "+str(actualDistFromGoal)+")")
    else:
       raise Exception("FAIL - cell has incorrect distance from goal by index: (Expected="+str(expectedDistance)+" Actual: "+str(actualDistFromGoal)+")")

def testDistanceFromGoalByIndexForExtrema():
    print "TESTING CELL DISTANCE FROM GOAL BY EXTREMA"
    global G_Goal
    global G_GridCols
    global G_GridRows
    global G_GridTotalCells
    
    G_GridCols=480
    G_GridRows=512
    G_GridTotalCells=G_GridRows*G_GridCols
    
    goaltestCol=1
    goaltestRow=2
    G_Goal=Cell(goaltestCol,goaltestRow)
    testCol=1
    testRow=2
    otherCell=Cell(testCol,testRow)
    distFromGoal=G_Goal.euclidianDistanceFromCellWithIndex(otherCell.getIndex())
    expectedDistance=math.sqrt(math.pow(testCol-goaltestCol,2)+math.pow(testRow-goaltestRow,2))
    if (distFromGoal == expectedDistance):
       printt(2,"PASSED - cell has correct distance from goal by extrema: (Expected="+str(expectedDistance)+" Actual: "+str(distFromGoal)+")")
    else:
       raise Exception("FAIL - cell has incorrect distance from goal by extrama: (Expected="+str(expectedDistance)+" Actual: "+str(distFromGoal)+")")

################################################ HELPERS ################################################

def pointToString(point):
    return "(" + str(point.x) + "," + str(point.y) + "," + str(point.z) + ")"

def printt(string):
    if G_PrintDebug:
        print tabs + string
     
def printt(tabs,string):
    if G_PrintDebug:
        print ("\t"*tabs + string)
       
def pointsHaveSamePosition(pointA, pointB):
    return (pointA.x == pointB.x and pointA.y == pointB.y)
     
################################################ PUBLISHERS ################################################

def getHeuristic(cell):
    return G_Heuristic[cell.getIndex()]

def dictToCellList(dict):
    cellList=cellList=[Cell.makeFromIndex(indexKey) for indexKey in dict.keys()]
    return cellList

################################################ Publisers ################################################

def publishPointListAsGridCells(points, publisher):
    gridCells = GridCells()  # make an empty grid cells
    gridCells.cell_height = G_GridResolution
    gridCells.cell_width = G_GridResolution
    gridCells.header.frame_id = "map"
    gridCells.cells = points
    publisher.publish(gridCells)

def publishListAsOccupancyGrid(list, publisher):
    # TODO: does this need to be a deep copy?
    occupancyGrid = OccupancyGrid()
    occupancyGrid.info.height = G_GridRows
    occupancyGrid.info.width = G_GridCols
    occupancyGrid.info.resolution = G_GridResolution
    occupancyGrid.info.origin.position.x = G_GridOriginX
    occupancyGrid.info.origin.position.y = G_GridOriginY
    occupancyGrid.info.origin.orientation.w = 1
    occupancyGrid.header.frame_id = "map"
    occupancyGrid.data = tuple(list)
    publisher.publish(occupancyGrid)
     
def publishCellListAsGridCells(cells, publisher):
    pointList=[cell.toPoint() for cell in cells]
    publishPointListAsGridCells(pointList, publisher)     
     
def publishStartAndGoalAsOccupancyGrid():
    print "SHOWING START AND GOAL ON HEURISTIC"
    #display the start and goal as black dots on an occupancy grid
    startGoalList = [0] * G_GridTotalCells
    goalIndex = G_Goal.getIndex()
    startGoalList[goalIndex] = 100
    startIndex = G_Start.getIndex()
    startGoalList[startIndex] = 100
    print "\tStart index: " + str(startIndex) + " = " + str(startGoalList[startIndex]) + " Goal index: " + str(goalIndex) + "= " + str(startGoalList[goalIndex])
    publishListAsOccupancyGrid(startGoalList, startGoalDebugOccupancyGridPublisher)     
     
################################################ CALLBACKS ################################################

def publishOccupancyGridAsGridCell():
    #display as GridCells
    print "\t\tPublishing Inflated Occupancy grid as GridCells"
    occupancyGridAsPointList=[]
    for index in range(G_GridTotalCells):
        if (G_OccupancyGrid.data[index]>50):
            occupancyGridAsPointList.append(Cell.makeFromIndex(index).toPoint())
    publishPointListAsGridCells(occupancyGridAsPointList, inflatedOccupancyGridPublisher)

################################################ CALLBACKS ################################################

def publishPointListAsPath(pathPoints, publisher):
#     print "Publishing Path..."
    path=Path()
    for point in pathPoints:
        poseStamped=PoseStamped()
        poseStamped.pose.position=point
        path.poses.append(poseStamped)
    path.header.frame_id="map"
    publisher.publish(path)
#     print "...Done Publishing Path"

def publishFrontier(dict):
    publishCellListAsGridCells(dictToCellList(dict), frontierPublisher)

def publishExplored(dict):
    publishCellListAsGridCells(dictToCellList(dict), exploredPublisher)

def publishExplorationFrontier(cellIndeces):
    pointList=[Cell.makeFromIndex(index).toPoint() for index in cellIndeces]
    publishPointListAsGridCells(pointList, explorationFrontierPublisher)
################################################ CALLBACKS ################################################

def initialPoseCallback(PoseWithCovarianceStamped):
#     print "\n\nINITIAL POSE CALLBACK"
#     print PoseWithCovarianceStamped
#     print "\tNew Start: " + pointToString(PoseWithCovarianceStamped.pose.pose.position)

    global G_Start
    testStart=Cell.makeFromPoint(PoseWithCovarianceStamped.pose.pose.position)
    if (testStart.isUnoccupied()):
        G_Start = testStart
#         print "\tSETTING START: " + G_Start.toString()
    
        publishCellListAsGridCells([G_Start], startPublisher)
#         publishStartAndGoalAsOccupancyGrid()
    else:
        print "\tINVALID START: CELL IS OCCUPIED!"

def simpleGoalCallback(poseStampedGoal):
    print "\n\nSIMPLE GOAL CALLBACK"
#     print poseStampedGoal
    print "New Goal: " + pointToString(poseStampedGoal.pose.position)
    testGoal=Cell.makeFromPoint(poseStampedGoal.pose.position)
    setGoal(testGoal)

def clickedPointCallback(PointStamped):
    print "\n\nPOINT CLICK CALLBACK"
#     print poseStampedGoal
    print "Clicked Goal: " + pointToString(PointStamped.point)
    testGoal=Cell.makeFromPoint(PointStamped.point)
    setGoal(testGoal)

def setGoal(newGoal):
    global G_Goal
    if (newGoal.isUnoccupied()):
        G_Goal = newGoal
        print "\tSetting goal: " + G_Goal.toString()
        
        publishCellListAsGridCells([G_Goal], goalPublisher)
#         publishStartAndGoalAsOccupancyGrid()  
        
        calculateHeuristic() 
                       
        navigateToGoal()
    else:
        print "\tINVALID GOAL: CELL IS OCCUPIED!"

def occupancyGridCallback(occupancyGrid):
    print "\n\nOCCUPANCY GRID CALLBACK"
#     print occupancyGrid.info
  
    global G_GridCols
    global G_GridRows
    global G_GridTotalCells
    global G_GridResolution
    global G_GridOriginX
    global G_GridOriginY
    global G_OccupancyGrid
    
    G_GridCols = occupancyGrid.info.width
    G_GridRows = occupancyGrid.info.height
    G_GridTotalCells = G_GridCols * G_GridRows
    G_GridResolution = occupancyGrid.info.resolution
    G_GridOriginX = occupancyGrid.info.origin.position.x
    G_GridOriginY = occupancyGrid.info.origin.position.y
    G_OccupancyGrid = occupancyGrid
    print "Grid size: (" + str(G_GridCols) + "," + str(G_GridRows) + "), resolution= " + str(G_GridResolution) + ", origin= (" + str(G_GridOriginX) + "," + str(G_GridOriginY) + ")"
    
    inflateOccupancyGrid()
    
    #    WOOOOOOT!
    replan()

    
################################################ AlGORITHM ################################################  

def replan():
    print "\nREPLANNING..."
    
    
#     publishListAsOccupancyGrid(gridList,occupancyGridPublisher)


    print "SETTING GOAL AS CENTROID OF LARGEST FRONTIER"
    frontiersCentroids=findExplorationFrontiers()
    centroidOfLargestFrontier=frontiersCentroids[0]
    #navigate to index of largest frontier
    setGoal(centroidOfLargestFrontier)
    publishCellListAsGridCells([G_Goal], goalPublisher)  

    calculateHeuristic()
    navigateToGoal()

def findExplorationFrontiers():

    #find all the indeces of any cells on the frontier, in no particular order
    def getFrontierIndexes():
        frontierIndexes=[]
        for testIndex in range(G_GridTotalCells):   #loop through grid
#             print "\tTesting cell "+str(testIndex)
            if Cell.isFreeIndex(testIndex):   #if this cell is free (explored and unoccupied)
#                 print "\t\tUnexplored..."
                FreeNeighborIndeces=Cell.getIndecesOfUnexploredNeighborsOfCellWithIndex(testIndex)    #get the list of neighbors that are unexplored
#                 print "\t\tHas "+str(len(FreeNeighborIndeces))+"..."
                if (FreeNeighborIndeces):   #if there's atleast one free neighbor
#                     print "\t\t\tAdding to frontier..."
                    frontierIndexes.append(testIndex)   #then this unexplored cell is contacting a free cell - it's a frontier
        return frontierIndexes
       
    print "FINDING UNEXPLORED FRONTIERS..."
    frontierIndexes=getFrontierIndexes()
    originalfrontierIndexes=copy.deepcopy(frontierIndexes)
    print "\tFound "+str(len(frontierIndexes))+" UNEXPLORED FRONTIERS"
    
    print "\tPUBLISHING FRONTIERS..."
    publishExplorationFrontier(frontierIndexes)

    print "\tMAKING HASH TABLE OF FRONTIER INDEXES"
    frontierIndexAssignments=dict.fromkeys(frontierIndexes)   
        
    def unclaimedIndexes():
        return filter(lambda frontierIndex:(frontierIndexAssignments[frontierIndex] is None),frontierIndexes)
        #return [frontierIndex for frontierIndex in frontierIndexes if frontierIndexAssignments[frontierIndex] is  None]

#     def claimFrontierIndexFor(frontierIndex, frontierNumber):
#         frontierIndexAssignments[frontierIndex]=frontierNumber

    def indexClaimedByFrontierNumber(testIndex,frontierNumber):
        return (frontierIndexAssignments[testIndex] == frontierNumber)

    def getCellsClaimedByFrontierNumber(frontierNumber):
        return filter(lambda frontierIndex:indexClaimedByFrontierNumber(frontierIndex,frontierNumber),frontierIndexes)
        #return [frontierIndex for frontierIndex in frontierIndexes if frontierIndexAssignments[frontierIndex] is  frontierNumber]
        
    
#     def areContiguous(indexA, indexB):
#         cellA=Cell.makeFromIndex(indexA)
#         cellB=Cell.makeFromIndex(indexB)
#         return ((abs(cellA.row-cellB.row)<=1) and (abs(cellA.col-cellB.col)<=1))
#     
#     def isTouching(testIndex, indexes):
# #         print "\t\t\t\t\tChecking if "+str(testIndex) + " touches frontier..."
#         for index in indexes:
#             if areContiguous(testIndex,index):
#                 return True
#         return False
#           
#     def tryToAddThingsTo(frontierNumber):
#         print "\n\t\t\tTrying to add things to frontier number "+str(frontierNumber)+"..."
#         somethingAdded=False
#         indexesInThisFrontier=getCellsClaimedByFrontierNumber(frontierNumber)
#         curUnclaimedIndexes=unclaimedIndexes()
#         
#         expandedFrontier=expand()
#         
# #         for testIndex in curUnclaimedIndexes:
# #             if isTouching(testIndex,indexesInThisFrontier):
# # #                 print "\t\t\t\tadding "+str(testIndex)+" to this frontier"
# #                 claimFrontierIndexFor(testIndex,frontierNumber)
# # #                 indexesInThisFrontier.append(testIndex)
# #                 indexesInThisFrontier= getCellsClaimedByFrontierNumber(frontierNumber)
# #                 somethingAdded=True
#                   
# #         print "\t\t\tFrontierIndexes has "+str(len(curUnclaimedIndexes)) +" indeces remaining"    
#             
# #         print "\t\t\tFrontier now has "+str(len(frontier)) +" indexes (added="+str(somethingAdded)+")"
#         publishExplorationFrontier(curUnclaimedIndexes)
#         return somethingAdded 
# 
#     def getContiguousFrontierWith(seedIndex, frontierNumber):
#         print "\t\tSeeding Frontier with cell "+Cell.makeFromIndex(seedIndex).toString()
# 
#         claimFrontierIndexFor(seedIndex,frontierNumber)#claim the seed index
#         
#         #keep trying to add things that are contiguous with whatever it has so far until nothing gets added
#         somethingAdded=tryToAddThingsTo(frontierNumber)    #flag to see if atleast one thing was added
#         while(somethingAdded):
#             somethingAdded=tryToAddThingsTo(frontierNumber)
#         
#         #publish again
#         print "\t\tPublishing new unclaimed frontier..."
#         finalFrontier=getCellsClaimedByFrontierNumber(frontierNumber)    
#         print "\t\tmade frontier with "+str(len(finalFrontier))+" cells\n"    
    
    def common_elements(list1, list2):
        return list(set(list1).intersection(list2))
#         return [element for element in list1 if element in list2]    
        
    def expandFrontier(seedIndex, curFrontierNumber, recursionDepth):
        def printr(string):
            print "\t"*recursionDepth+string
            
#         printr("Expanding Frontier with seed: "+str(seedIndex)+" depth: "+str(recursionDepth))
#         if (indexClaimedByFrontierNumber(seedIndex,curFrontierNumber)):
        if (frontierIndexAssignments[seedIndex] is None):                                             #if the seed was not claimed by this frontier already
#             printr("\t this index already claimed - nevermind!")
#         else: 
            frontierIndexAssignments[seedIndex]=curFrontierNumber                                      #claim the seed for the frontier   
            freeNeighborIndeces=Cell.getIndecesOfFreeNeighborsOfCellWithIndex(seedIndex)    #get the free neighbors of this seed
#             printr("\tFound "+str(len(freeNeighborIndeces))+" unexplored neighbors")              
            frontierIndecesTouchingSeedIndex=common_elements(freeNeighborIndeces,frontierIndexes) #find those neighbors that are also frontierIndeces
#             printr("\tOf which "+str(len(frontierIndecesTouchingSeedIndex))+" are also neighbors")      
            for addedIndex in frontierIndecesTouchingSeedIndex:                                         #for each neighbor
                expandFrontier(addedIndex,curFrontierNumber, recursionDepth+1)                          #continue expansion from that neighbor
#             printr("\texpanded Frontier from seed "+str(seedIndex)+"\n")  
          
    def getContiguousFrontiersFromFrontierCells():
        curFrontierNumber=0 #initialize index of first frontier
        curUnclaimedFrontierIndexes=unclaimedIndexes()  #get a list of all the unclaimed frontier indexes
        while(curUnclaimedFrontierIndexes):      #while there's still any "unclaimed" frontier indeces on our list (unclaimed cells belong to no cluster)
            "\tStill "+str(len(curUnclaimedFrontierIndexes))+" unclaimed frontier indexes"
            seedIndex=curUnclaimedFrontierIndexes[0]    #get the first index of the unclaimedIndexes
#             getContiguousFrontierWith(seedIndex,curFrontierNumber)  #seed a new frontier with it
            expandFrontier(seedIndex,curFrontierNumber,0)
            curFrontierNumber=curFrontierNumber+1                   #increment frontier number for next frontier
            curUnclaimedFrontierIndexes=unclaimedIndexes()          #get the new list of unclaimed indexes

        print "\tReconstructing Frontiers..."
        frontiers=[getCellsClaimedByFrontierNumber(frontierNumber) for frontierNumber in range(curFrontierNumber)]
        return frontiers
    
    print "\tFINDING CONTINUOUS FRONTIERS..."
    frontiers=getContiguousFrontiersFromFrontierCells()
    print "\t\tFound "+str(len(frontiers))+" Distinct Frontiers"
    
    def getCentroid(indexes):
        colSum=sum([Cell.getColOfIndex(index) for index in indexes])
        rowSum=sum([Cell.getRowOfIndex(index) for index in indexes])
        
        totIndexes=len(indexes)
        colAvg=int(colSum/totIndexes+0.5)
        rowAvg=int(rowSum/totIndexes+0.5)
        return Cell(colAvg, rowAvg)
      
    print "\tFINDING CENTROIDS"
    centroids=[getCentroid(frontier) for frontier in frontiers]
        
    print "\t\tFound "+str(len(centroids))+" Centroids: "
#     for centroid in centroids:
#         print "\tC "+centroid.toString()
    
    print "\tPUBLISHING CENTROIDS"
    publishCellListAsGridCells(centroids,explorationFrontierCentroidsPublisher)
    
    print "\tPUBLISHING ORIGINAL FRONTIERS AGAIN..."
    publishExplorationFrontier(originalfrontierIndexes)

    print "\tSORTING CENTROIDS BY FRONTIER SIZES..."
    frontierSizes=[len(frontier) for frontier in frontiers]
    frontiersSizesAndCentroids=zip(frontierSizes,centroids) 
    sortedCentroids=[SizesAndCentroid[1] for SizesAndCentroid in sorted(frontiersSizesAndCentroids, key=lambda SizesAndCentroid: SizesAndCentroid[0])]
    sortedCentroids.reverse()
    #return a list of the centroids of the frontiers sorted largest to smallest
    return sortedCentroids

def inflateOccupancyGrid():
    print "\tINFLATING OCCUPANCY GRID"
    global G_OccupancyGrid
    
    def inflateByIndex(index):
        if (Cell.isFreeIndex(index)):
            hasAtLeastOneOccupiedNeighbor=Cell.getIndecesOfOccupiedNeighborsOfCellWithIndex(index)
            
#         cell=Cell.makeFromIndex(index)
#         #NOTE: change isFree() to isUnoccupied() to inflate unexplored areas as well - but this takes way longer!
#         if (cell.isFree()):           #cell is still unoccupied and unexplored in this iteration
#             neighbors=cell.getNeighbors()
# #             totOccupiedNeighbors=0
#             hasAtLeastOneOccupiedNeighbor=False
#             for neighbor in neighbors:
#                 if neighbor.isOccupied(): #if the neighbor is occupied
# #                     totOccupiedNeighbors=totOccupiedNeighbors+1
#                     hasAtLeastOneOccupiedNeighbor=True
#                     break
                
            if (hasAtLeastOneOccupiedNeighbor):
#             if (totOccupiedNeighbors>1):    #but has atleast one occupied neighbor
                return 70                   #inflate
            else:
                return 0                    #probably not occupied and no occupied neighbors - definitly unoccupied
        else:                               #cell is currently occupied
            return G_OccupancyGrid.data[index]#keep it as it was if its occupied or unexplored
        
        #not very efficiant
#         neighbors=cell.getNeighbors()
#         totOccupiedNeighbors=0
#         for neighbor in neighbors:
#             if not neighbor.isUnoccupied():
#                 totOccupiedNeighbors=totOccupiedNeighbors+1
#         
#         if (cell.isUnoccupied()):           #cell is unoccupied in this iteration
#             if (totOccupiedNeighbors>1):    #but has atleast one occupied neighbor
#                 return 75                   #inflate
#             else:
#                 return 0                    #not occupied and no occupied neighbors - definitly unoccupied
#         else:                               #cell is currently occupied
#             if (totOccupiedNeighbors<1):    #but has no occupied neighbors
#                 return 40                   #may be a fluke. assume it's harmless
#             else:
#                 return G_OccupancyGrid.data[index]#occupied and atleast one occupied neighbor - leave as it was
                
    
    totIterations=int(G_RobotRadius/G_GridResolution+0.5)
    occupancyGridPublisher.publish(G_OccupancyGrid)
    print "\t\tInflating with "+str(totIterations)+" iterations."
    for iteration in range(totIterations):
        print "\t\t\tInflating iteration "+str(iteration)+"..."
        inflatedGrid=[inflateByIndex(index) for index in range(G_GridTotalCells)]
        G_OccupancyGrid.data=tuple(inflatedGrid)
        occupancyGridPublisher.publish(G_OccupancyGrid)

    
#     publishOccupancyGridAsGridCell()
    
    print "\tDONE INFLATING"

def calculateHeuristic():
    print "CALCULATING HEURISTIC..."
    global G_Heuristic
    G_Heuristic=[]
    heuristicMultiplier=1.0     #increasing this past 1 make the search find a solution quicker, but it may not be optimal. Decreasing below one also helps.
    for index in range(G_GridTotalCells):
        G_Heuristic.append(heuristicMultiplier*G_Goal.euclidianDistanceFromCellWithIndex(index))
       
#     print G_Heuristic
    
    normalizer = 150/max(G_Heuristic)
    def constrainedClip(value):
        val = normalizer*value
        if (val>99.5):
            val=100
        if (val<=1 and val>0.5):
            val=1
        return val
       
    #display the heuristic    

    publishedHeuristic=[constrainedClip(cell) for cell in G_Heuristic] 
    publishedHeuristic[G_Goal.getIndex()]=-1
    publishedHeuristic[G_Start.getIndex()]=-1
    publishListAsOccupancyGrid(publishedHeuristic,heuristicPublisher)
    print "...Done calulating heuristic"


################################################ MAIN ################################################

def segmentPathPoints(pathPoints, threshold, recurseDepth):
    def printr(string):
        if (G_PrintDebug):
            print "\t"*recurseDepth + string
    
    def xyPointToString(point):
        return "("+str(point.x)+","+str(point.y)+")"
    
    printr("SEGMENTING PATH (depth="+str(recurseDepth)+")...")

    totpoints=len(pathPoints)
    
    printr("\tReceived "+str(totpoints)+" points:")
    for point in pathPoints:
        printr("\t\tP"+xyPointToString(point))
    
    #     for point in pathPoints:
#         printr("\t"+xyPointToString(point))
    if (totpoints<=2):
        printr("\tToo Few Points in list - returning last point")
        return [pathPoints[len(pathPoints)-1]]
    else:
        printr("\tPublishing intermediate segment")
#         publishPointListAsPath(pathPoints, pathPublisher)
        
        #check if they all have the same x coordinate
        xCoord=pathPoints[0].x
        pointsHaveSameXCoord=True
        for point in pathPoints:
            if (point.x!=xCoord):
                pointsHaveSameXCoord=False
                break
        if (pointsHaveSameXCoord):
            printr("\tAll Points have same X Coordinate - returning last point")
            return [pathPoints[-1]]
        
        #atleast one point has a different x coordinate
        else:
            
            #get the line of best fit
            line=numpy.polyfit([point.x for point in pathPoints],[point.y for point in pathPoints],1)
            fitlineM=line[0]
            fitLineB=line[1]
            printr("\tBest fit line is: "+str(fitlineM)+"x+"+str(fitLineB))
            
            #note that this does not actually return the true distance (it is not normalized), but the value returned is proportional to the distance, so it can be used for maxDist
            def pointDistanceFromLine(point, m, b):   #line of the form y=mx+b
                return abs(point.y-m*point.x-b)
            
            #if the farthest point from the line of best fit is still within the threshold distance, then the points are roughly linear, so return the line (or in our case, the last point)
            maxDifference=max([pointDistanceFromLine(point,fitlineM,fitLineB) for point in pathPoints])
            if (maxDifference<=threshold):
                lastPoint=pathPoints[-1]
                printr("\tmax point dist ("+str(maxDifference)+") is within thresh ("+str(threshold)+") - returning list of last point: "+xyPointToString(lastPoint))
                return [lastPoint]       #return a list with the last point on the given list
            else:
                
                printr("\tmax point dist ("+str(maxDifference)+") is above thresh ("+str(threshold)+") - segmenting...")
                
                #calculate the slope and y intercept of the line connecting the first and last point
                xDiff=(pathPoints[-1].x-pathPoints[0].x)
                differences=[]
                if (-G_GridResolution<xDiff<G_GridResolution):  #special case: vertical connecting line
                    printr("\tWARNING:  Connecting line is vertial...")
                    lineX=pathPoints[0].x
                    differences=[abs(point.x-lineX) for point in pathPoints]
                else:
                    yDiff=(pathPoints[-1].y-pathPoints[0].y)
                    connectingLineM=yDiff/xDiff
                    connectingLineB=pathPoints[0].y-connectingLineM*pathPoints[0].x
                    printr("\tConnecting Line from start "+xyPointToString(pathPoints[0])+" to end "+xyPointToString(pathPoints[-1])+" is: Y="+str(connectingLineM)+"x+"+str(connectingLineB))
    
                    #make a list if the differences of the points from the line connecting the first and last point
                    differences =[pointDistanceFromLine(point,connectingLineM,connectingLineB) for point in pathPoints]
    
                #find the index and value of the maximum difference of the list
                maxDifference=differences[0]
                maxDifferenceIndex=0
                for index in range(len(differences)):
                    if(differences[index]>maxDifference):
                        maxDifference=differences[index]
                        maxDifferenceIndex=index        
                printr("\tPoint ["+str(maxDifferenceIndex)+"] "+xyPointToString(pathPoints[maxDifferenceIndex]) + " has maximum difference of "+str(maxDifference))
    
                if (maxDifferenceIndex==0 or maxDifferenceIndex==len(pathPoints)-1):
                    raise Exception("Farthest Point from Connecting Line of "+ str(len(pathPoints)) +" was start or end: ["+str(maxDifferenceIndex)+"]")
                
                #split the list into two halvs, do this algorithm on both halves
                printr("\tsegmenting along point...")
                
                firstHalf=pathPoints[:maxDifferenceIndex+1] #every point up to and including the segmenting point
                secondHalf=pathPoints[maxDifferenceIndex:]  #the segmenting point and everything after it
                
                firstHalfSegments=segmentPathPoints(firstHalf,threshold,recurseDepth+1)
                secondHalfSegments=segmentPathPoints(secondHalf,threshold,recurseDepth+1)

                printr("\tReceived line segments:")

                printr("\t\tlines from first Segment: ")
                for point in firstHalfSegments:
                    printr("\t\tP"+xyPointToString(point))
                    
                printr("\t\tlines from second Segment: ")
                for point in secondHalfSegments:
                    printr("\t\t\tP"+xyPointToString(point)) 
                    
                return firstHalfSegments+secondHalfSegments     #recursion!

class Node():
    gScore=0.0
    fScore=0.0
    parentIndex=None
    
    def __init__(self, GSCORE, index, PARENTINDEX):
        self.gScore=GSCORE
        self.fScore=GSCORE+G_Heuristic[index]
        self.parentIndex=PARENTINDEX
        
    @staticmethod    
    def makeNodeWithFScore(GSCORE, FSCORE, PARENTINDEX):    
        newNode=Node(GSCORE, 0, PARENTINDEX)
        newNode.fScore=FSCORE
        return newNode
        
        
    def toString(self):
        return "{G"+str(self.gScore)+"\tF"+str(self.fScore)+"\tP"+str(self.parentIndex)+"}"

def navigateToGoal():
    print "NAVIGATING TO GOAL..."    
    global G_Pathpoints
    global G_PrintDebug
    
    print "GETTING PATH..."
    pathCells=calculateAStar()
    print "Publishing Cell Path"
    publishCellListAsGridCells(pathCells, pathCellsPublisher)

    pathPoints=[cell.toPoint() for cell in pathCells]
    threshold=G_GridResolution*0.9#0.75
    print "SEGMENTING PATH..."
    G_PrintDebug=False
    G_Pathpoints=[pathPoints[0]]+segmentPathPoints(pathPoints, threshold,1)#+[pathPoints[-1]]   #concatinate with the first point because it will be lost otherwise
    G_PrintDebug=True
    publishPointListAsPath(G_Pathpoints, pathPublisher)
    
    print "\n\nNAVIGATING..."

def calculateAStar():
    global G_PrintDebug
    
    print "CALCULATING A STAR from "+G_Start.toString()+" to "+G_Goal.toString()+"..."
      
    startIndex=  G_Start.getIndex()
    print "\tStart index: "+str(startIndex)
    startNode=Node.makeNodeWithFScore(0, G_Heuristic[startIndex], None)
    #goalNode=Node(0,G_Goal.getIndex()) 
    
    closedset = {}
    openset = {G_Start.getIndex():startNode}
    parents = {}
    
    print "\tStart node is: "+startNode.toString()
    
    def getLowestFScore(currentIndex):
        lowest = 1000000
        lowestIndexes=[]
        
        #make a list of the indexes with the same lowest fscore
        for index in openset:
            curLow = openset[index].fScore
            if curLow < lowest:
                lowest = curLow
                lowestIndexes=[index]
            elif(curLow == lowest):
                lowestIndexes=[index]
    
        
        if (len(lowestIndexes)>1):
        
            print "Got "+str(len(lowestIndexes)) +" competing min neighbors:"
            for index in lowestIndexes:
                print "\t"+str(index)
        
            #TODO: Make this index based!    
        
            currentCell=Cell.makeFromIndex(currentIndex)
            IndexOfParentOfCurrentIndex=openset[currentIndex].parentIndex
            
            colDir=0
            rowDir=0
            if (IndexOfParentOfCurrentIndex is not None):
                parentCell=Cell.makeFromIndex(IndexOfParentOfCurrentIndex)
                
                colDir=currentCell.col-parentCell.col
                rowDir=currentCell.row-parentCell.row
                
            else:   #if the parent was none (current is probably the start node), then use the heuristic direction
                colDir= G_Goal.col-currentCell.col
                if colDir>0:
                    colDir=1
                elif(colDir<0):
                    colDir=-1
                
                rowDir=G_Goal.row-currentCell.row
                if rowDir>0:
                    rowDir=1
                elif(rowDir<0):
                    rowDir=-1    
    
            def scoreCompetingCell(index):
                competingCell=Cell.makeFromIndex(index)
                colDiff=competingCell.col-currentCell.col
                rowDiff=competingCell.row-currentCell.row
                
                #rates the agreement between the current velocity along an axis and the velocity required to get the cell of the proposed index
                colScore=abs(colDir-colDiff)
                rowScore=abs(rowDir-rowDiff)
                
                combinedScore=colScore+rowScore
            
            print "Current Direction <"+str(colDir)+","+str(rowDir)+">" 
                            
            bestLowestIndex=min(lowestIndexes, key=scoreCompetingCell)        
    
            print "Best lowest index: "+str(bestLowestIndex)
    
            return bestLowestIndex
        else:
            return lowestIndexes[0]

    def reconstructPath():
        global G_PrintDebug
        printt(1,"Reconstructing Path...")
        pathCells = []
        
#         printt(2,"closed set is:")
#         for key in closedset.keys():
#             printt(str(key)+"\t"+Cell.makeFromIndex(key).toString()+"\t"+closedset[key].toString())
            
        maxAllowablePath=G_GridRows*G_GridCols
        
        currentNodeIndex=G_Goal.getIndex()
        currentNode=closedset[currentNodeIndex]
        printt(2,"initial node: "+currentNode.toString())
        G_PrintDebug=False  
        while currentNode.parentIndex is not None:
            printt(3,"Appending current node "+currentNode.toString()+"to path")
            pathCells.append(Cell.makeFromIndex(currentNodeIndex))  #append the current cell to our path
            currentNodeIndex=currentNode.parentIndex                #get the parent index and use it for the next loop
            currentNode=closedset[currentNodeIndex]                 #get the node and use it for the next loop
        
            if (len(pathCells)>maxAllowablePath):
                raise Exception("PATH TOO LONG!")
        G_PrintDebug=True
        return pathCells
        printt(1," ...Done Reconstructing path")
    
    goalIndex=G_Goal.getIndex()
    currentIndex=G_Start.getIndex()
    G_PrintDebug=False
    while openset:       
        currentIndex = getLowestFScore(currentIndex)
        if currentIndex==goalIndex:
            print "FOUND GOAL at "+Cell.makeFromIndex(currentIndex).toString()+" ("+str(currentIndex)+")!"
            print "Start was "+G_Start.toString()+" ("+str(G_Start.getIndex())+")"
            
            publishFrontier(openset)
            publishExplored(closedset) 
            
            #put the goal on the closed set so the path can be reconstructed from the closed set
            closedset[currentIndex] = openset[currentIndex]
            G_PrintDebug=True  
            return reconstructPath()

        currentNode=openset[currentIndex]       #get the value (Node) from the open set dictionary corresponding to the key which is the currentIndex
        closedset[currentIndex] = openset.pop(currentIndex) #remove the key and value from the open set, assign that value to the currentIndex key of the closedset
#         currentCell= Cell.makeFromIndex(currentIndex)   
        
        printt(1,"Current Node: ("+str(currentIndex)+") C"+ Cell.makeFromIndex(currentIndex).toString() + " V"+currentNode.toString()) #currentCell.toString() #INDEX-BASED
        
#         neighbors=currentCell.getUnoccupiedNeighbors()
        neighborIndeces=Cell.getIndecesOfUnoccupiedNeighborsOfCellWithIndex(currentIndex) #INDEX-BASED

        printt(2,"analysing "+str(len(neighborIndeces))+" neighbors...")
        
        for neighborIndex in neighborIndeces:   #INDEX-BASED
#         for neighbor in neighbors:

#             neighborIndex=neighbor.getIndex()
            printt(2,"neighbor "+str(neighborIndex)+":\t"+Cell.makeFromIndex(neighborIndex).toString()) #neighbor.toString()) #INDEX-BASED
            tentative_g_score = currentNode.gScore + Cell.costDistanceBetweenIndeces(neighborIndex,currentIndex) #neighbor.costDistanceFromCellWithIndex(currentIndex)
            tentative_f_score = tentative_g_score + G_Heuristic[neighborIndex]
            
            if (closedset.__contains__(neighborIndex)) and (tentative_f_score >= closedset[neighborIndex].fScore):
                printt(3,"neighbor is already on closed set with lower f score, going to next neighbor...")
                continue
                
            if (not openset.__contains__(neighborIndex)) or (tentative_f_score < openset[neighborIndex].fScore):
                neighborNode=Node.makeNodeWithFScore(tentative_g_score,tentative_f_score,currentIndex)
                printt(3,"neighbor either not on open set or on open set with lower f score...")
                printt(3,"adding neighbor node to open set "+neighborNode.toString())
                openset[neighborIndex] = neighborNode
            else:
                printt(3,"neighbor already on open set with lower f score - going to next neighbor...")    
    
        publishFrontier(openset)
        publishExplored(closedset)      
    
    return None
    
    print "DONE CALCULATING A STAR"
    
if __name__ == '__main__':
    
    # TEST EVERYTHING
#     testAll()
    
    # register our node with roscore under this name
    rospy.init_node('LAB3')  
    
    ############### publishers
    rospy.loginfo("MAKING PUBLISHERS")
    heuristicPublisher = rospy.Publisher('lab3_heuristic', OccupancyGrid)
    occupancyGridPublisher = rospy.Publisher('lab3_occupancyMap', OccupancyGrid)
    startGoalDebugOccupancyGridPublisher=rospy.Publisher('lab3_StartGoalMap', OccupancyGrid)
    inflatedOccupancyGridPublisher=rospy.Publisher('lab3_InflatedoccupancyGrid', GridCells)
    exploredPublisher = rospy.Publisher('lab3_explored', GridCells)
    frontierPublisher = rospy.Publisher('lab3_frontier', GridCells)
    startPublisher = rospy.Publisher('lab3_Start', GridCells)
    goalPublisher = rospy.Publisher('lab3_Goal', GridCells)
    pathCellsPublisher=rospy.Publisher('lab3_PathCells', GridCells)
    pathPublisher = rospy.Publisher('lab3_path', Path)
    explorationFrontierPublisher=rospy.Publisher('lab3_explorationFrontier', GridCells)
    explorationFrontierCentroidsPublisher=rospy.Publisher('lab3_explorationFrontierCentroids',GridCells)
    
    ############### SUBSCRIBERS
    rospy.loginfo("SUBSCRIBING TO TOPICS")
    rospy.Subscriber("/map", OccupancyGrid, occupancyGridCallback)  # subscribe to OccupancyGrids posted on the map topic
#     rospy.Subscriber("map_metadata", MapMetaData, mapMetaDataCallback) #subscribe to MapMetaData posted on the 
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseCallback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, simpleGoalCallback)
    rospy.Subscriber("/clicked_point", PointStamped, clickedPointCallback)
    
    # MAIN PROGRAM
    try:
        print "\n"*2
        rospy.sleep(0.5)  # wait for first map to come in before doing anything

        # LOOP FOREVER [or rospy.spin()]
        while not rospy.is_shutdown():
            rospy.spin()  
    except rospy.ROSInterruptException:  # main program loops so check it for exceptions while its waiting inbetween loops
        pass
