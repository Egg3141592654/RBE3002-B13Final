from math import sqrt

"""
This module will be the storage protocol by which 
the clusters will be stored and dequeued.
This class will also define a cluster object to be read
"""

class ClusterPoint():
    def __init__(self, cellsEnumerated, rows, columns):
        """
        This constructor takes a group of cells that corresponds to the absolute position in the matrix
        We are going to convert this to get the centroid of the matrix and establish some bounds.
        """
        #We are just going to give 2 extreme values on the list to path plan to
        #this is a tuple of tuples sorted by x and y points in the real map 
        cellsEnumerated = sorted(cellsEnumerated)
        
        self.Extrema = ((int(cellsEnumerated[0] / rows) , cellsEnumerated[0] % rows), \
                        (int(cellsEnumerated[len(cellsEnumerated)-1] / rows) , cellsEnumerated[len(cellsEnumerated)-1] % rows))
        
    def calculateBestPoint(self, x, y):
        """
        This function returns a tuple of the best point to take when navigating to this cluster
        """
        distance1 = sqrt((x-self.Extrema[0][0])**2+(y-self.Extrema[0][1])**2)
        distance2 = sqrt((x-self.Extrema[1][0])**2+(y-self.Extrema[1][1])**2)
        if distance1 >= distance2:
            return self.Extrema[0]
        return self.Extrema[1]