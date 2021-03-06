import scipy.io
import numpy as np


class Rectangle:
    x_min = None
    x_max = None
    y_min = None
    y_max = None

    def __str__(self):
        return f"P1: [{self.x_min}, {self.y_min}], P2: [{self.x_max}, {self.y_max}]"


def mat2rect(mat):
    mat = mat["map"]
    mat = np.flip(mat, 0)
    rectangles = []
    height = mat.shape[0]
    length = mat.shape[1]
    for i in range(length):
        column = mat[:, i]
        newrectangle = False
        for rect in rectangles:
            if i != rect.x_max + 1:
                continue
            if column[rect.y_min]:
                validRect = True
                for k in range(rect.y_min, rect.y_max + 1):
                    if column[k] == 0:
                        validRect = False
                if validRect:
                    rect.x_max = i
                    column[rect.y_min : rect.y_max + 1] = 0

        for j in range(height):
            if column[j] == 1 and not newrectangle:
                start = j
                end = j
                newrectangle = True
            elif column[j] == 1:
                end = j
            elif newrectangle and column[j] == 0:
                rect = Rectangle()
                rect.x_min = i
                rect.y_min = start
                rect.x_max = i
                rect.y_max = end
                rectangles.append(rect)
                newrectangle = False
    return rectangles