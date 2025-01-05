#S.Diane, 2023
import pygame.draw
from helper import *

class Ngon:
    def __init__(self, pts):
        self.pts=pts
        self.ang=0
        self.x=0
        self.y=0

    def getTransformedContour(self):
        pts_ = rotArr(self.pts, self.ang)
        pts_ = np.array(pts_) + [self.x, self.y]
        return pts_

    def draw(self, screen, color=(0,0,255)):
        pts_=self.getTransformedContour()

        for i in range(len(pts_)):
            p1=pts_[i-1]
            p2=pts_[i]
            pygame.draw.line(screen, color, p1, p2, 2)
