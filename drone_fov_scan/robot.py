#S.Diane, 2023
import math

import pygame.draw
from ngon import *

class Robot:
    def __init__(self, x, y, a):
        self.x, self.y, self.a = x, y, a
        self.ngon=Ngon([
            [0,-25],
            [100,-75],
            [100,75],
            [0,25]
        ])
    def getPos(self):
        return [self.x, self.y]
    def draw(self, screen):
        self.ngon.ang=self.a/180*math.pi
        self.ngon.x=self.x
        self.ngon.y=self.y
        pygame.draw.circle(screen, (0,150,0), (self.x, self.y), 15)
        self.ngon.draw(screen, (200,150,0))