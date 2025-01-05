#S.Diane, 2023
import pygame.draw
from ngon import *
from helper import *

class Node:
    def __init__(self, x, y):
        self.x, self.y=x, y
        self.checked=False
        self.observed=False
        self.visited=False #посещена ли роботом
    def getPos(self):
        return [self.x, self.y]

class Graph:
    def __init__(self, ngonArea, step):
        self.nodes=[]
        pts=np.array(ngonArea.pts)
        x0=np.min(pts[:,0])
        x1=np.max(pts[:,0])
        y0=np.min(pts[:,1])
        y1=np.max(pts[:,1])
        for iy in range(int(y0), int(y1), step):
            for ix in range(int(x0), int(x1), step):
                if isPtInsideConvexNgon((ix, iy), pts):
                    n=Node(ix, iy)
                    self.nodes.append(n)

    def draw(self, screen):
        for n in self.nodes:
            color=(200,100,50) if n.observed else (0,0,0)
            r=8 if n.visited else 5 if n.checked else 3
            pygame.draw.circle(screen, color, (n.x, n.y), r)

    def findNearestNode(self, p):
        dd = [dist(p, m.getPos()) for m in self.nodes]
        ind = np.argmin(dd)
        return self.nodes[ind]

    def findNearestUncheckedNode(self, p):
        nodes_ = [m for m in self.nodes if not m.checked]
        if len(nodes_) == 0: return None
        dd = [dist(p, m.getPos()) for m in nodes_]
        ind = np.argmin(dd)
        return nodes_[ind]

    def getVisibleNodes(self, ngon):
        pts=ngon.getTransformedContour()
        return [n for n in self.nodes if isPtInsideConvexNgon(n.getPos(), pts)]

    def getObservedNodes(self):
        return [n for n in self.nodes if n.observed]

    #базовая ф-ция поиска маршрута
    def findRoute1(self, pStart):
        for n in self.nodes:
            n.checked=False

        n=self.findNearestNode(pStart)
        res=[n]

        while True:
            n.checked=True
            n=self.findNearestUncheckedNode(n.getPos()) #upd n=...
            if n is None: break
            res.append(n)

        return res






