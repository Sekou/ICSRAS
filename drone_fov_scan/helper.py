#S.Diane, 2023

import math

import pygame.draw
import numpy as np
from ngon import *

def dist(n1, n2):
    return np.linalg.norm((n1[0]-n2[0], n1[1] - n2[1]))

def rot(p, ang):
    s,c=math.sin(ang), math.cos(ang)
    x=p[0]*c-p[1]*s
    y=p[0]*s+p[1]*c
    return [x,y]

def rotArr(arr, ang):
    res=[rot(p, ang) for p in arr]
    return res

def doSegmentsIntersect(s0, s1):
    w0 = s0[1][0] - s0[0][0]
    w1 = s1[1][0] - s1[0][0]
    h0 = s0[1][1] - s0[0][1]
    h1 = s1[1][1] - s1[0][1]
    # векторные произведения (площади треугольников, образованных из сочетаний точек отрезков)
    p0 = h1 * (s1[1][0] - s0[0][0]) - w1 * (s1[1][1] - s0[0][1])
    p1 = h1 * (s1[1][0] - s0[1][0]) - w1 * (s1[1][1] - s0[1][1])
    p2 = h0 * (s0[1][0] - s1[0][0]) - w0 * (s0[1][1] - s1[0][1])
    p3 = h0 * (s0[1][0] - s1[1][0]) - w0 * (s0[1][1] - s1[1][1])
    return p0 * p1 <= 0 and p2 * p3 <= 0

def isPtInsideConvexNgon(p, ngon):
    c = np.mean(ngon, axis=0)
    s0 = [p, c]
    for i in range(len(ngon)):
        s1 = [ngon[i - 1], ngon[i]]
        if (doSegmentsIntersect(s0, s1)):
            return False
    return True

def getTrajLen(traj):
    res = 0
    for i in range(1, len(traj)):
        res += dist(traj[i - 1], traj[i])
    return res

def interpolatePos(traj, L):
    LSum = 0
    for i in range(1, len(traj)):
        p1, p2 = traj[i - 1], traj[i]
        step=dist(p1, p2)
        LSum += step
        if LSum>L:
            dL=LSum-L
            k1=dL/step
            k2=1-k1
            return np.array(p1)*k1+np.array(p2)*k2
    if L<0: return traj[0]
    return traj[-1]


def interpolateAng(traj, L):
    LSum = 0
    for i in range(1, len(traj)):
        p1, p2 = traj[i - 1], traj[i]
        p3 = traj[i+1] if i<len(traj)-1 else None
        d1=np.subtract(p2, p1)
        d2=d1 if p3 is None else np.subtract(p3, p2)
        a1=math.atan2(d1[1], d1[0])
        a2=math.atan2(d2[1], d2[0])
        step = dist(p1, p2)
        LSum += step
        if LSum > L:
            dA = a2-a1
            while dA>math.pi: dA-=2*math.pi
            while dA<-math.pi: dA+=2*math.pi
            dL=LSum-L
            k2 = 1 - dL / step
            #квадрат k2 для более резкого поворота ближе к следующей точке
            a_ = a1+dA * k2*k2 #TODO: нормализовать a_
            return a_
    if L < 0: return a1
    return a2

pygame.font.init()
font = pygame.font.SysFont('Comic Sans MS', 35)
def drawText(screen, str, x, y):
    text_surface = font.render(str, False, (0, 0, 0))
    screen.blit(text_surface, (x,y))