#S.Diane, 2023
#+расчет процента посещенности местности для выбранного поля видимости (зоны обзора)
#+учет углов поворота робота
#+редуцирование маршрута при условии сохранения 100% покрытия местности
#TODO: соединить узлы в граф и использовать структуру его связей для предотвращения скачков из одного края поля в другое

import sys, pygame

from pygame.locals import *

from robot import *
from graph import *
from helper import *

width=1000
height=600

def main():
    global robot, graph, route
    global tSim, tMove
    global mode

    screen = pygame.display.set_mode((width, height))
    timer = pygame.time.Clock()
    fps=30
    dt=1.0/fps

    ng=Ngon([
        [100,200],
        [200,200],
        [400,300],
        [550,500],
        [410,550],
        [250,500]
    ])

    def reset():
        global robot, graph, route
        global tSim, tMove
        global mode
        robot = Robot(100,300,0)
        # robot = Robot(500,300,90)
        graph = Graph(ng, 30)
        route=graph.findRoute1(robot.getPos())
        tSim = 0
        tMove = 0
        mode = "stop"  # "move"

    reset()

    vRobot=50 #px/s

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            if event.type == KEYDOWN:
                if event.key == K_1:
                    tSim=0
                    tMove=0
                    mode="move"
                    for n in graph.nodes:
                        n.visited=n.observed=False
                if event.key == K_r:
                    reset()

        #очистка экрана
        screen.fill((255,255,255))

        #1 planner checks 2 robot observes 3 visit if not observed

        route_=sorted([n for n in route if n.visited], key=lambda n:n.tVisited)
        traj1=[n.getPos() for n in route_]
        traj2=[n.getPos() for n in route if not n.observed]
        traj=traj1+traj2

        todo=[n for n in route if not n.observed]

        if mode=="move" and len(todo)>0:

            nextNode=route[0]
            for i in range(len(route)):
                if not route[i].observed:
                    nextNode=route[i]
                    break

            dx=nextNode.x-robot.x
            dy=nextNode.y-robot.y
            l=np.sqrt(dx**2+dy**2)
            robot.x+=vRobot*dx/l*dt
            robot.y+=vRobot*dy/l*dt

            # p=interpolatePos(traj, vRobot*tSim)
            # robot.x, robot.y = p
            nearestNode=graph.findNearestNode([robot.x, robot.y])
            if nearestNode.checked:
                nearestNode.visited=True
                nearestNode.tVisited=tMove

            ang=math.atan2(nextNode.y-robot.y, nextNode.x-robot.x)
            # ang=interpolateAng(traj, vRobot*tSim)
            robot.a = ang*180/math.pi

            tMove+=dt

        visNodes = graph.getVisibleNodes(robot.ngon)
        N = len(visNodes)
        drawText(screen, f"Nv = {N} visible nodes", 5, 85)

        for n in visNodes:
            n.observed=True

        observedNodes=graph.getObservedNodes()
        No = len(observedNodes)
        drawText(screen, f"No = {No} observed nodes", 5, 125)

        # pygame.draw.circle(screen, (255,0,0), (200,200), 10)

        ng.draw(screen)
        robot.draw(screen)
        graph.draw(screen)

        drawText(screen, f"Time = {tMove:.1f} s", 5, 5)

        L=getTrajLen(traj)
        drawText(screen, f"L = {L:.1f} px", 5, 45)

        for i in range(1, len(traj)):
            pygame.draw.line(screen, (200, 0, 200), traj[i-1], traj[i], 2)

        pygame.display.flip()
        timer.tick(fps)
        tSim+=dt

main()