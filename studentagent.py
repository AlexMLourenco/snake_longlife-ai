from agent import *
import random
import math
import heapq
import pickle


class Cord(object):
    def __init__(self, x, y, canReach):
        self.x = x
        self.y = y 
        self.canReach = canReach
        self.parent = None
        self.c = 0
        self.h = 0
        self.f = 0
    def __lt__(self,other):
        return self.f < other.f


class AStar(object):
    def __init__(self):
        self.opencords = []
        heapq.heapify(self.opencords)
        self.closecords = set()
        self.cords = []
        self.world = None

    def init_grid(self, world, start, end,vision,deadz):
        self.world = world
        for x in range(world.size.x):
            for y in range(world.size.y):
                if Point(x, y) in world.walls or Point(x,y) in vision.bodies or Point(x,y) in deadz:
                    canReach = False
                else:
                    canReach = True
                self.cords.append(Cord(x, y, canReach))
        #print("--------------")
        #print(self.cords)
        #print("-----------------")
        self.start = self.get_cord(start.x,start.y)
        #print(self.start)
        self.end = self.get_cord(end.x,end.y)


    def get_heuristic(self,adj, cord):
        
        return self.world.dist(Point(adj.x,adj.y),Point(cord.x,cord.y)) * 10
    
    def get_cord(self, x, y):
        
        return self.cords[x * self.world.size.y + y]
    
    def get_adjacent_cords(self, cord):
        cords = []
        if cord.x <= self.world.size.x-1:
            
            if cord.x == self.world.size.x-1:
                cords.append(self.get_cord(0, cord.y))
            else:
                cords.append(self.get_cord(cord.x+1, cord.y))
            
        if cord.y >= 0:
            if cord.y ==  0:
                cords.append(self.get_cord(cord.x, self.world.size.y-1))
            else:
                cords.append(self.get_cord(cord.x, cord.y-1))
            
        if cord.x >= 0:
            if cord.x == 0:
                cords.append(self.get_cord(self.world.size.x-1, cord.y))
            else:
                cords.append(self.get_cord(cord.x-1, cord.y))
            
        if cord.y <= self.world.size.y-1:
            if cord.y == self.world.size.y-1:
                cords.append(self.get_cord(cord.x, 0))
            else:
                cords.append(self.get_cord(cord.x, cord.y+1))
        
        return cords
        
        '''
        cords = []
        if cord.x == self.world.size.x:
            cords.append(self.get_cord(0, cord.y))
            cords.append(self.get_cord(cord.x-1, cord.y))
            cords.append(self.get_cord(cord.x, cord.y+1))
            cords.append(self.get_cord(cord.x, cord.y-1))
        if cord.x == 0:
            cords.append(self.get_cord(self.world.size.x, cord.y))
            cords.append(self.get_cord(cord.x+1, cord.y))
            cords.append(self.get_cord(cord.x, cord.y+1))
            cords.append(self.get_cord(cord.x, cord.y-1))
        if cord.y ==  0:
            cords.append(self.get_cord(cord.x, self.world.size.y))
            cords.append(self.get_cord(cord.x, cord.y +1))
            cords.append(self.get_cord(cord.x-1, cord.y))
            cords.append(self.get_cord(cord.x+1, cord.y))
        if cord.y ==  self.world.size.y:
            cords.append(self.get_cord(cord.x, 0))
            cords.append(self.get_cord(cord.x, cord.y +1))
            cords.append(self.get_cord(cord.x-1, cord.y))
            cords.append(self.get_cord(cord.x+1, cord.y))
        return cords
        '''
        
    
        
    def get_path(self):
        cord = self.end
        path = [(cord.x, cord.y)]
        while cord.parent is not self.start:
            cord = cord.parent
            path.append((cord.x, cord.y))

        path.append((self.start.x, self.start.y))
        path.reverse()
        return path

    def update_cords(self, adj, cord):
  
        adj.c = cord.c + 10
        adj.h = self.get_heuristic(adj,cord)
        adj.parent = cord
        adj.f = adj.h + adj.c

    def solve(self):

        heapq.heappush(self.opencords, (self.start.f, self.start))
    
        while len(self.opencords):
            f, cord = heapq.heappop(self.opencords)
            self.closecords.add(cord)
            if cord is self.end:
                return self.get_path()
            cords_adj = self.get_adjacent_cords(cord)
           
            for adj_cord in cords_adj:
                if adj_cord.canReach and adj_cord not in self.closecords:
                    if (adj_cord.f, adj_cord) in self.opencords:
                        if adj_cord.h > cord.h:
                            self.update_cords(adj_cord, cord)
                    else:
                        self.update_cords(adj_cord, cord)
                        heapq.heappush(self.opencords, (adj_cord.f, adj_cord))
                        
        

class StudentAgent(Agent):
    def __init__(self, name, body, world):
        super().__init__(name, body, world)
        self.mypath=[]
        self.destino = 0
        self.deadzones = self.getDeadZones()  
        
        

    def xau(self):
        return self.deadzones

    def chooseAction(self, vision, msg):
        head = self.body[0]
        #print(self.deadzones)
        if self.mypath == None or len(self.mypath) == 0:
            cord_init = Cord(head.x,head.y,True)
            destino = list(vision.food.keys())
            if len(destino) > 0 :
                destino.sort(key = lambda x : self.world.dist(head,x)) 
                self.destino = destino[0]
                cord_dest = Cord(destino[0].x,destino[0].y,True)
            else:
                destino = self.world.generatePos(self.world.walls, vision.food.keys())
                cord_dest = Cord(destino.x,destino.y,True)
                self.destino = destino

            p = AStar()

            p.init_grid(self.world,cord_init,cord_dest,vision, self.deadzones )
            self.mypath = p.solve()
            
            if(self.mypath == None):
                action = self.actions(head,vision)
                return action,b""
            self.mypath = self.mypath[1:]
        
        
        

        action = self.normalize(Point(self.mypath[0][0], self.mypath[0][1]) - head)
        self.mypath = self.mypath[1:] # guardar restantes paths
        
        next_pos = self.world.translate(head,action)

        if Point(next_pos.x,next_pos.y) in list(vision.bodies.keys()):
            action = Stay
            cord_init = Cord(head.x,head.y,True)
            p = AStar()
            
            destino = list(vision.food.keys())
            if len(destino) > 0 :
                destino.sort(key = lambda x : self.world.dist(head,x)) 
                self.destino = destino[0]
                cord_dest = Cord(destino[0].x,destino[0].y,True)
            else:
                destino = self.world.generatePos(self.world.walls, vision.food.keys())
                cord_dest = Cord(destino.x,destino.y,True)
                self.destino = destino

            p.init_grid(self.world,cord_init,cord_dest,vision, self.deadzones)
            self.mypath = p.solve()
            
            
            if(self.mypath == None):
                action = self.actions(head,vision)
                return action,b""
            self.destino = destino
            self.mypath = self.mypath[1:]
        

        return action, b"" #msg

    
    def normalize(self,point):
        if point.x == -59:
            return Point(1,0)
        
        if point.x == 59:
            return Point(-1,0)

        if point.y == 39:
            return Point(0,-1)
        
        if point.y == -39:
            return Point(0,1)

        return point

    def getDeadZones(self):
        deadz = {}
        
        for x in range(self.world.size.x):
            for y in range(self.world.size.y):
                pnt = Point(x,y)
                if pnt in self.world.walls:
                    continue
                validact = []
                for act in ACTIONS[1:]:
                    newpos = self.world.translate(pnt, act)
                    if newpos not in self.world.walls:
                        validact.append(act)
                if len(validact) < 2:
                    deadz[pnt] = 'DZ'
                    while True:
                        if len(validact) == 1:
                            old = validact[0]
                            deadz[pnt] = 'DZ'
                            pnt = self.world.translate(pnt, old)
                            validact = []
                            for act in ACTIONS[1:]:
                                newpos = self.world.translate(pnt, act)
                                if newpos not in self.world.walls:
                                    validact.append(act)
                            validact.remove(Point(-old.x,-old.y))
                            
                        else:
                            break    
        return deadz
    
    def actions(self, head, vision):
        validact = [Stay]
        destino = self.world.generatePos(self.world.walls, vision.food.keys())
        self.destino = destino
                
        for act in ACTIONS[1:]:
            newpos = self.world.translate(head, act)
            if newpos not in self.world.walls and newpos not in vision.bodies and newpos not in self.deadzones :
                validact.append(act)
        
        return random.choice(validact)
        