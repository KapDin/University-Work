#Importing required modules
import pygame, random, math, statistics
from pygame import gfxdraw #For some reason, the gfxdraw module needs to be imported seperately, pygame.gfxdraw doesn't work
import numpy as np
from scipy.spatial import distance

#Setting the base variables, such as FPS
fps = 144
numCars = 10
    
class Runtime: #This class is the main backbone of the simulation
    def __init__(self): #Initialising values
        self.t = 0.0
        self.frame_count = 0
        self.dt = 1/fps
        self.roads = []
        
    def update(self): #Function to update the simulation every frame
        self.t += self.dt
        self.frame_count += 1
    
    def run(self): #Function to run the simulation for each number of steps per "tick"
        self.update()
            
    def create_road(self, start, end):#Function to create the roads used in the simulation
        road = Road(start, end)
        self.roads.append(road)
        return road

    def create_roads(self, road_list): #Creating the roads into a list for ease of transport
        for road in road_list:
            self.create_road(*road)
            
    def spawn_car(self, vehList): #The function to spawn each vehicle on the track
        for num in range(1, numCars + 1):
            road = self.running.roads[0]
            vehTemp = Vehicle(road)
            vehTemp.current_road_no = self.running.roads.index(road)
            vehTemp.x_pos = (road.length/numCars) * num
            self.vehList.append(vehTemp)
        return vehList
        
            
class GUI: #The class for the main GUI, or window, to draw everything
    def __init__(self, running):
        self.running = running
        self.width = 1920
        self.height = 1080
        self.zoom = 1
        self.offset = (0, 0)
        self.mouse_down = False
        self.vehList = []
        self.avg_vel = 0
        self.avg_vel_list = []

    def loop(self, loop=None): #The function that loops pygame to be updated every frame
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.flip()
        clock = pygame.time.Clock()
        pygame.font.init()
        self.font_text = pygame.font.SysFont('Lucida Console', 16)
        isRun = True
        self.vehList = Runtime.spawn_car(self, self.vehList)
        
        while isRun:
            loop(self.running)
            self.draw_main_win()
            self.draw_roads() #Draws things on the screen
            self.vehRun()
            pygame.display.update()
            clock.tick(fps)
            for event in pygame.event.get():
                if event.type == pygame.QUIT: #Closes the window when it is told to
                    pygame.display.quit()
                    isRun = False
                elif event.type == pygame.MOUSEBUTTONDOWN: #All events associated with the mouse, such as GUI movement and zoom
                    if event.button == 1:
                        x, y = pygame.mouse.get_pos()
                        x0, y0 = self.offset
                        self.mouse_last = (x-x0*self.zoom, y-y0*self.zoom)
                        self.mouse_down = True
                    elif event.button == 3:
                        veh_to_brake = random.choice(self.vehList)
                        veh_to_brake.vel = veh_to_brake.vel - 5
                        print("done")
                    elif event.button == 4:
                        self.zoom *=  (self.zoom**2+self.zoom/4+1) / (self.zoom**2+1)
                    elif event.button == 5:
                        self.zoom *= (self.zoom**2+1) / (self.zoom**2+self.zoom/4+1)
                elif event.type == pygame.MOUSEMOTION:
                    if self.mouse_down:
                        x1, y1 = self.mouse_last
                        x2, y2 = pygame.mouse.get_pos()
                        self.offset = ((x2-x1)/self.zoom, (y2-y1)/self.zoom)
                elif event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_down = False    
        
    def run(self): #Function that loops the entire game with the number of steps per update
        def loop(running):
            running.run()
        self.loop(loop)
        
    def vehRun(self): #The Function that runs each vehicle, and it's general movement
        for front_veh in self.vehList:
            self.draw_vehicle(front_veh, front_veh.road, front_veh.color)
            self.avg_vel_list.append(front_veh.vel)
            veh = self.vehList[(self.vehList.index(front_veh) + 1)%numCars]
            if front_veh.x_pos <= front_veh.road.length: #If the vehicle is still on the road segment, continue on the road
                front_veh.x_pos = front_veh.x_pos + ((front_veh.vel / fps) + (1/2 * (front_veh.acc / (fps*fps))))
                if front_veh.vel/fps < front_veh.vel_max/fps :
                    front_veh.vel += front_veh.acc / fps
            else: #If the vehicle needs to move to the next segment
                front_veh.current_road_no += 1
                front_veh.current_road_no = (front_veh.current_road_no)%len(self.running.roads)
                front_veh.x_pos = front_veh.x_pos - front_veh.road.length + 0.01
                front_veh.road = self.running.roads[front_veh.current_road_no]
                
            minSep = veh.len / 2 + front_veh.len / 2 #The minimum seperation between 2 vehicles, if a vehicle comes closer than this, it stops completely.
            
            """Due to personal brain limitations, the below if and elifs are to check how far seperated the vehicles are""" 
            if veh.current_road_no == front_veh.current_road_no: #If the 2 vehicles are on the same road
                veh_sep = front_veh.x_pos - veh.x_pos
                if minSep <= veh_sep <= 2 * veh.len * veh.vel:
                    veh.vel += (front_veh.vel - veh.vel) / (veh_sep * 2 * math.sqrt(veh.acc * veh.dec))
                if veh_sep <+ minSep:
                    veh.vel = 0

            elif front_veh.current_road_no > veh.current_road_no: #If the vehicle ahead isn't a full loop around
                road_inbetween_len = 0
                for x in range(veh.current_road_no + 1, front_veh.current_road_no):
                    road_inbetween_len = road_inbetween_len + self.running.roads[x].length
                veh_sep = front_veh.x_pos + veh.road.length - veh.x_pos + road_inbetween_len - veh.len/2 - front_veh.len/2
                if minSep <= veh_sep <= 2 * veh.len * veh.vel:
                    veh.vel += (front_veh.vel - veh.vel) / (veh_sep * 2 * math.sqrt(veh.acc * veh.dec))
                if veh_sep <+ minSep:
                    veh.vel = 0

            elif front_veh.current_road_no < veh.current_road_no: #If the vehicle ahead has looped around, from road segment n to 0
                road_inbetween_len = 0
                for x in range((veh.current_road_no + 1), len(self.running.roads)):
                    road_inbetween_len = road_inbetween_len + self.running.roads[x%8].length
                for x in range(0, front_veh.current_road_no):
                    road_inbetween_len = road_inbetween_len + self.running.roads[x].length
                veh_sep = front_veh.x_pos + veh.road.length - veh.x_pos + road_inbetween_len - veh.len/2 - front_veh.len/2
                if minSep < veh_sep <= 2 * veh.len * veh.vel:
                    veh.vel += (front_veh.vel - veh.vel) / (veh_sep * 2 * math.sqrt(veh.acc * veh.dec))
                if veh_sep <= minSep:
                    veh.vel = 0
            if front_veh.vel <= 0:
                front_veh.vel = 0
        self.avg_vel = statistics.mean(self.avg_vel_list)
        self.avg_vel_list.clear()

    def convert_to_movable(self, x, y=None):#Converts to the movable co-ordinates, so you can click and drag the mouse to "look around"
        if isinstance(x, list):
            return [self.convert_to_movable(e[0], e[1]) for e in x]
        if isinstance(x, tuple):
            return self.convert_to_movable(*x)
        return (int(self.width/2 + (x + self.offset[0])*self.zoom), int(self.height/2 + (y + self.offset[1])*self.zoom))

    def convert_to_screen(self, x, y=None): #Converts to screen co-ordinates from the movable co-ordinate position
        if isinstance(x, list):
            return [self.convert_to_movable(e[0], e[1]) for e in x]
        if isinstance(x, tuple):
            return self.convert_to_movable(*x)
        return (int(-self.offset[0] + (x - self.width/2)/self.zoom), int(-self.offset[1] + (y - self.height/2)/self.zoom))

    def line(self, start_pos, end_pos, color):
        gfxdraw.line(self.screen, *start_pos, *end_pos, color)
        
    def draw_grid(self, unit=50, color=(150,150,150)): #Function that draws a grid on the background to give a sense of scale and position
        x_left, y_top = self.convert_to_screen(0, 0)
        x_right, y_bottom = self.convert_to_screen(self.width, self.height)
        x1 = int(x_left / unit)
        y1 = int(y_top / unit)
        x2 = int(x_right / unit)+1
        y2 = int(y_bottom / unit)+1

        for i in range(x1, x2): #Drawing the line
            self.line(self.convert_to_movable((unit*i, y_top)), self.convert_to_movable((unit*i, y_bottom)), color)
        for i in range(y1, y2):
            self.line(self.convert_to_movable((x_left, unit*i)), self.convert_to_movable((x_right, unit*i)), color)

    def draw_info(self): #Function that renders the on screen text (Time, Frame count, Average V, etc)
        text_fps = self.font_text.render(f'time={self.running.t:.5}', False, (0, 0, 0))
        text_frc = self.font_text.render(f'frame={self.running.frame_count}', False, (0, 0, 0))
        text_avg_vel = self.font_text.render(f'avg speed={self.avg_vel}', False, (0, 0, 0))
        self.screen.blit(text_fps, (0, 0))
        self.screen.blit(text_frc, (200, 0))
        self.screen.blit(text_avg_vel, (400, 0))

    def draw_main_win(self): #Function to draw the main window of the simulation
        self.screen.fill((200, 245, 200))  
        self.draw_grid(10, (220,220,220))
        self.draw_grid(100, (200,200,200))
        self.draw_grid(1000, (100, 100, 100))
        self.draw_info()
        
    def draw_rect(self, pos, size, angle=None, cos=None, sin=None, centered=True, color=(0, 0, 0)):
        #Function to draw a rotated rectangle given the above specifications
        x, y = pos
        length, height = size

        if angle:
            cos, sin = np.cos(angle), np.sin(angle)
        
        corner = lambda n1, n2: (x + (n1*length*cos + n2*height*sin)/2, y + (n1*length*sin - n2*height*cos)/2) #Calculating the corners of the polygon to be drawn

        if centered:
            corners = self.convert_to_movable([corner(*n) for n in [(-1,-1), (-1, 1), (1,1), (1,-1)]])
        else:
            corners = self.convert_to_movable([corner(*n) for n in [(0,-1), (0, 1), (2,1), (2,-1)]])

        gfxdraw.aapolygon(self.screen, corners, color) #Draws the shapes
        gfxdraw.filled_polygon(self.screen, corners, color)
    
    def draw_roads(self): #Function that draws all roads given their co-ordinates
        for road in self.running.roads:
            self.draw_rect(road.start, (road.length, 4), cos=road.cos, sin=road.sin, color=(180, 180, 180), centered=False)
            
    def draw_vehicle(self, vehicle, road, color): #Function to draw each vehicle
        l, h = vehicle.len,  2
        sin, cos = road.sin, road.cos
        x = road.start[0] + cos * vehicle.x_pos
        y = road.start[1] + sin * vehicle.x_pos
        self.draw_rect((x, y), (l, h), cos=cos, sin=sin, color=color, centered=True)

    def draw_vehicles(self): #Draws all vehicles
        for road in self.sim.roads:
            for vehicle in road.vehicles:
                self.draw_vehicle(vehicle, road)
            
class Road: #The road class, with all individual road properties
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.vehicles = []
        self.length = distance.euclidean(self.start, self.end)
        self.sin = (self.end[1]-self.start[1]) / self.length
        self.cos = (self.end[0]-self.start[0]) / self.length
        self.angle = np.arctan2(self.end[1] - self.start[1], self.end[0] - self.start[0])

class Vehicle: #The vehicle class, with all individual vehicle properties
    def __init__(self, road):
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        self.road = road
        self.vel_max = 35
        self.current_road_no = 0
        self.len = random.uniform(2.5, 6)
        self.x_pos = random.uniform(0, road.length)
        self.vel = self.vel_max
        self.acc = 1.5
        self.dec = 2.3
    
            
run = Runtime() #Function for the simulation

run.create_roads([ #Creates the specified set of roads
    ((0, 0), (50, 0)),
    ((50, 0), (100, 50)),
    ((100, 50), (100, 100)),
    ((100, 100), (50, 150)),
    ((50, 150), (0, 150)),
    ((0, 150), (-50, 100)),
    ((-50, 100), (-50, 50)),
    ((-50, 50), (0, 0))
])

start = GUI(run) #Draws the program
start.run() #Starts