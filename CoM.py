#Importing all used packages
import numpy as np
from scipy.integrate import quad
from scipy.linalg import norm
from matplotlib import pyplot as plt

class Fuel_Tank:
    def __init__(self): #Initialising values
        self.R = 0
        self.p0 = [0, 0, 0]
        self.p1 = [0, 0, 0]   
        self.fuel_level = 1
        self.fuel_type = 1
        self.y_dev_arr = []
        self.z_dev_arr = []
        
    def run(self, rocket_arr, tilt):
        #Setting the mas values
        self.p0 = np.array(rocket_arr[0])
        self.p1 = np.array(rocket_arr[1])
        self.R = rocket_arr[2]
        self.fuel_level = rocket_arr[3]
        self.fuel_type = rocket_arr[4]
        self.tank_no = rocket_arr[5]
        self.tilt = tilt
        if self.fuel_type == 1:
            fuel_dens = 901
        elif self.fuel_type == 2:
            fuel_dens = 1858
            self.tilt = 0
        #Vector in direction of axis
        v = self.p1 - self.p0
        #Find magnitude of vector
        mag = norm(v)
        #Unit vector in direction of axis
        v = v / mag
        #Make some vector not in the same direction as v
        not_v = np.array([1, 0, 0])
        if (v == not_v).all():
            not_v = np.array([0, 1, 0])
        #Make vector perpendicular to v
        n1 = np.cross(v, not_v)
        #Normalize n1
        n1 /= norm(n1)
        #Make unit vector perpendicular to v and n1
        n2 = np.cross(v, n1)
        #Surface ranges over t from 0 to length of axis and 0 to 2*pi
        t = np.linspace(0, mag, 100)
        theta = np.linspace(0, 2 * np.pi, 100)
        #Use meshgrid to make 2d arrays
        t, theta = np.meshgrid(t, theta)
        #Generate coordinates for surface
        X, Y, Z = [self.p0[i] + v[i] * t + self.R * np.sin(theta) * n1[i] + self.R * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        ax.plot_surface(X, Y, Z, alpha=0.44)
        
        volume = np.pi * self.R**2 * mag * self.fuel_level
        tank_mass = np.pi * mag * ((self.R+0.00635)**2 - self.R**2) * 7850
        fuel_mass = volume * fuel_dens
        plate_area = quad(lambda x: mag*self.fuel_level + np.tan(self.tilt)*x, -self.R, self.R)[0]
        
        #Working out centrre of mass in y and z direction
        CoM_y = quad(lambda y: y*((y*(np.tan(self.tilt)) + mag*self.fuel_level)), -self.R, self.R)[0]/plate_area + self.p0[1]
        CoM_z = quad(lambda y: ((y*(np.tan(self.tilt)) + mag*self.fuel_level)**2) / 2 , -self.R, self.R)[0]/plate_area + self.p0[2]
        
        fuel_CoM = np.array([0, CoM_y, CoM_z])
        tank_CoM = np.array([0, self.p0[1], mag/2])
        
        #Working out total centre of mass
        CoM = ((fuel_CoM*fuel_mass + tank_CoM*tank_mass)/(tank_mass + fuel_mass))
        print("Mass of tank {} at fuel level {} is {} kg".format(self.tank_no, self.fuel_level, tank_mass + fuel_mass))
        #print("CoM is located at {}".format(CoM))
        #print("Wet mass = {} and Dry mass = {}".format(tank_mass + fuel_mass, tank_mass))
        
        #Plotting CoM
        ax.scatter(CoM[0], CoM[1], CoM[2], alpha=1, color="black")
        ax.text(CoM[0], CoM[1], CoM[2], "CoM {}".format(self.tank_no), alpha=1)
        
        y_CoM_pos_arr.append(CoM[1])
        z_CoM_pos_arr.append(CoM[2])

        y_pos_f_CoM.append(fuel_CoM[1])
        z_pos_f_CoM.append(fuel_CoM[2])


tank1 = [[0, 0, 0], [0, 0, 20], 2, 0.95, 1, 1]
tanks = [tank1]

tankgen = Fuel_Tank()

z_CoM_arr = []
y_CoM_arr = []
fuel_arr = []

#Running over a time integral, simulating the rocket burning fuel
for angle in range(11):
    y_CoM_pos_arr = []
    z_CoM_pos_arr = []

    y_pos_f_CoM = []
    z_pos_f_CoM = []
    
    fuel_lev_arr = []
    
    for x in range(90):
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111, projection='3d')
        for each in tanks:
            tankgen.run(each, np.pi/180*angle)
            fuel_lev_arr.append(each[3])
            each[3] -= 0.01
        #Plot Graph of 3d cylinder
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_zlim(0, 20)
        ax.view_init(30, 30)
        ax.set_xlabel("Depth")
        ax.set_ylabel("Width")
        ax.set_zlabel("Height")
        plt.show()
    tanks[0][3] = 0.95
    z_CoM_arr.append(z_CoM_pos_arr)
    y_CoM_arr.append(y_CoM_pos_arr)
    fuel_arr.append(fuel_lev_arr)
    
    fig2 = plt.figure(figsize=(12,12))
    plt.title("A representation of the movement of the locations of the centre of mass for the total wet mass (of the whole tank) and of the fuel")
    plt.plot(y_CoM_pos_arr, z_CoM_pos_arr, label = "Centre of Mass (Whole Tank)")
    plt.plot(y_pos_f_CoM, z_pos_f_CoM, label = "Centre of Mass (Fuel)")
    plt.legend()
    plt.xlabel("Y CoM Position")
    plt.ylabel("Z CoM Position")
    plt.show()
    
#Plot 3D graph of variation in AoA
fig3 = plt.figure(figsize=(10,10))
ax = fig3.add_subplot(111, projection='3d')
ax.view_init(0, 90)
for val in range(len(z_CoM_arr)):
    ax.plot(fuel_arr[val], y_CoM_arr[val], z_CoM_arr[val], label = "Centre of Mass (AoA {} degrees)".format(val))
plt.legend()
plt.title("3D Plot of the Centre of Mass with Various AoA in the Y and Z direction")
plt.xlabel("Fuel Level")
plt.ylabel("Y CoM Position")
ax.set_zlabel("Z CoM Position")
plt.show()

fig4 = plt.figure(figsize=(10,10))
for val in range(len(y_CoM_arr)):
    plt.plot(fuel_arr[val], y_CoM_arr[val], label = "Centre of Mass in Y (AoA {} degrees)".format(val))
plt.legend()
plt.title("Showing how the CoM Changes with various AoA in the Y direction")
plt.xlabel("Fuel Level")
plt.ylabel("Y CoM Position")
plt.show()


