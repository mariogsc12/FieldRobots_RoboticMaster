
from coppeliaSim.utils.utils import get_client, Logger, init_logger 
from coppeliaSim.utils.math import distance_2d, distance_3d
from dataclasses import dataclass
from enum import Enum
import networkx as nx
import numpy as np
import pandas as pd
from pathlib import Path
import os

START_POSITION = [29, 5]
GOAL_POSITION = [59, 19]
SIMULATION_DATA_FOLDER_PATH = Path('C:/Users/mario/OneDrive/Desktop/MASTER/SEGUNDO_CUATRI/ROBOTS_DE_CAMPO/TRABAJO/FieldRobots_RoboticMaster/simulation_data') 
HEIGHTFIELD_FOLDER_PATH = SIMULATION_DATA_FOLDER_PATH / "heightfield"
OUTPUT_FOLDER_PATH = SIMULATION_DATA_FOLDER_PATH / "solution_path"

GRAVITY = 9.807
SUBDIV = 800
CONSTANT = 10
LINEAR_VELOCITY = 3.5
ANGULAR_VELOCITY = 5

sim = get_client()
init_logger(sim)

@dataclass
class RobotnikProperties:
    radius = 0.235/2
    wheel_distance = 0.614

class RobotWheels(Enum):
    front_left  = './front_left_wheel'
    front_right = './front_right_wheel'
    back_right  = './back_right_wheel'
    back_left   = './back_left_wheel'

    def get_handle(self):
        """
        get coppeliaSim handle of the wheel
        """
        try:
            return sim.getObject(self.value)
        except Exception as e:
            raise Exception(f"Impossible to get the handle of '{self.value}': {e}")
        
class AckermanController:
    """
    Ackerman controller for Robotnik robot
    """
    logger = Logger()
    
    def __init__(self, radius=RobotnikProperties.radius, wheel_distance=RobotnikProperties.wheel_distance):
        self.radius = radius
        self.wheel_distance = wheel_distance

        # Get all wheel handles
        self.wheels = {wheel.name: wheel.get_handle() for wheel in RobotWheels}
        
        self.logger.log(f"Succesfully initialized '{self.__class__.__name__}'")

    def move(self, linear_velocity: float, angular_velocity: float):
        """
        Compute twist (linear, angular) velocities 
            - linear_velocity in [m/s]
            - angular_velocity in [rad/s]
        """
        self.logger.log(f"Moving robot with twist: ({linear_velocity, angular_velocity})")
                
        v_right = linear_velocity + self.wheel_distance * angular_velocity
        v_left = linear_velocity - self.wheel_distance * angular_velocity

        # Convert to wheel angular velocity
        omega_right = v_right / self.radius
        omega_left = v_left / self.radius

        # Asign velocities to wheels
        sim.setJointTargetVelocity(self.wheels['front_left'],  omega_left)
        sim.setJointTargetVelocity(self.wheels['back_left'],   omega_left)
        sim.setJointTargetVelocity(self.wheels['front_right'], -omega_right)
        sim.setJointTargetVelocity(self.wheels['back_right'],  -omega_right)
        
    def stop(self):
        self.logger.log(f"Stopping robot")
        sim.setJointTargetVelocity(self.wheels['front_left'],  0)
        sim.setJointTargetVelocity(self.wheels['back_left'],   0)
        sim.setJointTargetVelocity(self.wheels['front_right'], 0)
        sim.setJointTargetVelocity(self.wheels['back_right'],  0)

class GlobalPlanner:
    """ Global Planner based on the example provided in Aula Global """
    
    logger = Logger()
    
    def __init__(self, 
                start_pos=START_POSITION,
                goal_pos=GOAL_POSITION,
                heightfield_folder_path=HEIGHTFIELD_FOLDER_PATH,
                output_folder_path=OUTPUT_FOLDER_PATH,
                subdiv=SUBDIV):
        
        self.start_position = start_pos
        self.goal_position = goal_pos
        self.heightfield_folder_path = heightfield_folder_path
        self.output_folder_path = output_folder_path
        self.subdiv = subdiv
        
        self.graph = nx.Graph()
        self.upper_left_corner = None
        self.lower_right_corner = None
        
        self.logger.log("Loading data")
        self.load_data()
        
        os.makedirs(self.output_folder_path, exist_ok=True)
        self.logger.log(f"Output folder check passed: {self.output_folder_path}")
            

    def load_data(self):
        indexes = pd.read_csv(self.heightfield_folder_path / 'indices.csv', index_col=False, dtype=np.float64, header=None)
        vertices = pd.read_csv(self.heightfield_folder_path / 'vertices.csv', index_col=False, dtype=np.float64, header=None)
        
        # EL ARREGLO MÁGICO: Aplanamos las matrices para no perder datos 
        # y convertimos los índices a enteros puros de Python
        self.indexes = indexes.to_numpy().flatten().astype(int).tolist()
        self.vertices = vertices.to_numpy().flatten().tolist()
        
    def add_nodes(self, graph):
        self.logger.log("Graph: Adding nodes")
        for i in range(len(self.vertices) // 3):
            x, y, z = self.vertices[i*3:i*3+3]
            graph.add_node(i, pos=(x, y, z))
            
    def get_corners(self, graph):
        self.logger.log("Graph: Getting corners")
        if self.upper_left_corner and self.lower_right_corner:
            return
        origin = 0, 0
        upper_left_corner = lower_right_corner = 0, 0, 0
        for node_index in graph.nodes:
            position_3d = graph.nodes[node_index]['pos']
            position_2d = position_3d[0], position_3d[1]
            x, y = position_2d
            if x < 0 and y > 0:
                distance = distance_2d(position_2d, origin)
                if distance > distance_2d(upper_left_corner[:2], origin):
                    upper_left_corner = position_3d
            elif x > 0 and y < 0:
                distance = distance_2d(position_2d, origin)
                if distance > distance_2d(lower_right_corner[:2], origin):
                    lower_right_corner = position_3d
        self.lower_right_corner = lower_right_corner
        self.upper_left_corner = upper_left_corner
        self.logger.log(f"Right corner: {lower_right_corner}")
        self.logger.log(f"Left corner: {upper_left_corner}")
            
    def generate_distance_path(self):
        self.add_nodes(self.graph)
        self.get_corners(self.graph)
        # Add edges to the graph
        for i in range(0, len(self.indexes), 3):
            i1 = self.indexes[i]
            i2 = self.indexes[i + 1]
            i3 = self.indexes[i + 2]
            self.graph.add_edge(i1, i2, weight=self.distance_cost(i1, i2))
            self.graph.add_edge(i1, i3, weight=self.distance_cost(i1, i3))
            self.graph.add_edge(i2, i3, weight=self.distance_cost(i2, i3))

        self.logger.log(f"Graph nodes: {len(self.graph.nodes)}")
        # Get closest node to start.
        closest_to_start = self.get_closest_node(self.graph, self.get_start())

        # Get closest node to goal.
        closest_to_goal = self.get_closest_node(self.graph, self.get_goal())

        self.logger.log("Graph: Path searching with A*")
        path = nx.astar_path(self.graph, closest_to_start, closest_to_goal, heuristic=self.distance_cost)
        self.logger.log("Graph: Create path object")
        self.distance_path = self.create_path(self.graph, path)

    def create_path(self, graph, path, extruded_shape=False):
        points = []
        points_3D = []
        for node_index in path:
            relative_position = graph.nodes[node_index]['pos']
            absolute_position = sim.multiplyVector(self.heightfield_to_world,relative_position)
            points.extend(absolute_position)
            points.extend((0, 0, 0, 1))
            points_3D.append(relative_position)
        prev_point = points_3D[0]
        distance = 0
        for next_point in points_3D:
            distance += distance_3d(prev_point, next_point)
            prev_point = next_point
        self.logger.log(f'Total distance: {distance}')
        if extruded_shape:
            section=[-0.1, 0, 0.1, 0]
            color=[255, 0, 0]
            shape = sim.generateShapeFromPath(points, section)
            sim.setShapeColor(shape, None, sim.colorcomponent_ambient_diffuse, color)
            
        self.save_path(points)
        return sim.createPath(points, 0, self.subdiv)
    
    def save_path(self, points: list, name: str = "solution_path"):
        filename = self.output_folder_path / f"{name}.csv"
        np_points = np.asarray(points)
        np.savetxt(filename, np_points, delimiter=',')

    def distance_cost(self, node_index_1, node_index_2):
        node_1 = self.graph.nodes[node_index_1]
        node_2 = self.graph.nodes[node_index_2]
        p1 = node_1['pos'][0], node_1['pos'][1]
        p2 = node_2['pos'][0], node_2['pos'][1]
        return distance_2d(p1, p2)
    
    def get_closest_node(self, graph, position):
        node_distances_to_position = np.array(
            [
                distance_2d(graph.nodes[index]['pos'][0:2], position)
                for index in graph.nodes
            ]
        )
        closest = node_distances_to_position.argmin()
        return closest
    
    def get_start(self):
        self.logger.log("Graph: get start")
        reference = sim.multiplyVector(self.heightfield_to_world, self.upper_left_corner)
        start = (
            reference[0] + START_POSITION[0],
            reference[1] - START_POSITION[1],
            0,
        )
        start = sim.multiplyVector(self.world_to_heightfield, start)
        self.logger.log("Start: " + str(start[:2]))
        return start[:2]

    def get_goal(self):
        self.logger.log("Graph: get goal")
        reference = sim.multiplyVector(self.heightfield_to_world, self.lower_right_corner)
        goal = (
            reference[0] - GOAL_POSITION[0],
            reference[1] + GOAL_POSITION[1],
            0,
        )
        goal = sim.multiplyVector(self.world_to_heightfield, goal)
        self.logger.log("Goal: " + str(goal[:2]))
        return goal[:2]
    
    def load_path(self, name:str = "solution_path"):
        self.logger.log("Loading path")
        filename = OUTPUT_FOLDER_PATH / f"{name}.csv"
        points = np.genfromtxt(filename, delimiter=',')
        points = list(points)
        self.logger.log(len(points) / 7)
        return sim.createPath(points, 0, self.subdiv)

    @property
    def heightfield_to_world(self):
        heightfield_handle = sim.getObject('/heightfield')
        return sim.getObjectMatrix(heightfield_handle, -1)
    
    @property
    def world_to_heightfield(self):
        return sim.getMatrixInverse(self.heightfield_to_world)
    
def main():
    controller = AckermanController()
    controller.move(0,0)
    # controller.stop()
    
    global_planner = GlobalPlanner()
    global_planner.generate_distance_path()
    global_planner.load_path()
    
if __name__ == "__main__":
    main()