
from coppeliaSim.utils.utils import get_client, Logger, init_logger, get_config, parse_config, MissionData, get_terminal_args
from coppeliaSim.utils.math import distance_2d, distance_3d, normalize_angle
from dataclasses import dataclass
from enum import Enum
import networkx as nx
import numpy as np
import pandas as pd
from pathlib import Path
import os
import math
import argparse
import yaml

sim = get_client()
init_logger(sim)


@dataclass
class RobotnikProperties:
    wheel_radius = 0.235/2
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
    
    def __init__(self, 
                 max_linear_vel,
                 max_angular_vel,
                 wheel_distance=RobotnikProperties.wheel_distance,
                 wheel_radius=RobotnikProperties.wheel_radius):
        
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        
        self.radius = wheel_radius
        self.wheel_distance = wheel_distance

        # Get all wheel handles
        self.wheels = {wheel.name: wheel.get_handle() for wheel in RobotWheels}
        
        self.logger.log(f"Succesfully initialized '{self.__class__.__name__}'")
        self.stop()

    def move(self, linear_velocity: float, angular_velocity: float):
        """
        Compute twist (linear, angular) velocities 
            - linear_velocity in [m/s]
            - angular_velocity in [rad/s]
        """
        linear_velocity = self.limit_velocity(linear_velocity, self.max_linear_vel)
        angular_velocity = self.limit_velocity(angular_velocity, self.max_angular_vel,
)
        
        self.logger.log(f"Moving with twist: ({linear_velocity:.2f}, {angular_velocity:.2f})")
                
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
    
    def limit_velocity(self, command: float, maximum:float):
        if command > maximum or command < -maximum:
            return maximum
        else:
            return command
        
    def stop(self):
        self.logger.log(f"Stopping robot")
        sim.setJointTargetVelocity(self.wheels['front_left'],  0)
        sim.setJointTargetVelocity(self.wheels['back_left'],   0)
        sim.setJointTargetVelocity(self.wheels['front_right'], 0)
        sim.setJointTargetVelocity(self.wheels['back_right'],  0)

class GlobalPlanner:
    """ Global Planner based on the example provided in Aula Global """
    
    logger = Logger()
    
    def __init__(self, mission_data: MissionData, config:dict):
        
        self.config = config
        
        # Get data from config
        self.start_position = mission_data.start
        self.goal_position = mission_data.goal
        
        self.heightfield_folder_path = Path(self.config["heightfield_folder_path"])
        self.solution_folder_path = Path(self.config["solution_folder_path"])
        self.subdiv = self.config["subdiv"]
        
        self.graph = nx.Graph()
        self.upper_left_corner = None
        self.lower_right_corner = None
        
        self.logger.log("Loading data")
        self.load_data()
        
        os.makedirs(self.solution_folder_path, exist_ok=True)
        self.logger.log(f"Output folder check passed: {self.solution_folder_path}")
            

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
            points_3D.append(absolute_position)
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
        filename = self.solution_folder_path / f"{name}.csv"
        np_points = np.asarray(points)
        np.savetxt(filename, np_points, delimiter=',')

    def distance_cost(self, node_index_1, node_index_2):
        node_1 = self.graph.nodes[node_index_1]
        node_2 = self.graph.nodes[node_index_2]
        p1 = node_1['pos'][0], node_1['pos'][1]
        p2 = node_2['pos'][0], node_2['pos'][1]
        return distance_2d(p1, p2)
    
    def get_closest_node(self, graph, position):
        nodes = list(graph.nodes)

        node_distances = [
            distance_2d(graph.nodes[n]['pos'][0:2], position)
            for n in nodes
        ]

        closest_index = np.argmin(node_distances)
        return nodes[closest_index]
    
    def get_start(self):
        return tuple(self.start_position)

    def get_goal(self):
        return tuple(self.goal_position)
    
    def load_path(self, name:str = "solution_path"):
        self.logger.log("Loading path")
        filename = self.solution_folder_path / f"{name}.csv"
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

class LocalPlanner:
    _ROBOTNIK_HEIGTH = 0.416
    
    logger = Logger()

    def __init__(self,                 
                 mission_data: MissionData,
                 path_handle, 
                 local_planner_config:dict,
                 robot_config:dict):
        
        self.start_pos = mission_data.start
        self.goal_pos = mission_data.goal
        
        self.path_handle = path_handle
        self.local_planner_config = local_planner_config
        self.robot_config = robot_config
        
        # Get data from config
        self.threshold_distance = self.local_planner_config["lookahead_distance"]
        
        # Initialize controller
        self.robot_controller = AckermanController(max_linear_vel=robot_config["max_linear_vel"],max_angular_vel=robot_config["max_angular_vel"])
        
        # Get handles for simulation items
        self.robot_handle = sim.getObject('/RobotnikSummitXL')
        self.start_handle = sim.getObject('/Start')
        self.goal_handle = sim.getObject('/Goal')
        
        # Initialize auxiliar variables
        self.double_table = sim.unpackDoubleTable(sim.readCustomDataBlock(path_handle,'PATH'))
        self.position_index = 0
        self.finished = False
        
        # Move markers and robot to defined positions
        path_pos = self.get_position_on_path()
        start_pos = [self.start_pos[0], self.start_pos[1], path_pos[2] + 0.5]  
        start_robot_pos = [self.start_pos[0], self.start_pos[1], path_pos[2]+self._ROBOTNIK_HEIGTH]  
        sim.setObjectPosition(self.robot_handle, -1, start_robot_pos)
        sim.setObjectPosition(self.start_handle, -1, start_pos)

        goal_pos = [self.goal_pos[0], self.goal_pos[1], path_pos[2] +  + 0.5]  
        sim.setObjectPosition(self.goal_handle, -1, goal_pos)
        
        
    def get_position_on_path(self):
        start_index = self.position_index * 7
        end_index = start_index + 7
        # size = len(self.double_table)
        # self.logger.log(f'Size: {size}')
        # self.logger.log(f'Start Index: {start_index}')
        # self.logger.log(f'End Index: {end_index}')
        return self.double_table[start_index:end_index]
       
    def run(self):
        path_pos = self.get_position_on_path()
        m = sim.getObjectMatrix(self.robot_handle, -1)
        m = sim.getMatrixInverse(m)

        path_pos = sim.multiplyVector(m, path_pos)
        error_distance = math.sqrt(
            math.pow(path_pos[0], 2)
            + math.pow(path_pos[1], 2)
            + math.pow(path_pos[2], 2)
        )
        error_angle = normalize_angle(math.atan2(path_pos[1], path_pos[0])) 
        
        linear_velocity =  self.local_planner_config["kp_linear"] * error_distance * math.cos(error_angle)
        angular_velocity = self.local_planner_config["kp_angular"] * error_angle
        
        self.logger.log(f'Error: ({error_distance:.2f}, {math.degrees(error_angle):.2f})')
        
        # Goal reached condition
        if self.position_index * 7 + 7 >= len(self.double_table) and error_distance < self.threshold_distance:
            linear_velocity = angular_velocity = 0
            self.robot_controller.stop()
            self.finished = True
            self.logger.log("Goal reached!")
        else:
            self.robot_controller.move(linear_velocity, angular_velocity)

        while error_distance < self.threshold_distance and self.position_index * 7 + 7 < len(self.double_table):
            self.position_index += 1
            path_pos = self.get_position_on_path()
            m = sim.getObjectMatrix(self.robot_handle, -1)
            m = sim.getMatrixInverse(m)

            path_pos = sim.multiplyVector(m, path_pos)
            error_distance = math.sqrt(
                math.pow(path_pos[0], 2)
                + math.pow(path_pos[1], 2)
                + math.pow(path_pos[2], 2)
            )

    def follow_path(self):
        step = 0.1
        path_pos = self.get_position_on_path()
        point = path_pos[:2]
        if self.time < step + sim.getSimulationTime():
            sim.setObjectPosition(self.robot_handle, -1, point)
            self.time = sim.getSimulationTime()
            if self.index < len(self.double_table) - 7:
                self.index += 7
                self.position_index += 1
                 
def main():
    main_logger = Logger()
    main_logger.log("\n" * 50)
    main_logger.log("===================================================")
    main_logger.log("=================  NEW EXECUTION ==================")
    main_logger.log("===================================================")
    
    args = get_terminal_args()
    config_data = get_config(args.config_path)
    mission_data, config_data = parse_config(config_data)
    
    global_planner = GlobalPlanner(mission_data=mission_data, config=config_data["global_planner"])
    
    global_planner.generate_distance_path()
    path = global_planner.load_path()
    
    local_planner = LocalPlanner(
        mission_data=mission_data,
        path_handle=path,
        local_planner_config=config_data["local_planner"],
        robot_config=config_data["robot"]
    )
    
    try:
        print("Starting follow path mode with local planner... Press Ctrl+C to stop.")
        while not local_planner.finished:
            local_planner.run()
        print("Goal reached. Stopping local planner")
            
    except KeyboardInterrupt:
        print("Script stopped by user")
    
    finally:
        main_logger.log("Simulation finished", to_print=True)
    
if __name__ == "__main__":
    main()