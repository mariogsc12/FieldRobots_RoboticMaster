
from coppeliaSim.utils.utils import get_client, Logger, init_logger, get_config, parse_config, MissionData, get_terminal_args, PathType, ControllerType, Mode, log_performance_analysis
from coppeliaSim.utils.math import distance_2d, distance_3d, normalize_angle
from dataclasses import dataclass
from enum import Enum
import networkx as nx
import numpy as np
import pandas as pd
from pathlib import Path
import os
import math

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
        
class DifferentialController:
    """
    Differential controller
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

class PIDPoseController:
    
    def __init__(self, kp_x, kp_y, kp_theta):
        self.kp_x = kp_x
        self.kp_y = kp_y
        self.kp_theta = kp_theta

    def compute(self, robot_pose, target_pose):

        xr, yr, thetar = robot_pose
        xt, yt = target_pose

        dx = xt - xr
        dy = yt - yr

        # Transform to robot frame
        error_x = math.cos(thetar) * dx + math.sin(thetar) * dy
        error_y = -math.sin(thetar) * dx + math.cos(thetar) * dy

        error_theta = math.atan2(dy, dx) - thetar
        error_theta = normalize_angle(error_theta)

        v = self.kp_x * error_x * math.cos(error_theta)
        omega = self.kp_y * error_y + self.kp_theta * error_theta

        return v, omega
    
class GlobalPlanner:
    """ Global Planner based on the example provided in Aula Global """
    FRICTION = 0.5
    GRAVITY = 9.81
    CONSTANT = 10
    
    logger = Logger()
    
    def __init__(self, mission_data: MissionData, config:dict, mode:Mode):
        
        self.mode = mode
        self.config = config
        
        # Get data from config
        self.start_position = mission_data.start
        self.goal_position = mission_data.goal
        
        self.heightfield_folder_path = Path(self.config["heightfield_folder_path"])
        self.executed_folder_path = Path(self.config["executed_folder_path"])
        self.subdiv = self.config["subdiv"]
        
        self.graph = nx.Graph()
        self.upper_left_corner = None
        self.lower_right_corner = None
        self.robot_mass = sim.getShapeMass(sim.getObject('/RobotnikSummitXL'))
        
        self.logger.log("Loading data")
        self.load_data()
        
        os.makedirs(self.executed_folder_path, exist_ok=True)
        self.logger.log(f"Output folder check passed: {self.executed_folder_path}")
            
    def load_data(self):
        indexes = pd.read_csv(self.heightfield_folder_path / 'indices.csv', index_col=False, dtype=np.float64, header=None)
        vertices = pd.read_csv(self.heightfield_folder_path / 'vertices.csv', index_col=False, dtype=np.float64, header=None)
    
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
    
    def generate_direct_path(self):
        if not self.graph.nodes:
            self.add_nodes(self.graph)

        start = [float(v) for v in self.start_position]
        goal = [float(v) for v in self.goal_position]
        
        n_points = 20
        points = []
        points_3D = []
        
        for i in range(n_points + 1):
            t = i / n_points
            x = start[0] * (1 - t) + goal[0] * t
            y = start[1] * (1 - t) + goal[1] * t
            
            # Extract the closest node to start position
            closest_node = self.get_closest_node(self.graph, (x, y))
            
            # Get z value of this node and add small margin
            z = self.graph.nodes[closest_node]['pos'][2] + 0.3
            
            # Transform to world coordinates
            absolute_position = sim.multiplyVector(self.heightfield_to_world, [x, y, z])
            
            points_3D.append(absolute_position)
            points.extend(absolute_position)
            
            # Orientation (quaterniong)
            points.extend([0.0, 0.0, 0.0, 1.0]) 

        self.logger.log(f"Creating direct path with {len(points_3D)} points", to_print=True)
        
        # Save the solution path
        self.save_path(points, subfolder="")
        self.desired_path = self.draw_path(points)
        
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
        self.desired_path = self.create_path(self.graph, path, subfolder="min_distance")
        
    def generate_energy_path(self):
        self.add_nodes(self.graph)
        self.get_corners(self.graph)
        # Add edges to the graph
        for i in range(0, len(self.indexes), 3):
            i1 = self.indexes[i]
            i2 = self.indexes[i + 1]
            i3 = self.indexes[i + 2]
            self.graph.add_edge(i1, i2, weight=self.energy_cost(i1, i2))
            self.graph.add_edge(i2, i1, weight=self.energy_cost(i2, i1))
            self.graph.add_edge(i1, i3, weight=self.energy_cost(i1, i3))
            self.graph.add_edge(i3, i1, weight=self.energy_cost(i3, i1))
            self.graph.add_edge(i2, i3, weight=self.energy_cost(i2, i3))
            self.graph.add_edge(i3, i2, weight=self.energy_cost(i3, i2))


        self.logger.log(f"Graph nodes: {len(self.graph.nodes)}")
        # Get closest node to start.
        closest_to_start = self.get_closest_node(self.graph, self.get_start())

        # Get closest node to goal.
        closest_to_goal = self.get_closest_node(self.graph, self.get_goal())

        self.logger.log("Graph: Path searching")
        path = nx.dijkstra_path(self.graph, closest_to_start, closest_to_goal)
        self.logger.log("Graph: Create path object")
        self.desired_path = self.create_path(self.graph, path, subfolder="min_energy")

    def create_path(self, graph, path, subfolder, extruded_shape=False):
        points = []
        points_3D = []
        
        for node_index in path:
            relative_position = graph.nodes[node_index]['pos']
            absolute_position = sim.multiplyVector(self.heightfield_to_world,relative_position)
            points.extend(absolute_position)
            points.extend((0, 0, 0, 1))
            points_3D.append(absolute_position)
            
        goal_x = self.goal_position[0]
        goal_y = self.goal_position[1]
        
        # Get the z value of the last node
        last_node_index = path[-1]
        goal_z = graph.nodes[last_node_index]['pos'][2]
        
        exact_goal_relative = [float(goal_x), float(goal_y), float(goal_z)]
        exact_goal_absolute = sim.multiplyVector(self.heightfield_to_world, exact_goal_relative)
        
        # Add goal to the solution
        points.extend(exact_goal_absolute)
        points.extend((0, 0, 0, 1)) 
        points_3D.append(exact_goal_absolute)
        
        prev_point = points_3D[0]
        distance = 0
        for next_point in points_3D:
            distance += distance_3d(prev_point, next_point)
            prev_point = next_point
        self.logger.log(f'Total distance: {distance}')
            
        self.save_path(points,subfolder)
        
        return self.draw_path(points)
    
    def save_path(self, points: list, subfolder:str="distance", name: str = "solution_path"):
        folder_path = Path(self.executed_folder_path) / str(self.mode) / subfolder 
        
        folder_path.mkdir(parents=True, exist_ok=True)
        filename = folder_path / f"{name}.csv"
    
        # Get x,y,z
        xyz_points = [points[i:i+3] for i in range(0, len(points), 7)]
        
        # Save with pandas
        df = pd.DataFrame(xyz_points, columns=["x", "y", "z"])
        df.to_csv(filename, index=False)
        
        self.logger.log(f"Solution path saved to {filename}", to_print=True)

    def distance_cost(self, node_index_1, node_index_2):
        node_1 = self.graph.nodes[node_index_1]
        node_2 = self.graph.nodes[node_index_2]
        p1 = node_1['pos'][0], node_1['pos'][1]
        p2 = node_2['pos'][0], node_2['pos'][1]
        return distance_2d(p1, p2)

    def energy_cost(self, node_index_1, node_index_2):
        """
        We need the angle respect to the XY plane.
        So we calculate the angle against the z-axis and then substract
        this 90 - calculated_angle to get the angle respect to the XY plane.

        The formula we use for energy is:
        E = mg(u cos(?) + sin(?))*l
        where:
        - m is the mass of the robot
        - g is the gravity
        - ? is the of the vector respect to the XY plane
        - l is the vector's length.
        """
        point_1 = self.graph.nodes[node_index_1]['pos']
        point_2 = self.graph.nodes[node_index_2]['pos']
        vector_1_to_2 = [
            point_2[0] - point_1[0],
            point_2[1] - point_1[1],
            point_2[2] - point_1[2]
        ]
        vector_length = distance_3d(vector_1_to_2, [0, 0, 0])
        angle_to_z = math.acos(vector_1_to_2[2] / vector_length)
        angle_to_xy_plane = (math.pi/2 - angle_to_z)
        energy = self.robot_mass * self.GRAVITY * (self.FRICTION * math.cos(angle_to_xy_plane) + self.CONSTANT * math.sin(angle_to_xy_plane)) * vector_length
        if energy < 0:
            energy = 0
        return energy

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
    
    def draw_path(self, points):
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
                 robot_config:dict,
                 mode: Mode,
                 controller: ControllerType):
        
        self.start_pos = mission_data.start
        self.goal_pos = mission_data.goal
        
        self.path_handle = path_handle
        self.local_planner_config = local_planner_config
        self.robot_config = robot_config
        self.mode = mode
        self.controller = controller
        
        # Get data from config
        self.threshold_distance = self.local_planner_config["lookahead_distance"]
        
        # Initialize controller
        self.robot_controller = DifferentialController(max_linear_vel=robot_config["max_linear_vel"],max_angular_vel=robot_config["max_angular_vel"])
        self.pose_pid_controller = PIDPoseController(kp_x=self.local_planner_config["pose_pid"]["kp_x"],
                                                kp_y=self.local_planner_config["pose_pid"]["kp_y"], 
                                                kp_theta=self.local_planner_config["pose_pid"]["kp_theta"])
        
        # Get handles for simulation items
        self.robot_handle = sim.getObject('/RobotnikSummitXL')
        self.start_handle = sim.getObject('/Start')
        self.goal_handle = sim.getObject('/Goal')
        
        # Initialize auxiliar variables
        self.double_table = sim.unpackDoubleTable(sim.readCustomDataBlock(path_handle,'PATH'))
        self.position_index = 0
        self.finished = False
        self.executed_path = []
        self.executed_twist = []
        
        # Move markers and robot to defined positions
        path_pos = self.get_position_on_path()
        start_pos = [self.start_pos[0], self.start_pos[1], path_pos[2] + 0.5]  
        start_robot_pos = [self.start_pos[0], self.start_pos[1], path_pos[2]+self._ROBOTNIK_HEIGTH]  
        sim.setObjectPosition(self.robot_handle, -1, start_robot_pos)
        sim.setObjectPosition(self.start_handle, -1, start_pos)

        goal_pos = [self.goal_pos[0], self.goal_pos[1], path_pos[2] + 0.5]  
        sim.setObjectPosition(self.goal_handle, -1, goal_pos)
        
        # Robot trajectory path
        self.traj_drawing = sim.addDrawingObject(
            sim.drawing_linestrip,
            3,          # thickness
            0,
            -1,
            10000,
            self.controller.color
        )
        
        
    def get_position_on_path(self):
        start_index = self.position_index * 7
        end_index = start_index + 7
        # size = len(self.double_table)
        # self.logger.log(f'Size: {size}')
        # self.logger.log(f'Start Index: {start_index}')
        # self.logger.log(f'End Index: {end_index}')
        return self.double_table[start_index:end_index]
    
    def pure_pusuit_controller(self, error_distance: float, error_angle:float) -> tuple[float, float]:
        linear_velocity =  self.local_planner_config["pure_pursuit"]["kp_linear"] * error_distance * math.cos(error_angle)
        angular_velocity = self.local_planner_config["pure_pursuit"]["kp_angular"] * error_angle
        return linear_velocity, angular_velocity
        
    def pid_pose_controller(self, path_pos) -> tuple[float, float]:
        robot_pos = sim.getObjectPosition(self.robot_handle, -1)
        robot_orient = sim.getObjectOrientation(self.robot_handle, -1)

        robot_pose = (
            robot_pos[0],
            robot_pos[1],
            robot_orient[2]
        )

        target_pose = (
            path_pos[0],
            path_pos[1]
        )
        
        linear_velocity, angular_velocity = self.pose_pid_controller.compute(robot_pose, target_pose)
        return linear_velocity, angular_velocity

    def condition_check(self, error_distance: float) -> bool:
        if self.position_index * 7 + 7 >= len(self.double_table) and error_distance < self.threshold_distance:
            self.robot_controller.stop()
            self.finished = True
            self.logger.log("Goal reached!")
            
            return True
        else:
            return False
        
    def controller_manager(self, error_distance:float, error_angle:float):
        if self.controller == ControllerType.PURE_PURSUIT:
            linear_velocity, angular_velocity = self.pure_pusuit_controller(error_distance, error_angle)
        elif self.controller == ControllerType.POSE_PID:
            world_target = self.get_position_on_path()
            linear_velocity, angular_velocity = self.pid_pose_controller(world_target)
        else:
            raise ValueError(f"Unknown controller type: {self.controller}")
            
        self.robot_controller.move(linear_velocity, angular_velocity)
        self.record_robot_data(linear_velocity, angular_velocity)
        
            
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
        self.logger.log(f'Error: ({error_distance:.2f}, {math.degrees(error_angle):.2f})')
        
        # Goal reached condition
        if not self.condition_check(error_distance):
            self.controller_manager(error_distance, error_angle)
            
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
                
    def record_robot_data(self,linear_velocity:float, angular_velocity:float):

        pos = sim.getObjectPosition(self.robot_handle, -1)

        # save
        self.executed_path.append(pos)
        self.executed_twist.append([linear_velocity, angular_velocity])

        # draw in CoppeliaSim
        sim.addDrawingObjectItem(
            self.traj_drawing,
            [pos[0], pos[1], pos[2]]
        )

    def save_executed_path(self, path_type: str = "", folder="solutions"):
        path = path_type if self.mode is Mode.PLAN else ""
        full_folder = os.path.join(folder, str(self.mode), str(path), str(self.controller))
        os.makedirs(full_folder, exist_ok=True)
        filename = os.path.join(full_folder, "executed_path.csv")

        df = pd.DataFrame(
            self.executed_path,
            columns=["x", "y", "z"]
        )

        df.to_csv(filename, index=False)

        self.logger.log(f"Executed path saved to {filename}", to_print=True)
        
    def save_executed_twist(self, path_type: str = "", folder="solutions"):
        path = path_type if self.mode is Mode.PLAN else ""
        
        full_folder = os.path.join(folder, str(self.mode), str(path), str(self.controller))
        os.makedirs(full_folder, exist_ok=True)
        filename = os.path.join(full_folder, "executed_twist.csv")

        df = pd.DataFrame(
            self.executed_twist,
            columns=["v", "w"]
        )

        df.to_csv(filename, index=False)

        self.logger.log(f"Executed twist saved to {filename}", to_print=True)
           
def main():
    main_logger = Logger()
    main_logger.log("\n" * 50)
    main_logger.log("===================================================")
    main_logger.log("=================  NEW EXECUTION ==================")
    main_logger.log("===================================================")
    
    args = get_terminal_args()
    config_data = get_config(args.config_path)
    mission_data, config_data = parse_config(config_data)
    
    path_str = args.path_type if args.mode is Mode.PLAN else ""
    base_folder = config_data["global_planner"]["executed_folder_path"]
    
    full_folder = os.path.join(base_folder, str(args.mode), str(path_str), str(args.controller))
    metrics_file = os.path.join(full_folder, "performance_metrics.txt")
    
    @log_performance_analysis(
        log_func=print, 
        prefix="Execution finished in: ", 
        file_path=metrics_file
    ) 
    def run_simulation():
        controller = args.controller
        global_planner = GlobalPlanner(mission_data=mission_data, config=config_data["global_planner"], mode=args.mode)
        
        if args.mode == Mode.DIRECT:
            global_planner.generate_direct_path()
                
        elif args.mode == Mode.PLAN:
            if args.path_type == PathType.MIN_DISTANCE:
                global_planner.generate_distance_path()
            elif args.path_type == PathType.MIN_ENERGY:
                global_planner.generate_energy_path()
        else:
            raise ValueError(f"Unknown mode: {args.mode}")
        
        local_planner = LocalPlanner(
            mode = args.mode,
            controller = controller,
            mission_data=mission_data,
            path_handle=global_planner.desired_path,
            local_planner_config=config_data["local_planner"],
            robot_config=config_data["robot"]
        )
        
        try:
            print("Starting follow path mode with local planner... Press Ctrl+C to stop.")
            while not local_planner.finished:
                local_planner.run()
            print("Goal reached. Stopping local planner")
                
        except KeyboardInterrupt:
            main_logger.log("Script stopped by user", to_print=True)
        
        local_planner.save_executed_path(
            path_type=args.path_type,
            folder=config_data["global_planner"]["executed_folder_path"]
        )
        
        local_planner.save_executed_twist(
            path_type=args.path_type,
            folder=config_data["global_planner"]["executed_folder_path"]
        )
        
        main_logger.log("Simulation succesfully finished", to_print=True)

    run_simulation()

if __name__ == "__main__":
    main()