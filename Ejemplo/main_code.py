#python
'''luaExec
function save_mesh_data(folder_path)
    local indices_file = folder_path .. '/indices.csv'
    local vertices_file = folder_path .. '/vertices.csv'
    local height_field_handle = sim.getObject('../heightfield')
    vertices, indices, normals = sim.getShapeMesh(height_field_handle)
    save_to_csv(vertices, vertices_file)
    save_to_csv(indices, indices_file)
end

function save_to_csv(array, filename)
    sim.addLog(1, #array)
    local file = io.open(filename, "w")
    for i, v in ipairs(array) do
        local line = v .. "\n"
        file:write(line)
    end
    file:close()
end
'''

"""
Xmin = 28
Ymin = 27
Xmax = 55
Ymax = 42
"""
import pandas as pd
import networkx as nx
import numpy as np
import math
from enum import Enum
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

global motion_control
global local_planner
global global_planner

class MobileRobot(Enum):
    PIONEER = 'PIONEER'
    ROBOTNIK = 'ROBOTNIK'

class PathType(Enum):
    MIN_ENERGY = 'MIN_ENERGY'
    MIN_DISTANCE = 'MIN_DISTANCE'
    COMBINED = 'COMBINED'


ROBOT = MobileRobot.ROBOTNIK
PATH_TYPE = PathType.MIN_DISTANCE

if ROBOT == MobileRobot.ROBOTNIK:
    FRICTION = 0.5
elif ROBOT == MobileRobot.PIONEER:
    FRICTION = 0.71

SAVE = True
LOAD_PATH = False
FOLDER_PATH = '/home/mario/REPOS/master/FieldRobots_RoboticMaster'
# START_POSITION = [0, 0]
# GOAL_POSITION = [50, 50]
START_POSITION = [28, 27]
GOAL_POSITION = [55, 42]
GRAVITY = 9.807
SUBDIV = 800
CONSTANT = 10
LINEAR_VELOCITY = 3.5
ANGULAR_VELOCITY = 5


def sysCall_init():
    global local_planner
    global global_planner
    global_planner = GlobalPlanner()
    if SAVE:
        log("Saving data")
        global_planner.save()
    log("Loading data")
    global_planner.load_data()
    # Generate path
    if LOAD_PATH:
        path = global_planner.load_path()
    elif PATH_TYPE == PathType.MIN_DISTANCE:
        log("Generating distance global path")
        global_planner.generate_distance_path()
        # global_planner.generate_energy_path()
        path = global_planner.distance_path
    elif PATH_TYPE == PathType.MIN_ENERGY or PATH_TYPE == PathType.COMBINED:
        log("Generating energy global path")
        global_planner.generate_energy_path()
        # global_planner.generate_distance_path_2()
        path = global_planner.energy_path

    if PathType.COMBINED:
        lookahead_distance = 25
    else:
        lookahead_distance = 1
    local_planner = LocalPlanner(
        path,
        lookahead_distance=lookahead_distance
    )

def sysCall_actuation():
    # motion_control.move(1, 0)
    local_planner.run()
    # local_planner.follow_path()

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass


def distance_3d(pos1, pos2):
    x1, y1, z1 = pos1
    x2, y2, z2 = pos2
    cost = math.sqrt(
        (x1 - x2) * (x1 - x2)
        + (y1 - y2) * (y1 - y2) 
        + (z1 - z2) * (z1 - z2)
    )
    return cost

def distance_2d(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    cost = math.sqrt(
        (x1 - x2) * (x1 - x2)
        + (y1 - y2) * (y1 - y2) 
    )
    return cost


class GlobalPlanner:
    def __init__(self, start_position=START_POSITION, goal_position=GOAL_POSITION, folder_path=FOLDER_PATH):
        self.start_position = start_position
        self.goal_position = goal_position
        self.distance_graph = nx.Graph()
        self.energy_graph = nx.DiGraph()
        self.folder_path = folder_path
        self.energy_path = None
        self.distance_path = None
        self.robot_mass = sim.getShapeMass(sim.getObject('.'))
        self.upper_left_corner = None
        self.lower_right_corner = None
        self.subdiv = SUBDIV

    def get_corners(self, graph):
        log("Graph: Getting corners")
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
        log(f"Right corner: {lower_right_corner}")
        log(f"Left corner: {upper_left_corner}")

    def get_start(self):
        log("Graph: get start")
        reference = sim.multiplyVector(self.heightfield_to_world, self.upper_left_corner)
        start = (
            reference[0] + START_POSITION[0],
            reference[1] - START_POSITION[1],
            0,
        )
        start = sim.multiplyVector(self.world_to_heightfield, start)
        log("Start: " + str(start[:2]))
        return start[:2]

    def get_goal(self):
        log("Graph: get goal")
        reference = sim.multiplyVector(self.heightfield_to_world, self.lower_right_corner)
        goal = (
            reference[0] - GOAL_POSITION[0],
            reference[1] + GOAL_POSITION[1],
            0,
        )
        goal = sim.multiplyVector(self.world_to_heightfield, goal)
        log("Goal: " + str(goal[:2]))
        return goal[:2]

    def create_path_from_points(self, path, extruded_shape=False):
        heightfield_handle = sim.getObject('../heightfield')
        heightfield_matrix = sim.getObjectMatrix(heightfield_handle, -1)
        points = []
        for relative_position in path:
            absolute_position = sim.multiplyVector(heightfield_matrix,relative_position)
            points.extend(absolute_position)
            points.extend((0, 0, 0, 1))
        if extruded_shape:
            section=[-0.1, 0, 0.1, 0]
            color=[255, 0, 0]
            shape = sim.generateShapeFromPath(points, section)
            sim.setShapeColor(shape, None, sim.colorcomponent_ambient_diffuse, color)
        self.save_path(points)
        return sim.createPath(points, 0, self.subdiv)

    @property
    def heightfield_to_world(self):
        heightfield_handle = sim.getObject('../heightfield')
        return sim.getObjectMatrix(heightfield_handle, -1)
    
    @property
    def world_to_heightfield(self):
        return sim.getMatrixInverse(self.heightfield_to_world)

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
        log(f'Total distance: {distance}')
        if extruded_shape:
            section=[-0.1, 0, 0.1, 0]
            color=[255, 0, 0]
            shape = sim.generateShapeFromPath(points, section)
            sim.setShapeColor(shape, None, sim.colorcomponent_ambient_diffuse, color)
        # points = [
        #     -1.29445, 0.075, 0.13736, 0.002, -0.008, 0.001, 1,
        #     2, 0, 0.13736, 0, 0, 0, 1,
        #     2, 2, 0.13736, 0, 0, 0, 1
        # ]
        self.save_path(points)
        return sim.createPath(points, 0, self.subdiv)

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
        point_1 = self.energy_graph.nodes[node_index_1]['pos']
        point_2 = self.energy_graph.nodes[node_index_2]['pos']
        vector_1_to_2 = [
            point_2[0] - point_1[0],
            point_2[1] - point_1[1],
            point_2[2] - point_1[2]
        ]
        vector_length = distance_3d(vector_1_to_2, [0, 0, 0])
        angle_to_z = math.acos(vector_1_to_2[2] / vector_length)
        angle_to_xy_plane = (math.pi/2 - angle_to_z)
        energy = self.robot_mass * GRAVITY * (FRICTION * math.cos(angle_to_xy_plane) + CONSTANT * math.sin(angle_to_xy_plane)) * vector_length
        if energy < 0:
            energy = 0
        return energy

    def distance_cost(self, node_index_1, node_index_2):
        node_1 = self.distance_graph.nodes[node_index_1]
        node_2 = self.distance_graph.nodes[node_index_2]
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

    def load_data(self):
        indexes = pd.read_csv(self.folder_path + 'indices.csv', index_col=False, dtype=np.float64, header=None)
        vertices = pd.read_csv(self.folder_path + 'vertices.csv', index_col=False, dtype=np.float64, header=None)
        self.indexes = indexes.T.to_numpy()[0]
        self.vertices = vertices.T.to_numpy()[0]

    def add_nodes(self, graph):
        log("Graph: Adding nodes")
        for i in range(len(self.vertices) // 3):
            x, y, z = self.vertices[i*3:i*3+3]
            graph.add_node(i, pos=(x, y, z))

    def generate_distance_path_2(self):
        self.add_nodes(self.distance_graph)
        self.get_corners(self.distance_graph)
        # Add edges to the graph
        for i in range(0, len(self.indexes), 3):
            i1 = self.indexes[i]
            i2 = self.indexes[i + 1]
            i3 = self.indexes[i + 2]
            self.distance_graph.add_edge(i1, i2, weight=self.distance_cost(i1, i2))
            self.distance_graph.add_edge(i1, i3, weight=self.distance_cost(i1, i3))
            self.distance_graph.add_edge(i2, i3, weight=self.distance_cost(i2, i3))

        # Get closest node to start.
        closest_to_start = self.get_closest_node(self.distance_graph, self.get_start())

        # Get closest node to goal.
        closest_to_goal = self.get_closest_node(self.distance_graph, self.get_goal())
        
        # Extrapolate direct path
        log("Graph: Sample straight path")
        path = [closest_to_start]
        nodes = self.distance_graph.nodes
        start = nodes[closest_to_start]['pos'][:2]
        goal = nodes[closest_to_goal]['pos'][:2]
        length = distance_2d(start, goal)
        step_size = 1
        direction = [goal[0] - start[0], goal[1] - start[1]]
        scaled_direction = [direction[0] * step_size / length, direction[1] * step_size / length]
        current_point = start
        path = [nodes[closest_to_start]['pos']]
        while distance_2d(goal, current_point) > 2 * step_size:
            current_point = [
                current_point[0] + scaled_direction[0],
                current_point[1] + scaled_direction[1]
            ]
            closest_node = self.get_closest_node(self.distance_graph, current_point)
            z_value = nodes[closest_node]['pos'][2]
            if distance_2d(goal, current_point) > 2 * step_size:
                path.append(current_point + [z_value])
        path.append(nodes[closest_to_goal]['pos'])

        log("Graph: Create path object")
        self.distance_path = self.create_path_from_points(path)  

    def generate_distance_path(self):
        self.add_nodes(self.distance_graph)
        self.get_corners(self.distance_graph)
        # Add edges to the graph
        for i in range(0, len(self.indexes), 3):
            i1 = self.indexes[i]
            i2 = self.indexes[i + 1]
            i3 = self.indexes[i + 2]
            self.distance_graph.add_edge(i1, i2, weight=self.distance_cost(i1, i2))
            self.distance_graph.add_edge(i1, i3, weight=self.distance_cost(i1, i3))
            self.distance_graph.add_edge(i2, i3, weight=self.distance_cost(i2, i3))

        log(f"Graph nodes: {len(self.distance_graph.nodes)}")
        # Get closest node to start.
        closest_to_start = self.get_closest_node(self.distance_graph, self.get_start())

        # Get closest node to goal.
        closest_to_goal = self.get_closest_node(self.distance_graph, self.get_goal())

        log("Graph: Path searching with A*")
        path = nx.astar_path(self.distance_graph, closest_to_start, closest_to_goal, heuristic=self.distance_cost)
        log("Graph: Create path object")
        self.distance_path = self.create_path(self.distance_graph, path)

    def generate_energy_path(self):
        self.add_nodes(self.energy_graph)
        self.get_corners(self.energy_graph)
        # Add edges to the graph
        for i in range(0, len(self.indexes), 3):
            i1 = self.indexes[i]
            i2 = self.indexes[i + 1]
            i3 = self.indexes[i + 2]
            self.energy_graph.add_edge(i1, i2, weight=self.energy_cost(i1, i2))
            self.energy_graph.add_edge(i2, i1, weight=self.energy_cost(i2, i1))
            self.energy_graph.add_edge(i1, i3, weight=self.energy_cost(i1, i3))
            self.energy_graph.add_edge(i3, i1, weight=self.energy_cost(i3, i1))
            self.energy_graph.add_edge(i2, i3, weight=self.energy_cost(i2, i3))
            self.energy_graph.add_edge(i3, i2, weight=self.energy_cost(i3, i2))


        log(f"Graph nodes: {len(self.energy_graph.nodes)}")
        # Get closest node to start.
        closest_to_start = self.get_closest_node(self.energy_graph, self.get_start())

        # Get closest node to goal.
        closest_to_goal = self.get_closest_node(self.energy_graph, self.get_goal())

        log("Graph: Path searching")
        path = nx.dijkstra_path(self.energy_graph, closest_to_start, closest_to_goal)
        log("Graph: Create path object")
        self.energy_path = self.create_path(self.energy_graph, path, extruded_shape=True)

    def save(self):
        object=sim.getScriptInt32Param(sim.handle_self,sim.scriptintparam_objecthandle)
        path=sim.getObjectAlias(object,1)
        # vertices, indices = sim.callScriptFunction('save_mesh_data@'+path, sim.scripttype_childscript, FOLDER_PATH)
        sim.callScriptFunction('save_mesh_data@'+path, sim.scripttype_childscript, FOLDER_PATH)
        # log(f'Vertices: {len(vertices)}')
        # log(f'Indices: {len(indices)}')
        # pd.DataFrame(vertices).to_csv(self.folder_path + "vertices.csv")
        # pd.DataFrame(indices).to_csv(self.folder_path + "indices.csv")
        # vertices, indices, normals = sim.getShapeMesh(height_field_handle)

    def save_path(self, points):
        filename = FOLDER_PATH + PATH_TYPE.value + '.csv'
        np_points = np.asarray(points)
        np.savetxt(filename, np_points, delimiter=',')

    def load_path(self):
        log("Loading path")
        filename = FOLDER_PATH + PATH_TYPE.value + '.csv'
        points = np.genfromtxt(filename, delimiter=',')
        points = list(points)
        log(len(points) / 7)
        return sim.createPath(points, 0, self.subdiv)


class LocalPlanner:
    def __init__(self, path_handle, lookahead_distance=3):
        self.robot_handle = sim.getObject('.')
        self.start_dummy = sim.getObject('../Start')
        self.position_index = 0
        self.motion_control = MotionControl()
        self.path_handle = path_handle
        self.double_table = sim.unpackDoubleTable(sim.readCustomDataBlock(path_handle,'PATH'))
        # self.threshold_distance = 0.01
        # self.goal_threshold = 0.005
        self.threshold_distance = lookahead_distance
        self.goal_threshold = 1

        self.time = sim.getSimulationTime()
        self.index = 0

    def get_position_on_path(self):
        size = len(self.double_table)
        log(f'Size: {size}')
        start_index = self.position_index * 7
        end_index = start_index + 7
        log(f'Start Index: {start_index}')
        log(f'End Index: {end_index}')
        return self.double_table[start_index:end_index]

    def run(self):
        # if self.position_index * 7 + 7 > len(self.double_table):
        #     self.position_index = int(len(self.double_table) / 7 - 1)
        # path_pos = sim.getPositionOnPath(self.path_handle, self.position_index)
        path_pos = self.get_position_on_path()
        sim.setObjectPosition(self.start_dummy, -1, path_pos)
        rob_pos = sim.getObjectPosition(self.robot_handle, -1)
        m = sim.getObjectMatrix(self.robot_handle, -1)
        m = sim.getMatrixInverse(m)
        sim_time = sim.getSimulationTime()

        path_pos = sim.multiplyVector(m, path_pos)
        distance = math.sqrt(
            path_pos[0] * path_pos[0]
            + path_pos[1] * path_pos[1]
            + path_pos[2] * path_pos[2]
        )
        angle = math.atan2(path_pos[0], path_pos[1]) - math.pi/2
        if ROBOT == MobileRobot.PIONEER:
            linear_velocity = 0.2
            angular_velocity = 0.2 * angle
        elif ROBOT == MobileRobot.ROBOTNIK:
            if sim_time < 10.0:
                linear_velocity = 2.0
            else:
                linear_velocity = LINEAR_VELOCITY
            angular_velocity = -1 * ANGULAR_VELOCITY * angle
        log(f'Distance: {distance}')
        log(f'Angle: {math.degrees(angle)}')
        if self.position_index * 7 + 7 == len(self.double_table) and distance < self.goal_threshold:
            linear_velocity = angular_velocity = 0
        self.motion_control.move(linear_velocity, angular_velocity)

        while distance < self.threshold_distance and self.position_index * 7 + 7 < len(self.double_table):
            self.position_index += 1
            path_pos = self.get_position_on_path()
            sim.setObjectPosition(self.start_dummy, -1, path_pos)
            rob_pos = sim.getObjectPosition(self.robot_handle, -1)
            m = sim.getObjectMatrix(self.robot_handle, -1)
            m = sim.getMatrixInverse(m)

            path_pos = sim.multiplyVector(m, path_pos)
            distance = math.sqrt(
                math.pow(path_pos[0], 2)
                + math.pow(path_pos[1], 2)
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

class MotionControl:
    def __init__(self, robot=ROBOT):
        self.robot = robot
        if robot == MobileRobot.PIONEER:
            self.motorLeft = sim.getObject("./leftMotor")
            self.motorRight = sim.getObject("./rightMotor")
            self.radius = 0.195 / 2
            self.wheel_distance = 0.381
        elif robot == MobileRobot.ROBOTNIK:
            self.front_left=sim.getObject('./front_left_wheel')
            self.front_right=sim.getObject('./front_right_wheel')
            self.back_right=sim.getObject('./back_right_wheel')
            self.back_left=sim.getObject('./back_left_wheel')
            self.radius = 0.235/2
            self.wheel_distance = 0.614

    def move(self, linear_velocity, angular_velocity):
        v_right = linear_velocity + self.wheel_distance * angular_velocity
        v_left = linear_velocity - self.wheel_distance * angular_velocity
        angular_right = v_right / self.radius
        angular_left = v_left / self.radius
        if self.robot == MobileRobot.PIONEER:
            sim.setJointTargetVelocity(self.motorLeft,-angular_left)
            sim.setJointTargetVelocity(self.motorRight,-angular_right)
        elif self.robot == MobileRobot.ROBOTNIK:
            sim.setJointTargetVelocity(self.front_left,angular_left)
            sim.setJointTargetVelocity(self.back_left,angular_left)
            sim.setJointTargetVelocity(self.front_right,-angular_right)
            sim.setJointTargetVelocity(self.back_right,-angular_right)


def log(message, verbosity_level=1):
    sim.addLog(verbosity_level, message)