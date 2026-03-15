from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any, Tuple
from pathlib import Path
import yaml
from dataclasses import dataclass
import argparse
from enum import Enum

class Logger:
    _instance = None 

    def __new__(cls, sim: Any = None):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            cls._instance.sim = sim
        return cls._instance

    def log(self, message: str, verbosity_level: int = 1, to_print: bool = False):
        if self.sim:
            self.sim.addLog(verbosity_level, message)
            if to_print:
                print(message)
        else:
            print(message) 

# global logger
logger: Logger = None

def init_logger(sim: Any):
    global logger
    logger = Logger(sim)
    
def get_client() -> Any:
    print("Connecting to CoppeliaSim...")
    try:
        # Initialize client and get 'sim' object
        client = RemoteAPIClient()
        sim = client.getObject('sim')
        print("Succesfully connected to CoppeliaSim!")
        
        return sim
    except Exception as e:
        raise Exception(f"Impossible to connect to CoppeliaSim: {e}")
    
def get_config(config_path: Path) -> dict:
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)
    
@dataclass
class MissionData:
    start: list
    goal: list
    
class PathType(Enum):
    MIN_ENERGY = 'min_energy'
    MIN_DISTANCE = 'min_distance'
    COMBINED = 'combined'
    
    def __str__(self):
        return self.value
    
class ControllerType(Enum):
    PURE_PURSUIT = 'pure_pursuit'
    POSE_PID = 'pose_pid'
    
    def __str__(self):
        return self.value

    @property
    def color(self):
        if self == ControllerType.PURE_PURSUIT:
            return [1,0,0]
        elif self == ControllerType.POSE_PID:
            return [0,0,1]
    
def parse_config(config: dict) -> tuple["MissionData", dict]:
    mission_dict = config.pop("mission")  
    mission_data = MissionData(start=mission_dict["start_pos"],
                               goal=mission_dict["goal_pos"])
    return mission_data, config

def get_terminal_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Point2point simulation with robotnik in coppeliaSim"
    )
    
    parser.add_argument(
        "--config_path",
        type=Path,
        default=Path(__file__).parent.parent / "config.yaml",
        help="Path to the yaml configuration file"
    )

    parser.add_argument(
        "--path_type",
        type=str,
        choices=[e.value for e in PathType],
        default=PathType.MIN_DISTANCE,
        help="Method used to compute the solution path"
    )

    parser.add_argument(
        "--controller",
        type=str,
        choices=[e.value for e in ControllerType],
        default=ControllerType.PURE_PURSUIT,
        help="Controller used to follow the path"
    )
    
    args = parser.parse_args()
    
    print(f"Starting python script with:" \
          f"\n\t - config path: {args.config_path}" \
          f"\n\t - path type:   {args.path_type}" \
          f"\n\t - controller:  {args.controller}" 
    )
    
    args.path_type = PathType(args.path_type)
    args.controller = ControllerType(args.controller)
    return args 
    