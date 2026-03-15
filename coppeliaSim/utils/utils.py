from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any, Tuple
from pathlib import Path
import yaml
from dataclasses import dataclass

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
    
    
def parse_config(config: dict) -> tuple["MissionData", dict]:
    mission_dict = config.pop("mission")  
    mission_data = MissionData(start=mission_dict["start_pos"],
                               goal=mission_dict["goal_pos"])
    return mission_data, config