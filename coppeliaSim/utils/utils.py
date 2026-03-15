from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import tracemalloc
import psutil
from typing import Callable, TypeVar, Union, Tuple, Dict, Optional, Any
from typing_extensions import ParamSpec
from pathlib import Path
import yaml
from dataclasses import dataclass
import argparse
from enum import Enum, auto, Flag
import functools

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
    
class Mode(Enum):
    DIRECT = 'direct'
    PLAN = 'plan'
    
    def __str__(self):
        return self.value
    
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
        help="Method used to compute the solution path (default = min_distance)"
    )

    parser.add_argument(
        "--controller",
        type=str,
        choices=[e.value for e in ControllerType],
        default=ControllerType.PURE_PURSUIT,
        help="Controller used to follow the path (default = pure_pursuit)"
    )

    parser.add_argument(
        "--mode",
        type=str,
        choices=[e.value for e in Mode],
        default=Mode.DIRECT,
        help="Methodology used (default = plan)"
    )
    
    args = parser.parse_args()
    
    print(f"Starting python script with:" \
          f"\n\t - mode:        {args.mode}" \
          f"\n\t - config path: {args.config_path}" \
          f"\n\t - controller:  {args.controller}" \
          f"\n\t - path type:   {args.path_type}" if args.mode == Mode.PLAN else "" 
    )
    
    args.path_type = PathType(args.path_type)
    args.controller = ControllerType(args.controller)
    args.mode = Mode(args.mode)
    
    return args 
    
    

# ======================================
#       Performance Analysis
# ======================================

P = ParamSpec("P")  # Function parameters
T = TypeVar("T")  # Function return type


class Measurements(Flag):
    TIME = auto()
    MEMORY = auto()


def _bytes_to_mb(b: Union[float, int]) -> float:
    return b / 10**6


def _percent(part: Union[float, int], total: Union[float, int]) -> float:
    return 0.0 if total == 0 else (part / total) * 100.0


def _collect_measurments(
    func: Callable[P, T],
    *args: P.args,
    **kwargs: P.kwargs,
) -> Tuple[T, Dict[Measurements, Union[float, int]]]:
    """Execute a received function and measure time and memory used"""
    values = {}

    start = time.perf_counter()
    tracemalloc.start()

    try:
        result = func(*args, **kwargs)

        values[Measurements.TIME] = time.perf_counter() - start
        _, peak = tracemalloc.get_traced_memory()
        values[Measurements.MEMORY] = peak

        return result, values
    finally:
        tracemalloc.stop()


def log_performance_analysis(
    log_func: Optional[Callable[[str], None]] = print,
    prefix: Optional[str] = None,
    file_path: Path = None,
) -> Callable[[Callable[P, T]], Callable[P, T]]:
    """Decorator to display the log execution time and used memory"""
    
    def _decorator(func: Callable[P, T]) -> Callable[P, T]:
        @functools.wraps(func)
        def _wrapper(*args: P.args, **kwargs: P.kwargs) -> T:
            result, metrics = _collect_measurments(func, *args, **kwargs)

            elapsed_s = metrics[Measurements.TIME]
            peak_bytes = metrics[Measurements.MEMORY]
            total_bytes = psutil.virtual_memory().total

            minutes = int(elapsed_s // 60)
            seconds = elapsed_s % 60

            log_message = "%s%dm %.1fs (%.5fs) with max memory usage of %.5f MB (%.5f%%)" % (
                prefix or "",
                minutes,
                seconds,
                elapsed_s,
                _bytes_to_mb(peak_bytes),
                _percent(peak_bytes, total_bytes),
            )

            if log_func:
                log_func(log_message)
            
            if file_path:
                path_obj = Path(file_path)
                
                path_obj.parent.mkdir(parents=True, exist_ok=True)
                
                with open(path_obj, "a", encoding="utf-8") as f:
                    f.write(log_message + "\n")

            return result

        return _wrapper

    return _decorator