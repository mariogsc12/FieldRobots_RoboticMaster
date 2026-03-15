from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any, Tuple

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