from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any, Tuple

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