from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("Connecting to CoppeliaSim...")
try:
    # Initialize client and get 'sim' object
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    print("Succesfully connected to CoppeliaSim!")
except Exception as e:
    raise Exception(f"Impossible to connect to CoppeliaSim: {e}")