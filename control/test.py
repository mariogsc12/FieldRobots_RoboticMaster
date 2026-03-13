import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("Conectando a CoppeliaSim remoto...")
try:
    # Iniciar cliente y obtener el objeto 'sim'
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    print("¡Conectado a CoppeliaSim remoto!")
except Exception as e:
    raise Exception(f"No se pudo conectar al simulador: {e}")

# Nombres de los motores. 
# Nota: Si tu robot tiene otro nombre base, la ruta podría ser '/TuRobot/front_left_wheel'
motor_names = ['/front_left_wheel', '/front_right_wheel',
               '/back_right_wheel', '/back_left_wheel']
motor_handles = []

print("Obteniendo handles...")
for name in motor_names:
    try:
        # En la nueva API, si no encuentra el objeto lanza una excepción
        handle = sim.getObject(name) 
        motor_handles.append(handle)
    except Exception as e:
        raise Exception(f"No se pudo obtener el handle de '{name}'. Verifica el árbol de escena. Error exacto: {e}")

print("Handles obtenidos:", motor_handles)

# Función para mover motores
def set_motor_velocity(velocities):
    for handle, vel in zip(motor_handles, velocities):
        # La nueva API es directa: solo handle y velocidad
        sim.setJointTargetVelocity(handle, vel)

print("Moviendo motores (vel=5)...")
# Ejemplo: mover todos los motores a velocidad 5
set_motor_velocity([5, -5, -5, 5])
time.sleep(2)

print("Deteniendo motores...")
# Detener motores
set_motor_velocity([0, 0, 0, 0])

print("Programa finalizado correctamente.")