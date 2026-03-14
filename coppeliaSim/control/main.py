
from coppeliaSim.utils.utils import get_client, Logger, init_logger 
from dataclasses import dataclass
from enum import Enum

sim = get_client()
init_logger(sim)

@dataclass
class RobotnikProperties:
    radius = 0.235/2
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
    
    def __init__(self, radius=RobotnikProperties.radius, wheel_distance=RobotnikProperties.wheel_distance):
        self.radius = radius
        self.wheel_distance = wheel_distance

        # Get all wheel handles
        self.wheels = {wheel.name: wheel.get_handle() for wheel in RobotWheels}
        
        self.logger.log(f"Succesfully initialized '{self.__class__.__name__}'")

    def move(self, linear_velocity: float, angular_velocity: float):
        """
        Compute twist (linear, angular) velocities 
            - linear_velocity in [m/s]
            - angular_velocity in [rad/s]
        """
        self.logger.log(f"Moving robot with twist: ({linear_velocity, angular_velocity})")
                
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
        
    def stop(self):
        self.logger.log(f"Stopping robot")
        sim.setJointTargetVelocity(self.wheels['front_left'],  0)
        sim.setJointTargetVelocity(self.wheels['back_left'],   0)
        sim.setJointTargetVelocity(self.wheels['front_right'], 0)
        sim.setJointTargetVelocity(self.wheels['back_right'],  0)
        
def main():
    controller = AckermanController()
    controller.move(0,0.25)
    # controller.stop()

if __name__ == "__main__":
    main()