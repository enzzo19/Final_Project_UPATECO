"""Controller to drive spuck to detect a box and stop."""
from controller import Robot

def run_robot(robot):
    time_step = 32
    max_speed = 4

    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Enable distance sensors (front: ps0 and ps7)
    front_left_sensor = robot.getDevice('ps7')
    front_left_sensor.enable(time_step)
    front_right_sensor = robot.getDevice('ps0')
    front_right_sensor.enable(time_step)

     # Enable ir sensors
    left_ir = robot.getDevice('ir1')
    left_ir.enable(time_step)
    right_ir = robot.getDevice('ir0')
    right_ir.enable(time_step)

    # Step simulation
    while robot.step(time_step) != -1:
        # Read distance sensor values
        distance_left = front_left_sensor.getValue()
        distance_right = front_right_sensor.getValue()
        print(f"Distancia izquierda (ps7): {distance_left:.3f}, Distancia derecha (ps0): {distance_right:.3f}")
        # Read IR sensor values
        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()
        print("left: {} right: {}".format(left_ir_value, right_ir_value))
        
    
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

if __name__ == '__main__':
    my_robot = Robot()
    run_robot(my_robot)