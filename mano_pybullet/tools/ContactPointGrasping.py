import argparse
import numpy as np
import pybullet as pb
import pybullet_data
from pybullet_utils.bullet_client import BulletClient
from pynput import keyboard
import threading
import time

from ..hand_body import HandBody
from ..hand_model import HandModel20, HandModel45

parser = argparse.ArgumentParser('GUI debug tool')
parser.add_argument('--dofs', type=int, default=20,
                    help='Number of degrees of freedom (20 or 45)')

parser.add_argument('--left-hand', dest='left_hand', action='store_true',
                    help='Show left hand')
parser.add_argument('--right-hand', dest='left_hand', action='store_false',
                    help='Show right hand')
parser.set_defaults(left_hand=False)

parser.add_argument('--visual-shapes', dest='visual', action='store_true',
                    help='Show visual shapes')
parser.add_argument('--no-visual-shapes', dest='visual', action='store_false',
                    help='Hide visual shapes')
parser.set_defaults(visual=True)

parser.add_argument('--self-collisions', dest='self_collisions', action='store_true',
                    help='Enable self collisions')
parser.add_argument('--no-self-collisions', dest='self_collisions', action='store_false',
                    help='Disable self collisions')
parser.set_defaults(self_collisions=False)


def setup_simulation(client):
    client.setGravity(0, 0, -10)
    client.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=-40.0,
        cameraPitch=-40.0,
        cameraTargetPosition=[0.0, 0.0, 0.0]
    )
    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = client.loadURDF("plane.urdf")


def load_hands(client, args):
    if args.dofs == 20:
        left_hand_model = HandModel20(left_hand=True)
        right_hand_model = HandModel20(left_hand=False)
    elif args.dofs == 45:
        left_hand_model = HandModel45(left_hand=args.left_hand)
        right_hand_model = HandModel45(left_hand=args.left_hand)
    else:
        raise ValueError('Only 20 and 45 DoF models are supported.')

    flags = sum([
        HandBody.FLAG_ENABLE_COLLISION_SHAPES,
        HandBody.FLAG_ENABLE_VISUAL_SHAPES * args.visual,
        HandBody.FLAG_JOINT_LIMITS,
        HandBody.FLAG_DYNAMICS,
        HandBody.FLAG_USE_SELF_COLLISION * args.self_collisions])

    left_hand = HandBody(client, left_hand_model, flags=flags)
    left_hand.set_target([-0.3, 0.0, 0.2], [0.0, 0.0, 1.0, 1.0])
    time.sleep(0.2)
    right_hand = HandBody(client, right_hand_model, flags=flags)
    right_hand.set_target([0.3, 0.0, 0.2], [0.0, 0.0, -1.0, 1.0])

    #time.sleep(.2)
    return left_hand, right_hand


def create_cylinder(client):
    # Cylinder parameters
    cylinder_radius = 0.1
    cylinder_height = 0.25
    cylinder_mass = 0.1  # mass in kg

    cylinder_orientation = client.getQuaternionFromEuler([0, 0, 0])
    cylinder_id = client.createCollisionShape(client.GEOM_CYLINDER, radius=cylinder_radius, height=cylinder_height)
    cylinder_visual_id = client.createVisualShape(client.GEOM_CYLINDER, radius=cylinder_radius,
                                                  length = cylinder_height, rgbaColor=[1, 0, 0, 1])
    cylinder_position = [0, 0, cylinder_height / 2]

    cylinder_body = client.createMultiBody(baseMass=cylinder_mass, baseCollisionShapeIndex=cylinder_id,
                                           baseVisualShapeIndex=cylinder_visual_id, basePosition=cylinder_position,
                                           baseOrientation=cylinder_orientation)
    return cylinder_body


def check_contact(client, hand_id, object_id):
    contacts = client.getContactPoints(hand_id, object_id)
    for contact in contacts:
        if contact[8] < 0.01:  # Check if contact distance is less than 1cm
            return True
    return False



#TODO: IDK why the loading is so much slower but it is compartimentalized

def main(args):
    global constraint_id
    client = BulletClient(pb.GUI)

    # Start simulation
    client.setRealTimeSimulation(True)

    left_hand, right_hand = load_hands(client, args)
    time.sleep(1)
    setup_simulation(client)
    cylinder_body = create_cylinder(client)

    client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    step = 0.1  # Step size for position adjustments

    #Setup input
    # Define a function to continuously check keyboard state
    def monitor_keyboard():
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    # Dictionary to hold the state of each key
    key_state = {
        'up': False, 'down': False, 'left': False, 'right': False,
        'w': False, 'a': False, 's': False, 'd': False, 'z':False, 'f': False, 'q': False,
        '1': False, '2': False, '3': False, '4': False, '5': False,
        'cylinder_grab': False, 'top_grab': False
    }

    def on_press(key):
        try:
            if key == keyboard.Key.up:
                key_state['up'] = True
            elif key == keyboard.Key.down:
                key_state['down'] = True
            elif key == keyboard.Key.left:
                key_state['left'] = True
            elif key == keyboard.Key.right:
                key_state['right'] = True
            elif key.char == 'e':
                key_state['cylinder_grab'] = True
            elif key.char == 'r':
                key_state['top_grab'] = True
            elif key.char == 'w':
                key_state['w'] = True
            elif key.char == 'a':
                key_state['a'] = True
            elif key.char == 's':
                key_state['s'] = True
            elif key.char == 'd':
                key_state['d'] = True
            elif key.char == 'z':
                key_state['z'] = True
            elif key.char == 'f':
                key_state['f'] = True
            elif key.char == 'q':
                key_state['q'] = True
            elif key.char == '1':
                key_state['1'] = True
            elif key.char == '2':
                key_state['2'] = True
            elif key.char == '3':
                key_state['3'] = True
            elif key.char == '4':
                key_state['4'] = True
            elif key.char == '5':
                key_state['5'] = True

        except AttributeError:
            pass

    def on_release(key):
        try:
            if key == keyboard.Key.up:
                key_state['up'] = False
            elif key == keyboard.Key.down:
                key_state['down'] = False
            elif key == keyboard.Key.left:
                key_state['left'] = False
            elif key == keyboard.Key.right:
                key_state['right'] = False
            elif key.char == 'e':
                key_state['cylinder_grab'] = False
            elif key.char == 'r':
                key_state['top_grab'] = False
            elif key.char == 'w':
                key_state['w'] = False
            elif key.char == 'a':
                key_state['a'] = False
            elif key.char == 's':
                key_state['s'] = False
            elif key.char == 'd':
                key_state['d'] = False
            elif key.char == 'z':
                key_state['z'] = False
            elif key.char == 'f':
                key_state['f'] = False
            elif key.char == 'q':
                key_state['q'] = False
            elif key.char == '1':
                key_state['1'] = False
            elif key.char == '2':
                key_state['2'] = False
            elif key.char == '3':
                key_state['3'] = False
            elif key.char == '4':
                key_state['4'] = False
            elif key.char == '5':
                key_state['5'] = False
        except AttributeError:
            pass

        if key == keyboard.Key.esc:
            return False  # Stop listener

    # Start the keyboard monitoring in a separate thread
    thread = threading.Thread(target=monitor_keyboard)
    thread.start()


    left_position = [-0.3, 0.0, 0.2]  # X,Y,Z
    left_rotation = [0.0, 0.0, 1.0, 1.0]  # Quaternion
    right_position = [.3, 0.0, 0.2]  # X,Y,Z
    right_rotation = [0.0, 0.0, -1.0, 1.0]  # Quaternion
    angles = [0.0] * 20
    grasped = False


    # Main simulation loop
    while True:
        client.stepSimulation()
        if key_state['up']:
            right_position[1] += step
        if key_state['down']:
            right_position[1] -= step
        if key_state['left']:
            right_position[0] -= step
        if key_state['right']:
            right_position[0] += step
        if key_state['z']:
            right_position[2] += step
        # left hand position
        if key_state['w']:
            left_position[1] += step
        if key_state['s']:
            left_position[1] -= step
        if key_state['a']:
            left_position[0] -= step
        if key_state['d']:
            left_position[0] += step
        # Update the hand's target position and rotation
        left_hand.set_target(left_position, left_rotation, angles)
        right_hand.set_target(right_position, right_rotation, angles)
        time.sleep(0.1)
        if key_state['cylinder_grab']:
            if not grasped and check_contact(client, right_hand.body_id, cylinder_body):
                print("Contact made with cylinder, grasping now.")
                # Assuming that contact points provide the link index for the hand
                contacts = client.getContactPoints(right_hand.body_id, cylinder_body)
                print(contacts)
                print(contacts)
                if contacts:
                    link_index = contacts[0][3]  # Get link index from the first contact point
                    pivot = contacts[0][5]  # Contact point on the object in its local frame
                    pivotHand = contacts[0][6]  # Contact point on the hand in its local frame
                    # Create a fixed constraint to grab the cylinder
                    constraint_id = client.createConstraint(right_hand.body_id, link_index,
                                                            cylinder_body, -1,
                                                            pb.JOINT_FIXED, [0, 0, 0], pivotHand, pivot)
                    grasped = True
        if key_state['top_grab']:
            # Optionally, simulate some time with the object grasped
            print("Cylinder grasped, continuing simulation.")

            client.removeConstraint(constraint_id)
            constraint_id = None
            grasped = False


        time.sleep(0.01)

    # Continue the simulation for a while to observe
    for _ in range(100):
        client.stepSimulation()
        time.sleep(0.01)

    client.disconnect()


if __name__ == '__main__':
    main(parser.parse_args())
