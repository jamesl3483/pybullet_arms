"""Keyboard Input manipulation."""

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


def main(args):
    """Test GUI application."""
    client = BulletClient(pb.GUI)
    client.setGravity(0, 0, -10)

    client.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=-40.0,
        cameraPitch=-40.0,
        cameraTargetPosition=[0.0, 0.0, 0.0]
    )

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

    client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    #Start simulation
    client.setRealTimeSimulation(True)
    #load left hand
    left_hand = HandBody(client, left_hand_model, flags=flags)
    left_position = [-0.3, 0.0, 0.2]  # X,Y,Z
    left_rotation = [0.0, 0.0, 1.0, 1.0]  # Quaternion
    left_hand.set_target(left_position, left_rotation)
    #pause to let the hand load into position
    time.sleep(.2)
    #load right hand

    right_hand = HandBody(client, right_hand_model, flags=flags)
    right_position = [.3, 0.0, 0.2]  # X,Y,Z
    right_rotation = [0.2, 0.0, -1.0, 1.0]  # Quaternion
    right_hand.set_target(right_position, right_rotation)
    client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    time.sleep(.2)
    step = 0.01  # Step size for position adjustments

    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = client.loadURDF("plane.urdf")

    # Cylinder parameters
    cylinder_radius = 0.05
    cylinder_height = 0.25
    cylinder_mass = 0.51  # mass in kg

    # Create a cylinder
    # orientation is provided as a quaternion, here it's the identity quaternion for no rotation
    cylinder_orientation = client.getQuaternionFromEuler([0, 0, 0])
    cylinder_id = client.createCollisionShape(client.GEOM_CYLINDER, radius=cylinder_radius, height=cylinder_height)
    cylinder_visual_id = client.createVisualShape(client.GEOM_CYLINDER, radius=cylinder_radius,
                                             length = cylinder_height, rgbaColor=[1, 0, 0, 1])
    cylinder_position = [0, 0, cylinder_height / 2]  # x, y, z coordinates to place it half above the ground

    # Create a multi-body from the collision and visual shapes
    cylinder_body = client.createMultiBody(baseMass=cylinder_mass, baseCollisionShapeIndex=cylinder_id,
                                      baseVisualShapeIndex=cylinder_visual_id, basePosition=cylinder_position,
                                      baseOrientation=cylinder_orientation)






    # Define a function to continuously check keyboard state
    def monitor_keyboard():
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    # Dictionary to hold the state of each key
    key_state = {
        'up': False, 'down': False, 'left': False, 'right': False,
        'w': False, 'a': False, 's': False, 'd': False, 'f': False, 'q': False,
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

    #######For individual finger contraction#####
    index = 0.0
    middle = 0.0
    pinky = 0.0
    ring = 0.0
    thumb = 0.0
    ################################################

    #####For general finger grabbing######
    grabbing_position = False
    holding = False
    finger_base = 0.0
    finger_middle = 0.0
    finger_tip = 0.0
    ######################################

    angles = [0.0] * 20

    # hand.get_state[0] : tuple of position
    # hand.get_state[1]: tuple of rotation
    # hand.get_state[2]: tuple of finger constraints
    # hand.get_state[3]: tuple of finger position
    # hand.get_state[4]: tuple of finger velocity
    # hand.get_state[5]: tuple of finger torque

    try:
        while client.isConnected():
            #values = [client.readUserDebugParameter(uid) for uid in slider_ids]
            # Update position based on key states
            #Default move mode
            #right hand position
            if key_state['up']:
                right_position[1] += step
            if key_state['down']:
                right_position[1] -= step
            if key_state['left']:
                right_position[0] -= step
            if key_state['right']:
                right_position[0] += step

            #left hand position
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


            #holding the cup
            if key_state['cylinder_grab'] or holding:
                holding = False
                #Move the thumb in the y axis correct position
                # create a slight cupping shape before contracting the base of the fingers
                # angles[16] = 1.5
                # angles[17] = 0.7
                finger_middle = 0.15
                finger_tip = 0.25
                target_angles = [0, finger_base, finger_middle, finger_tip] * 4 + [.9, 1.5, 0, finger_tip]
                right_hand.set_target(right_position, right_rotation, target_angles)
                angles = target_angles
                prev_angle = list(left_hand.get_state()[3])
                grabbing_position = True
                time.sleep(0.1)

                while True:
                    all_angles_stable = True
                    for i, angle in enumerate(left_hand.get_state()[4]):
                        if abs(angle) > 0.01:
                            all_angles_stable = False
                    if all_angles_stable:
                        break

                    time.sleep(0.05)

            if grabbing_position and key_state['top_grab']:
                #Generally move the finger base first then move up to the tip

                # Flags to control the sequence of movement
                # Initial target angles for each segment of the finger
                finger_base = 1.5
                finger_middle = 1.7
                finger_tip = 1.7

                # Assuming these indices correspond to the base joints of each finger
                base_indices = [1, 5, 9, 13, 17]
                tip_indices = [3, 7, 11, 15, 19]

                for idx in range(20):
                    if tip_indices.__contains__(idx):
                        continue
                    target_angles[idx] = finger_tip - idx % 4 * 0.3

                right_hand.set_target(right_position, right_rotation, target_angles)

                while True:
                    all_angles_stable = True
                    current_angles = right_hand.get_state()[3]
                    tip_torque = right_hand.get_state()[5]  # Assuming this stores angle changes akin to velocities

                    for i in tip_indices:
                        if tip_torque[i] > 0.55:
                            target_angles[i] = current_angles[i]        # tip
                            target_angles[i-1] = current_angles[i-1]    # middle
                            target_angles[i-2] = current_angles[i-2]    # base
                            # p.setJointMotorControl2(self.robot.hand_id, i, p.TORQUE_CONTROL, force=torque)
                            right_hand.set_target(right_position, right_rotation, target_angles)
                            angles = target_angles
                        time.sleep(0.2)
                    holding = True

                    # Provide a short delay for the system to stabilize
                    time.sleep(0.05)
                time.sleep(10)



            if key_state['f']: # moving individual fingers
                print('individual finger mode')
                while True:
                    if key_state['q']:
                        break
                    if key_state['1']:
                        index += step * 5
                    elif index > 0:
                        index -= step * 5
                    if key_state['2']:
                        middle += step * 5
                    elif middle > 0:
                        middle -= step * 5
                    if key_state['3']:
                        pinky += step * 5
                    elif pinky > 0:
                        pinky -= step * 5
                    if key_state['4']:
                        ring += step * 5
                    elif ring > 0:
                        ring -= step * 5
                    if key_state['5']:
                        thumb += step * 5
                    elif thumb > 0:
                        thumb -= step * 5


                    angles = [0] + [index] * 3 + [0] + [middle] * 3 + [0] + [pinky] * 3 + [0] + [
                        ring] * 3 + [0] + [thumb] * 3

                    # Update the hand's target position and rotation
                    left_hand.set_target(left_position, left_rotation, angles)
                    right_hand.set_target(right_position, right_rotation, angles)
                    time.sleep(0.1)

            # Small delay to prevent updating too quickly
            #
            # print("position:", right_hand.get_state()[3])
            # print("torque:", right_hand.get_state()[5])
            time.sleep(0.1)
    except pb.error as err:
        if str(err) not in ['Not connected to physics server.', 'Failed to read parameter.']:
            raise


if __name__ == '__main__':
    main(parser.parse_args())
