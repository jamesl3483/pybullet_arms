"""Keyboard Input manipulation."""

import argparse
import numpy as np
import pybullet as pb
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
    left_position = [-0.1, 0.0, 0.0]  # X,Y,Z
    left_rotation = [0.0, 0.0, 1.0, 1.0]  # Quaternion
    left_hand.set_target(left_position, left_rotation)
    #pause to let the hand load into position
    time.sleep(.2)
    #load right hand

    right_hand = HandBody(client, right_hand_model, flags=flags)
    right_position = [.1, 0.0, 0.0]  # X,Y,Z
    right_rotation = [0.0, 0.0, -1.0, 1.0]  # Quaternion
    right_hand.set_target(right_position, right_rotation)
    client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    index = 0.0
    middle = 0.0
    pinky = 0.0
    ring = 0.0
    thumb = 0.0

    step = 0.1  # Step size for position adjustments

    # Define a function to continuously check keyboard state
    def monitor_keyboard():
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    # Dictionary to hold the state of each key
    key_state = {
        'up': False, 'down': False, 'left': False, 'right': False,
        'w': False, 'a': False, 's': False, 'd': False,
        '1': False, '2': False, '3': False, '4': False, '5': False
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
            elif key.char == 'w':
                key_state['w'] = True
            elif key.char == 'a':
                key_state['a'] = True
            elif key.char == 's':
                key_state['s'] = True
            elif key.char == 'd':
                key_state['d'] = True
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
            elif key.char == 'w':
                key_state['w'] = False
            elif key.char == 'a':
                key_state['a'] = False
            elif key.char == 's':
                key_state['s'] = False
            elif key.char == 'd':
                key_state['d'] = False
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

    # for coord in ('X', 'Y', 'Z'):
    #     uid = client.addUserDebugParameter(f'base_{coord}', -0.5, 0.5, 0)
    #     slider_ids.append(uid)
    # for coord in ('R', 'P', 'Y'):
    #     uid = client.addUserDebugParameter(f'base_{coord}', -3.14, 3.14, 0)
    #     slider_ids.append(uid)

    #For the individual joints
    # left_joints = []
    # for i, joint in enumerate(left_hand_model.joints):
    #     left_name = left_hand_model.link_names[i]
    #     #print(left_name)
    #     for axis, (lower, upper) in zip(joint.axes, joint.limits):
    #         #print(f'{left_name}[{axis}]', lower, upper, 0)
    #         uid = client.addUserDebugParameter(f'{left_name}[{axis}]', lower, upper, 0)
    #         #print(uid)
    #         left_joints.append(uid)

    # for i, joint in enumerate(right_hand_model.joints):
    #     right_name = right_hand_model.link_names[i]
    #     for axis, (lower, upper) in zip(joint.axes, joint.limits):
    #         uid = client.addUserDebugParameter(f'{right_name}[{axis}]', lower, upper, 0)
    #         slider_ids.append(uid)

    try:
        while client.isConnected():
            #values = [client.readUserDebugParameter(uid) for uid in slider_ids]
            # Update position based on key states
            if key_state['up']:
                right_position[1] += step
            if key_state['down']:
                right_position[1] -= step
            if key_state['left']:
                right_position[0] -= step
            if key_state['right']:
                right_position[0] += step

            if key_state['w']:
                left_position[1] += step
            if key_state['s']:
                left_position[1] -= step
            if key_state['a']:
                left_position[0] -= step
            if key_state['d']:
                left_position[0] += step

            if key_state['1']:
                print("hi")
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

            # Assume the getQuaternionFromEuler function and values are handled correctly
            #rotation = client.getQuaternionFromEuler(values[3:6])

            #angles = values[6:]
            angles = [0] + [index] * 3 + [0] + [middle] * 3 + [0] + [pinky] * 3 + [0] + [
                ring] * 3 + [0] + [thumb] * 3
            #TODO: Update the joint angles based on the the input

            # Update the hand's target position and rotation
            left_hand.set_target(left_position, left_rotation, angles)
            right_hand.set_target(right_position, right_rotation, angles)

            # Small delay to prevent updating too quickly
            time.sleep(0.1)


    except pb.error as err:
        if str(err) not in ['Not connected to physics server.', 'Failed to read parameter.']:
            raise


if __name__ == '__main__':
    main(parser.parse_args())
