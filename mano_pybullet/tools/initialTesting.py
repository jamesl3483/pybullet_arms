"""Keyboard Input manipulation."""

import argparse
import numpy as np
import pybullet as pb
import pybullet_data
from pybullet_utils.bullet_client import BulletClient
from pynput import keyboard
import threading
import time
import cv2

from ..hand_body import HandBody
from ..hand_model import HandModel20, HandModel45
from ..mano_model import ManoModel

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


# hand.get_state[0] : tuple of position
# hand.get_state[1]: tuple of rotation
# hand.get_state[2]: tuple of finger constraints
# hand.get_state[3]: tuple of finger position
# hand.get_state[4]: tuple of finger velocity
# hand.get_state[5]: tuple of finger torque

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
    finger_base = 0.0
    finger_middle = 0.15
    finger_tip = 0.25
    # target_angles = [0.0, finger_base, finger_middle, finger_tip] * 4 + [.9, 1.5, 0, finger_tip]
    # target_angles[8] = 0.2
    left_starting_angles = [0] * 20
    angles = left_starting_angles[:]
    left_hand.set_target(left_position, left_rotation, left_starting_angles)

    #pause to let the hand load into position
    time.sleep(.2)

    #load right hand
    right_hand = HandBody(client, right_hand_model, flags=flags)
    right_position = [.3, 0.0, 0.2]  # X,Y,Z
    right_rotation = client.getQuaternionFromEuler([0.2, 0.3, -2.0])   # Quaternion
    finger_middle = 0.15
    finger_tip = 0.25
    right_starting_angles = [0.0, finger_base, finger_middle, finger_tip] * 4 + [1.0, 1.7, 0, finger_tip]
    right_starting_angles[8] = -0.2
    angles = [0] * 20
    right_hand.set_target(right_position, right_rotation, right_starting_angles)
    client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    time.sleep(.2)

    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = client.loadURDF("plane.urdf")


    ###### When Creating objects make sure to put them in a list of ids                            #############
    ###### This is for the grabbing mechanism which will see which objects it is most connected to #############

    list_of_objects = []
    # Cylinder parameters
    cylinder_radius = 0.04
    cylinder_height = 0.25
    cylinder_mass = 2 # mass in kg

    # Set the additional search path to the directory containing the URDF file
    pb.setAdditionalSearchPath("mano_pybullet/models")

    # Load flask and lid URDFs
    flask_position = [0, 0, 0.1]
    lid_position = [0, 0, 0.15]  # Adjust based on the actual size of the flask and lid
    flask_orientation = client.getQuaternionFromEuler([0, 0, 0])
    lid_orientation = client.getQuaternionFromEuler([0, 0, 0])
    #
    # flask_id = client.loadURDF("flask.urdf", flask_position, flask_orientation)
    # lid_id = client.loadURDF("lid.urdf", lid_position, lid_orientation)
    # client.changeDynamics(flask_id, -1, mass=2, lateralFriction=2.0, spinningFriction=1.0,
    #                       localInertiaDiagonal=[0.015, 0.015, 0.015])

    # Load box URDF
    box_position = [0.7, 0, 0.1]  # Adjust based on the desired initial position
    box_position2 = [-0.7, 0, 0.1]  # Adjust based on the desired initial position
    box_orientation = pb.getQuaternionFromEuler([0, 0, 0])  # No rotation

    box_id = pb.loadURDF("box.urdf", box_position, box_orientation)
    list_of_objects.append(box_id)
    box2_id = pb.loadURDF("box.urdf", box_position2, box_orientation)
    list_of_objects.append(box2_id)



    '''# Create a cylinder
    # orientation is provided as a quaternion, here it's the identity quaternion for no rotation
    # cylinder_orientation = client.getQuaternionFromEuler([0, 0, 0])
    # cylinder_id = client.createCollisionShape(client.GEOM_CYLINDER, radius=cylinder_radius, height=cylinder_height)
    # cylinder_visual_id = client.createVisualShape(client.GEOM_CYLINDER, radius=cylinder_radius,
    #                                          length = cylinder_height, rgbaColor=[1, 0, 0, 1])
    # cylinder_position = [0, 0, cylinder_height / 2]  # x, y, z coordinates to place it half above the ground
    #
    # # Create a multi-body from the collision and visual shapes
    # cylinder_body = client.createMultiBody(baseMass=cylinder_mass, baseCollisionShapeIndex=cylinder_id,
    #                                   baseVisualShapeIndex=cylinder_visual_id, basePosition=cylinder_position,
    #                                   baseOrientation=cylinder_orientation)
    #
    # list_of_objects.append(cylinder_body)

    # # Adjust the dynamics of the object
    # client.changeDynamics(cylinder_body, -1, mass=cylinder_mass, lateralFriction=2.0, spinningFriction=1.0,
    #                       localInertiaDiagonal=[0.015, 0.015, 0.015])'''

    # Define a function to continuously check keyboard state
    def monitor_keyboard():
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    # Dictionary to hold the state of each key
    key_state = {
        'up': False, 'down': False, 'left': False, 'right': False,
        'w': False, 'a': False, 's': False, 'd': False, 'f': False, 'q': False,
        '1': False, '2': False, '3': False, '4': False, '5': False,
        'cylinder_grab': False, 'top_grab': False, 'movement_mode': False, 'switch_hand': False
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
            elif key.char == 'z':
                key_state['movement_mode'] = True
            elif key.char == 'p':
                key_state['switch_hand'] = True
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
            elif key.char == 'z':
                key_state['movement_mode'] = False
            elif key.char == 'p':
                key_state['switch_hand'] = False
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


    #For saving state when switching hands
    left_angles = left_starting_angles[:]
    right_angles = right_starting_angles[:]

    ######################################
    mode = 0
    step = 0.01  # Step size for position adjustments
    ROTATION_INCREMENT = 0.05
    is_right = False
    right_constraint_id = None
    left_constraint_id = None



    def create_flask_and_lid(client):
        # Load flask and lid URDFs, assuming they are already defined with appropriate collision and visual shapes
        flask_id = client.loadURDF("flask.urdf", [0, 0, 0.1])
        lid_id = client.loadURDF("lid.urdf", [0, 0, 0.15])

        if flask_id is None or lid_id is None:
            print("Invalid object ID provided.")
            return None

        revolute_joint_id = client.createConstraint(parentBodyUniqueId=flask_id,
                parentLinkIndex=-1,
                childBodyUniqueId=lid_id,
                childLinkIndex=-1,
                jointType=pb.JOINT_FIXED,
                jointAxis=[0.0, 0.0, 1.0], #not used for fixed joint
                parentFramePosition=[0.0, 0.0, 0.1],
                childFramePosition=[0.0, 0.0, -0.01])

        return flask_id, lid_id, revolute_joint_id

    def adjust_lid_height_based_on_revolute_angle(client, flask_id, lid_id, revolute_joint_id,
                                                  angle_to_height_ratio):
        # Get the current angle of the revolute joint
        _, revolute_joint_angle, _, _, _, _ = pb.getJointState(flask_id, revolute_joint_id)

        # Calculate the new height of the lid based on the revolute joint's angle
        new_height = revolute_joint_angle * angle_to_height_ratio

        # Adjust the lid's position based on the calculated height
        # Note: This is a simplification. In a real scenario, you might need to consider the lid's original position and orientation.
        client.resetBasePositionAndOrientation(lid_id, [0, 0, new_height], [0, 0, 0, 1])

    def check_and_break_constraint(client, constraint_id, force_threshold):
        # Get the constraint force
        if constraint_id is not None:
            constraint_info = client.getConstraintState(constraint_id) #constraint_info[0:3] give the forces on the joints
            applied_force = constraint_info[0:3]  # This index may vary depending on the PyBullet version
            if sum(applied_force) > force_threshold:
                client.removeConstraint(constraint_id)
                print("Constraint broken!")
                return None
        return constraint_id

    def check_finger_angles(current_angles, finger_base, finger_middle, finger_tip, base_indices, middle_indices,
                            tip_indices):
        # Check if base angles match
        base_match = all(abs(current_angles[idx] - finger_base) < 0.005 for idx in base_indices)
        # Check if middle angles match
        middle_match = all(abs(current_angles[idx] - finger_middle) < 0.005 for idx in middle_indices)
        # Check if tip angles match
        tip_match = all(abs(current_angles[idx] - finger_tip) < 0.005 for idx in tip_indices)

        return base_match and middle_match and tip_match

    # Create a constraint between the hand and the object
    def create_grasp_constraint(client, hand_id, hand_link, object_id, pivotInHand, pivotInObject, ornHand,
                                ornObject):
        constraint_id = client.createConstraint(
            parentBodyUniqueId=hand_id,
            parentLinkIndex=hand_link,
            childBodyUniqueId=object_id,
            childLinkIndex=-1,
            jointType=pb.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=pivotInHand,
            childFramePosition=pivotInObject,
            parentFrameOrientation=ornHand,
            childFrameOrientation=ornObject
        )

        # Set ERP and CFM for better stability
        client.changeConstraint(constraint_id, erp=0.2, maxForce=500)
        return constraint_id

    # Check if we have contact and create a constraint
    def check_contact(client, hand_id, body_ids):
        for body_id in body_ids:
            contacts = client.getContactPoints(hand_id, body_id)
            for contact in contacts:
                if contact[8] < 0.01:  # Check if contact distance is less than 1cm
                    return body_id, contacts
        return None, None

    def curl_fingers_until_contact(hand, position, rotation, starting_angles, is_right):
        finger_base = 1.0
        finger_middle = 0.7
        finger_tip = 0.5

        base_indices = [1, 5, 9, 13]
        middle_indices = [2, 6, 10, 14, 18]
        tip_indices = [3, 7, 11, 15, 19]

        target_angles = starting_angles[:]

        while True:
            # Set initial target angles for the fingers
            if is_right:
                for idx in range(20):
                    if idx in base_indices:
                        target_angles[idx] = min(target_angles[idx] + 0.1, finger_base)
                    elif idx in middle_indices:
                        target_angles[idx] = min(target_angles[idx] + 0.1, finger_middle)
                    elif idx in tip_indices:
                        target_angles[idx] = min(target_angles[idx] + 0.1, finger_tip)
            else:
                for idx in range(20):
                    if idx in base_indices:
                        target_angles[idx] = 0.0
                    elif idx in middle_indices:
                        target_angles[idx] = max(target_angles[idx] + 0.1, finger_middle)
                    elif idx in tip_indices:
                        target_angles[idx] = max(target_angles[idx] + 0.1, finger_tip)

            hand.set_target(position, rotation, target_angles)

            time.sleep(0.1)  # Adjust delay as necessary

            # Check if angles are stable and contact is made
            current_angles = hand.get_state()[3]
            if check_finger_angles(current_angles, finger_base, finger_middle, finger_tip, base_indices, middle_indices,
                                   tip_indices):
                print("Target finger positions reached. Adjusting simulation behavior accordingly.")
                break;
            tip_torque = hand.get_state()[5]
            contact_made = False

            for i in tip_indices:
                if tip_torque[i] > 0.1 and check_contact(client, hand.body_id, list_of_objects):
                    contact_made = True
                    target_angles[i] = current_angles[i]  # tip
                    target_angles[i - 1] = current_angles[i - 1]  # middle
                    target_angles[i - 2] = current_angles[i - 2]  # base

            if contact_made:
                for idx in tip_indices:
                    if idx >= len(target_angles):
                        continue
                    # Once contact is made, adjust to maintain grip
                    target_angles[idx] = current_angles[idx]  # tip
                    target_angles[idx - 1] = current_angles[idx - 1]  # middle
                    target_angles[idx - 2] = current_angles[idx - 2]  # base

                hand.set_target(position, rotation, target_angles)
                break




            time.sleep(0.1)  # Small delay to prevent rapid updates
        return target_angles

    def translate_hand(position):
        if key_state['up']:
            position[1] += step
        if key_state['down']:
            position[1] -= step
        if key_state['left']:
            position[0] -= step
        if key_state['right']:
            position[0] += step
        if key_state['1']:
            position[2] += step
        if key_state['2']:
            position[2] -= step
        return position

    def rotate_hand(rotation):
        rotation_euler = list(client.getEulerFromQuaternion(rotation))
        if key_state['left']:
            rotation_euler[2] += ROTATION_INCREMENT  # Yaw left
        if key_state['right']:
            rotation_euler[2] -= ROTATION_INCREMENT  # Yaw right
        if key_state['up']:
            rotation_euler[0] += ROTATION_INCREMENT  # Pitch up
        if key_state['down']:
            rotation_euler[0] -= ROTATION_INCREMENT  # Pitch down
        return client.getQuaternionFromEuler(rotation_euler)

    def cameras():
        def capture_image(camera_position, target_position, up_vector, fov, aspect, near, far):
            view_matrix = pb.computeViewMatrix(camera_position, target_position, up_vector)
            projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, near, far)
            width, height, rgbImg, depthImg, segImg = pb.getCameraImage(width=640, height=480,
                                                                        viewMatrix=view_matrix,
                                                                        projectionMatrix=projection_matrix)

            # Convert RGB image for OpenCV
            rgbImg = np.reshape(rgbImg, (height, width, 4))

            # Convert depth image to 8-bit for visualization
            depthImg = np.array(depthImg)
            depthImg = cv2.normalize(depthImg, None, 0, 255, cv2.NORM_MINMAX)
            depthImg = np.uint8(depthImg)

            # Apply color map to segmentation mask for visualization
            segImg = np.array(segImg)
            segImgColor = cv2.applyColorMap(np.uint8(segImg), cv2.COLORMAP_JET)

            return rgbImg, depthImg, segImgColor

        # Define camera parameters
        camera_positions = [
            [1, 1, 1],
            [-1, -1, 1]
        ]
        target_positions = [
            [0, 0, 0],
            [0, 0, 0]
        ]
        up_vectors = [
            [0, 0, 1],
            [0, 0, 1]
        ]
        fov = 60
        aspect = 640 / 480
        near = 0.1
        far = 100

        while True:
            rgbImg, depthImg, segImgColor = capture_image(camera_positions[0], target_positions[0], up_vectors[0], fov,
                                                  aspect, near, far)
            rgbImg1, depthImg1, segImgColor1 = capture_image(camera_positions[1], target_positions[1], up_vectors[1], fov,
                                                     aspect, near, far)
            cv2.imshow('Camera 1', rgbImg)
            cv2.imshow('Depth Image 2', depthImg1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



    flask_id, lid_id, revolute_joint_id = create_flask_and_lid(client)
    #add to the list of objects in the scene
    list_of_objects.append(flask_id)
    list_of_objects.append(lid_id)

    video_thread = threading.Thread(target=cameras)
    video_thread.start()

    try:
        while client.isConnected():




            revolute_joint_id = check_and_break_constraint(client, revolute_joint_id, 100)
            # Movement mode translation or rotation
            ################# MODES ####################
            # 0 = right_translation, 1 = right_grab and 2 degree rotation
            # 2 = left_translation, 3 = left_grab and 2 degree rotation
            # Press 'z' to switch between modes
            if key_state['movement_mode']:
                mode = (mode + 1) % 4
                time.sleep(0.1)
            ########################################

            if mode == 0: #right_translation
                print("right translation")
                is_right = True
                right_position = translate_hand(right_position)
                right_hand.set_target(right_position, right_rotation, right_angles)

            if mode == 1: # right_grabbing and rotation
                print("right grab and rotation")
                right_rotation = rotate_hand(right_rotation)

                if key_state['cylinder_grab']:
                    right_angles = curl_fingers_until_contact(right_hand, right_position, right_rotation, right_angles,
                                                              is_right)

                    # Check if we have contact and create a constraint
                    object, contacts = check_contact(pb, right_hand.body_id, list_of_objects)
                    if contacts:
                        contact = contacts[0]
                        link_index = 0  # Get link index from the base of the hand
                        pivotHand = contact[6]  # Contact point on the hand in its local frame
                        pivot = contact[5]  # Contact point on the object in its local frame

                        # Get world positions and orientations
                        hand_world_position, hand_world_orientation = pb.getLinkState(right_hand.body_id,
                                                                                      link_index)[0:2]
                        object_world_position, object_world_orientation = pb.getBasePositionAndOrientation(
                            object)

                        # Convert world coordinates to local coordinates for the hand
                        parent_frame_position, parent_frame_orientation = pb.invertTransform(hand_world_position,
                                                                                             hand_world_orientation)
                        pivotInHand, _ = pb.multiplyTransforms(
                            parent_frame_position,
                            parent_frame_orientation,
                            pivot,
                            [0, 0, 0, 1]
                        )

                        # Convert world coordinates to local coordinates for the object
                        child_frame_position, child_frame_orientation = pb.invertTransform(object_world_position,
                                                                                           object_world_orientation)
                        pivotInObject, _ = pb.multiplyTransforms(
                            child_frame_position,
                            child_frame_orientation,
                            pivot,
                            [0, 0, 0, 1]
                        )

                        # Create the constraint
                        cid = create_grasp_constraint(pb, right_hand.body_id, link_index,
                                                      object, pivotInHand, pivotInObject,
                                                      parent_frame_orientation, child_frame_orientation)

                        print(f"Constraint created at link index {link_index}")
                        right_constraint_id = cid  # Store the constraint ID for future operations
                    else:
                        print("Nothing in contact, will not curl fingers")

                # If the `q` key is pressed, remove the constraint
                if key_state['q']:
                    if right_constraint_id is not None:
                        pb.removeConstraint(right_constraint_id)
                    right_constraint_id = None  # Reset the constraint ID
                    right_angles = right_starting_angles[:]
                right_hand.set_target(right_position, right_rotation, right_angles)

            if mode == 2: #left_translation
                print("left translation")
                is_right = False
                left_new_position = translate_hand(left_position)
                current_state = left_hand.get_state()[:3]

                # if abs(left_new_position - list(current_state)) > 0.009:
                left_hand.set_target(left_position, left_rotation, left_angles)

            if mode == 3:
                print("left grab and rotation")
                left_rotation = rotate_hand(left_rotation)

                if key_state['cylinder_grab']:
                    left_angles = curl_fingers_until_contact(left_hand, left_position, left_rotation, left_angles, is_right)

                    # Check if we have contact and create a constraint
                    object, contacts = check_contact(pb, left_hand.body_id, list_of_objects)
                    if contacts:
                        contact = contacts[0]
                        link_index = 0 # Get link index from the base of the hand
                        pivotHand = contact[6]  # Contact point on the hand in its local frame
                        pivot = contact[5]  # Contact point on the object in its local frame

                        # Get world positions and orientations
                        hand_world_position, hand_world_orientation = pb.getLinkState(left_hand.body_id,
                                                                                      link_index)[0:2]
                        object_world_position, object_world_orientation = pb.getBasePositionAndOrientation(
                            object)

                        # Convert world coordinates to local coordinates for the hand
                        parent_frame_position, parent_frame_orientation = pb.invertTransform(hand_world_position,
                                                                                             hand_world_orientation)
                        pivotInHand, _ = pb.multiplyTransforms(
                            parent_frame_position,
                            parent_frame_orientation,
                            pivot,
                            [0, 0, 0, 1]
                        )

                        # Convert world coordinates to local coordinates for the object
                        child_frame_position, child_frame_orientation = pb.invertTransform(object_world_position,
                                                                                           object_world_orientation)
                        pivotInObject, _ = pb.multiplyTransforms(
                            child_frame_position,
                            child_frame_orientation,
                            pivot,
                            [0, 0, 0, 1]
                        )

                        # Create the constraint
                        cid = create_grasp_constraint(pb, left_hand.body_id, link_index,
                                                      object, pivotInHand, pivotInObject,
                                                      parent_frame_orientation, child_frame_orientation)

                        print(f"Constraint created at link index {link_index}")
                        left_constraint_id = cid  # Store the constraint ID for future operations


                # If the `q` key is pressed, remove the constraint
                if key_state['q']:
                    pb.removeConstraint(left_constraint_id)
                    print(f"Constraint destroyed at link index {link_index}")
                    left_constraint_id = None  # Reset the constraint ID
                    left_angles = left_starting_angles[:]
                left_hand.set_target(left_position, left_rotation, left_angles)



            time.sleep(0.1)


    except pb.error as err:
        if str(err) not in ['Not connected to physics server.', 'Failed to read parameter.']:
            raise


if __name__ == '__main__':
    main(parser.parse_args())
