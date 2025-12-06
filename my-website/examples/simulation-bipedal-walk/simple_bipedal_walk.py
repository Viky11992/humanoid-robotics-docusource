import pybullet as p
import pybullet_data
import time
import math

def create_bipedal_robot():
    """
    Creates a simplified bipedal robot with a torso and two legs (thigh, shank).
    """
    # Torso
    torso_half_extents = [0.1, 0.05, 0.2]
    torso_mass = 2.0
    torso_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=torso_half_extents, rgbaColor=[0.8, 0.2, 0.2, 1])
    torso_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=torso_half_extents)

    # Leg Link (e.g., thigh or shank)
    leg_half_extents = [0.05, 0.05, 0.2]
    leg_mass = 1.0
    leg_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=leg_half_extents, rgbaColor=[0.2, 0.8, 0.2, 1])
    leg_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=leg_half_extents)

    # Define links and joints
    # Link structure: Torso (base) -> Hip (joint) -> Thigh (link) -> Knee (joint) -> Shank (link)
    # This simplified model will only have hips connecting to thighs for two legs directly from the torso.

    # Joint properties
    joint_axis = [0, 1, 0] # Hip rotation around Y-axis (side to side)
    hip_joint_parent_frame_pos = [0, 0, -torso_half_extents[2]] # At bottom of torso
    hip_joint_child_frame_pos = [0, 0, leg_half_extents[2]] # At top of thigh

    # Right leg
    right_leg_pos = [0, -torso_half_extents[1] - leg_half_extents[0], -torso_half_extents[2] - leg_half_extents[2]]

    # Left leg
    left_leg_pos = [0, torso_half_extents[1] + leg_half_extents[0], -torso_half_extents[2] - leg_half_extents[2]]


    # Create the multi-body robot
    robotId = p.createMultiBody(
        baseMass=torso_mass,
        baseCollisionShapeIndex=torso_collision_shape,
        baseVisualShapeIndex=torso_visual_shape,
        basePosition=[0, 0, 1.0], # Start position of the torso
        baseOrientation=[0, 0, 0, 1],
        linkMasses=[leg_mass, leg_mass], # One thigh for each leg
        linkCollisionShapeIndices=[leg_collision_shape, leg_collision_shape],
        linkVisualShapeIndices=[leg_visual_shape, leg_visual_shape],
        linkPositions=[
            [0, -torso_half_extents[1] - leg_half_extents[0]*2, -torso_half_extents[2] - leg_half_extents[2]*2], # Right thigh
            [0, torso_half_extents[1] + leg_half_extents[0]*2, -torso_half_extents[2] - leg_half_extents[2]*2] # Left thigh
        ],
        linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
        linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
        linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
        jointType=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
        jointAxis=[joint_axis, joint_axis],
        parentFramePositions=[hip_joint_parent_frame_pos, hip_joint_parent_frame_pos], # Both hips attach to torso bottom
        childFramePositions=[hip_joint_child_frame_pos, hip_joint_child_frame_pos],
        parentLinkIndices=[-1, -1] # Both hips attach to base (-1)
    )

    # Adjust position for the legs - this is tricky with createMultiBody, might need URDF
    # For a very simple demo, let's simplify and make the links just rotate from a common point
    # Re-creating with a more appropriate base and links structure for legs
    p.removeBody(robotId) # Remove the previous attempt

    # Let's define a simple "stick figure" biped
    lower_leg_len = 0.5
    upper_leg_len = 0.5
    body_mass = 1.0
    leg_mass = 0.5

    # Base: torso
    base_col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.2])
    base_vis_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.2], rgbaColor=[0.8, 0.2, 0.2, 1])

    # Upper leg segment
    upper_leg_col_shape = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.05, height=upper_leg_len)
    upper_leg_vis_shape = p.createVisualShape(p.GEOM_CAPSULE, radius=0.05, height=upper_leg_len, rgbaColor=[0.2, 0.8, 0.2, 1])

    # Lower leg segment
    lower_leg_col_shape = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.05, height=lower_leg_len)
    lower_leg_vis_shape = p.createVisualShape(p.GEOM_CAPSULE, radius=0.05, height=lower_leg_len, rgbaColor=[0.2, 0.2, 0.8, 1])

    # Link list for createMultiBody
    link_masses = [leg_mass, leg_mass, leg_mass, leg_mass] # Right thigh, right shank, left thigh, left shank

    link_collision_shapes = [upper_leg_col_shape, lower_leg_col_shape, upper_leg_col_shape, lower_leg_col_shape]
    link_visual_shapes = [upper_leg_vis_shape, lower_leg_vis_shape, upper_leg_vis_shape, lower_leg_vis_shape]

    # Joint types (all revolute for simplicity)
    joint_types = [p.JOINT_REVOLUTE] * 4
    joint_axes = [[0, 1, 0]] * 4 # All rotate around Y-axis (pitch)

    # Hip positions relative to base
    r_hip_pos = [0, -0.15, -0.2] # Right hip joint relative to torso base
    l_hip_pos = [0, 0.15, -0.2]  # Left hip joint relative to torso base

    # Knee positions relative to parent link
    r_knee_pos = [0, 0, -upper_leg_len/2] # Right knee relative to right thigh
    l_knee_pos = [0, 0, -upper_leg_len/2] # Left knee relative to left thigh

    biped = p.createMultiBody(
        baseMass=body_mass,
        baseCollisionShapeIndex=base_col_shape,
        baseVisualShapeIndex=base_vis_shape,
        basePosition=[0, 0, 1.0], # Start position of the torso
        baseOrientation=[0, 0, 0, 1],
        linkMasses=link_masses,
        linkCollisionShapeIndices=link_collision_shapes,
        linkVisualShapeIndices=link_visual_shapes,
        linkPositions=[
            [r_hip_pos[0], r_hip_pos[1], r_hip_pos[2] - upper_leg_len/2], # Right thigh
            [0, 0, -lower_leg_len/2], # Right shank relative to thigh end
            [l_hip_pos[0], l_hip_pos[1], l_hip_pos[2] - upper_leg_len/2], # Left thigh
            [0, 0, -lower_leg_len/2]  # Left shank relative to thigh end
        ],
        linkOrientations=[
            p.getQuaternionFromEuler([0, 0, 0]),
            p.getQuaternionFromEuler([0, 0, 0]),
            p.getQuaternionFromEuler([0, 0, 0]),
            p.getQuaternionFromEuler([0, 0, 0])
        ],
        linkInertialFramePositions=[[0,0,0]]*4,
        linkInertialFrameOrientations=[[0,0,0,1]]*4,
        jointType=joint_types,
        jointAxis=joint_axes,
        parentFramePositions=[r_hip_pos, r_knee_pos, l_hip_pos, l_knee_pos], # Joint positions in parent frame
        childFramePositions=[[0,0,upper_leg_len/2], [0,0,lower_leg_len/2], [0,0,upper_leg_len/2], [0,0,lower_leg_len/2]], # Link positions in child frame
        parentLinkIndices=[-1, 0, -1, 2] # Right hip to base (-1), Right knee to link 0 (thigh), Left hip to base (-1), Left knee to link 2 (thigh)
    )

    # Enable torque control for all joints
    for joint_index in range(p.getNumJoints(biped)):
        p.setJointMotorControl2(biped, joint_index, p.VELOCITY_CONTROL, force=0) # Disable default motor

    return biped


def run_simple_bipedal_walk_simulation():
    """
    Runs a simple bipedal walking simulation using PyBullet.
    This is a highly simplified demo, not a full walking controller.
    """
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1/240.0) # Simulation timestep

    planeId = p.loadURDF("plane.urdf")
    robotId = create_bipedal_robot()

    # Joint indices: 0=R_Hip, 1=R_Knee, 2=L_Hip, 3=L_Knee
    r_hip_idx, r_knee_idx, l_hip_idx, l_knee_idx = 0, 1, 2, 3

    # Initial stance
    p.setJointMotorControl2(robotId, r_hip_idx, p.POSITION_CONTROL, targetPosition=0, force=500)
    p.setJointMotorControl2(robotId, r_knee_idx, p.POSITION_CONTROL, targetPosition=0, force=500)
    p.setJointMotorControl2(robotId, l_hip_idx, p.POSITION_CONTROL, targetPosition=0, force=500)
    p.setJointMotorControl2(robotId, l_knee_idx, p.POSITION_CONTROL, targetPosition=0, force=500)

    for _ in range(240):
        p.stepSimulation()
        time.sleep(1/240.0)

    # Simple walking gait - swing one leg forward, then the other
    gait_duration = 240 # simulation steps per phase

    # Phase 1: Lift right leg, swing forward
    print("Phase 1: Right leg forward")
    p.setJointMotorControl2(robotId, r_hip_idx, p.POSITION_CONTROL, targetPosition=math.radians(30), force=500)
    p.setJointMotorControl2(robotId, r_knee_idx, p.POSITION_CONTROL, targetPosition=math.radians(30), force=500) # Bend knee
    p.setJointMotorControl2(robotId, l_hip_idx, p.POSITION_CONTROL, targetPosition=math.radians(-10), force=500) # Compensate with left hip

    for _ in range(gait_duration):
        p.stepSimulation()
        time.sleep(1/240.0)

    # Phase 2: Place right leg, lift left leg, swing forward
    print("Phase 2: Left leg forward")
    p.setJointMotorControl2(robotId, r_hip_idx, p.POSITION_CONTROL, targetPosition=math.radians(0), force=500)
    p.setJointMotorControl2(robotId, r_knee_idx, p.POSITION_CONTROL, targetPosition=math.radians(0), force=500)
    p.setJointMotorControl2(robotId, l_hip_idx, p.POSITION_CONTROL, targetPosition=math.radians(30), force=500)
    p.setJointMotorControl2(robotId, l_knee_idx, p.POSITION_CONTROL, targetPosition=math.radians(30), force=500) # Bend knee
    p.setJointMotorControl2(robotId, r_hip_idx, p.POSITION_CONTROL, targetPosition=math.radians(-10), force=500) # Compensate with right hip

    for _ in range(gait_duration):
        p.stepSimulation()
        time.sleep(1/240.0)

    # Phase 3: Return to initial stance (simplified)
    print("Phase 3: Back to stance")
    p.setJointMotorControl2(robotId, r_hip_idx, p.POSITION_CONTROL, targetPosition=math.radians(0), force=500)
    p.setJointMotorControl2(robotId, r_knee_idx, p.POSITION_CONTROL, targetPosition=math.radians(0), force=500)
    p.setJointMotorControl2(robotId, l_hip_idx, p.POSITION_CONTROL, targetPosition=math.radians(0), force=500)
    p.setJointMotorControl2(robotId, l_knee_idx, p.POSITION_CONTROL, targetPosition=math.radians(0), force=500)

    for _ in range(gait_duration):
        p.stepSimulation()
        time.sleep(1/240.0)

    # Keep simulation running until closed by user
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1/240.0)

    p.disconnect()


if __name__ == "__main__":
    run_simple_bipedal_walk_simulation()