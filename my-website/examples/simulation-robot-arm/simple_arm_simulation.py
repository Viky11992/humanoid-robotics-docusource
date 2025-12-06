import pybullet as p
import pybullet_data
import time

def run_simple_arm_simulation():
    """
    Runs a simple 2D robot arm simulation using PyBullet.
    The arm has two links and two revolute joints.
    """
    # 1. Initialize PyBullet physics engine
    physicsClient = p.connect(p.GUI) # p.GUI for graphical interface, p.DIRECT for non-graphical
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Optionally add data path for built-in models
    p.setGravity(0, 0, -9.8) # Set gravity

    # 2. Create a plane for the robot to rest on
    planeId = p.loadURDF("plane.urdf")

    # 3. Define the robot arm (two links, two revolute joints)
    # Using a simple box representation for links
    link_mass = 0.5
    link_half_extents = [0.5, 0.05, 0.05] # Length, width, height

    # Base link (fixed to the world)
    base_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=link_half_extents, rgbaColor=[0.8, 0.2, 0.2, 1])
    base_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=link_half_extents)
    base_link_orientation = p.getQuaternionFromEuler([0, 0, 0])
    base_position = [0, 0, 0.5] # Slightly above the plane
    base_joint_info = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=base_collision_shape_id,
                                        baseVisualShapeIndex=base_visual_shape_id,
                                        basePosition=base_position, baseOrientation=base_link_orientation)

    # Link 1
    link1_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=link_half_extents, rgbaColor=[0.2, 0.8, 0.2, 1])
    link1_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=link_half_extents)

    # Link 2
    link2_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=link_half_extents, rgbaColor=[0.2, 0.2, 0.8, 1])
    link2_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=link_half_extents)

    # Create the multi-body robot arm
    # Define joint frames and parent/child links
    # Joint 0 connects base to link1
    # Joint 1 connects link1 to link2

    # Position of joint 0 relative to base_position
    joint0_parent_frame_pos = [link_half_extents[0], 0, 0] # At the end of the base link
    joint0_child_frame_pos = [-link_half_extents[0], 0, 0] # At the start of link 1

    # Position of joint 1 relative to link 1
    joint1_parent_frame_pos = [link_half_extents[0], 0, 0] # At the end of link 1
    joint1_child_frame_pos = [-link_half_extents[0], 0, 0] # At the start of link 2

    # Create the arm with two links and two revolute joints
    armId = p.createMultiBody(
        baseMass=0, # Base is fixed, so mass is 0
        baseCollisionShapeIndex=-1, # No base collision shape for this setup
        baseVisualShapeIndex=-1, # No base visual shape for this setup
        basePosition=[0, 0, 0.5], # Start position of the base (where the fixed joint connects to the world)
        baseOrientation=[0, 0, 0, 1],
        linkMasses=[link_mass, link_mass],
        linkCollisionShapeIndices=[link1_collision_shape_id, link2_collision_shape_id],
        linkVisualShapeIndices=[link1_visual_shape_id, link2_visual_shape_id],
        linkPositions=[
            [base_position[0] + link_half_extents[0]*2, base_position[1], base_position[2]], # Link1 position relative to base
            [joint1_parent_frame_pos[0] + link_half_extents[0]*2, 0, 0] # Link2 position relative to link1
        ],
        linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
        linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
        linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
        jointType=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
        jointAxis=[[0, 0, 1], [0, 0, 1]], # Rotate around Z-axis
        parentFramePositions=[joint0_parent_frame_pos, joint1_parent_frame_pos],
        childFramePositions=[joint0_child_frame_pos, joint1_child_frame_pos],
        parentLinkIndices=[-1, 0] # Joint 0 connects to base (-1), Joint 1 connects to link 0
    )


    # 4. Control the robot (set joint positions)
    # Define target joint angles in radians
    target_angles = [[math.radians(45), math.radians(-90)],
                     [math.radians(90), math.radians(0)],
                     [math.radians(0), math.radians(0)]]

    for angles in target_angles:
        p.setJointMotorControlArray(
            bodyUniqueId=armId,
            jointIndices=[0, 1],
            controlMode=p.POSITION_CONTROL,
            targetPositions=angles,
            forces=[500, 500] # Max force to apply
        )
        for _ in range(240): # Run for a few steps
            p.stepSimulation()
            time.sleep(1/240.0) # Real-time simulation

    # 5. Keep simulation running until closed by user
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1/240.0)

    p.disconnect() # Disconnect from the physics server

if __name__ == "__main__":
    run_simple_arm_simulation()