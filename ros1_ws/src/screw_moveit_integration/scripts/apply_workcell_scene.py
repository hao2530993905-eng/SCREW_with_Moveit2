#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


BOXES = [
    # Fixed robot-base supports. Keep their upper faces below base_link so the
    # immobile UR base is not reported as colliding with its own workbench.
    ("Box_0", [0.20, 0.20, 0.02], [-0.04, -0.04, -0.02]),
    ("Box_1", [1.00, 1.00, 0.60], [0.30, -0.40, -0.32]),
    ("Box_2", [0.30, 0.30, 0.08], [0.00, -0.32, 0.00]),
    ("Box_3", [0.03, 0.03, 1.00], [0.47, 0.11, 0.48]),
    ("Box_4", [0.30, 0.30, 0.08], [0.37, -0.33, 0.00]),
]


def identity_pose(x=0.0, y=0.0, z=0.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def box(name, dimensions, position):
    collision = CollisionObject()
    collision.header.frame_id = "base_link"
    collision.id = name
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = dimensions
    collision.primitives = [primitive]
    collision.primitive_poses = [identity_pose(*position)]
    collision.operation = CollisionObject.ADD
    return collision


def screwdriver():
    attached = AttachedCollisionObject()
    attached.link_name = "tool0"
    # The adapter, screwdriver body and bit are one collision assembly. Only
    # its abstract attachment frame is exempt; contact with every physical UR3
    # link, including flange and wrist_3_link, must invalidate a plan.
    attached.touch_links = ["tool0"]
    attached.object.header.frame_id = "tool0"
    attached.object.id = "screwdriver_tool"
    cylinders = [
        (0.046, 0.0369, 0.0230),
        (0.045, 0.0238, 0.0685),
        (0.125, 0.0130, 0.1535),
    ]
    for height, radius, center_z in cylinders:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]
        attached.object.primitives.append(primitive)
        attached.object.primitive_poses.append(
            identity_pose(0.0, 0.0, center_z)
        )
    attached.object.operation = CollisionObject.ADD
    return attached


def main():
    rospy.init_node("apply_saved_workcell_scene")
    rospy.wait_for_service("/apply_planning_scene", timeout=30.0)
    apply_scene = rospy.ServiceProxy(
        "/apply_planning_scene",
        ApplyPlanningScene,
    )
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [
        box(name, dimensions, position)
        for name, dimensions, position in BOXES
    ]
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [screwdriver()]
    response = apply_scene(scene)
    if not response.success:
        raise RuntimeError("MoveIt rejected the saved planning scene")
    rospy.loginfo(
        "Loaded 5 workcell obstacles and attached screwdriver collision model"
    )


if __name__ == "__main__":
    main()
