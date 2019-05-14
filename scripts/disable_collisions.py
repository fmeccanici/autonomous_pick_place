import rospy
import moveit_commander

pubPlanningScene = rospy.Publisher('planning_scene', moveit_commander.PlanningSceneInterface

rospy.wait_for_service('/get_planning_scene', 10.0)
get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
response = get_planning_scene(request)

acm = response.scene.allowed_collision_matrix
if not 'gripper_finger_tip_left_joint' in acm.default_entry_names:
    # add button to allowed collision matrix
    acm.default_entry_names += ['gripper_finger_tip_left_joint']
    acm.default_entry_values += [True]

    planning_scene_diff = PlanningScene(
            is_diff=True,
            allowed_collision_matrix=acm)

    self._pubPlanningScene.publish(planning_scene_diff)
    rospy.sleep(1.0)

