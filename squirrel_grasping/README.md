This package provides necessary functionality for blind top-grasping of an object given its pose.
Depending on which hand you are using, either do a
- `roslaunch squirrel_grasping squirrel_grasping_metahand.launch` for the Metahand
and
- `roslaunch squirrel_grasping squirrel_grasping_softhand.launch` for the SoftHand.

In both cases, this will start an action server you can readily use by connecting to it. For the Metahand, this server is called `metahand_grasp_server` and for the SoftHand `softhand_grasp_server`. Both require a `BlindGraspAction` as a request. Observe that the `BCylinder` (`squirrel_object_perception_msgs`) element of the action needs to be filled correctly, as the action server internally uses the bounding box height to compute the final gripper pose.

As a side note, the provided pose can be in any frame as (for both hands) the server internally transforms it properly.

For a minimal working example please see `grasp_test.py` under `test`
