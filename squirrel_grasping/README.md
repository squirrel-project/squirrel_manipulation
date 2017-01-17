This package provides necessary functionality for blind top-grasping of an object given its pose.
Depending on which you want to use, either do a
- `roslaunch squirrel_grasping squirrel_grasping_metahand.launch` for the KCL hand
and
- `roslaunch squirrel_grasping squirrel_grasping_softhand.launch` for the SoftHand.

In both cases, this will start an action server you can readily use by connecting to it. For the KCL hand, this server is called `metahand_grasp_server` and for the SoftHand `softhand_grasp_server`. Both require a `BlindGraspAction` as a request.

As a side note, the provided pose can be in any frame as (for both hands) the server internally transforms it properly.
