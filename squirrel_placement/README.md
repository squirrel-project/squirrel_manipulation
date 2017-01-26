This package provides minimal functionality for placing objects using either the Metahand or the SoftHand. Depending on which you want to use, either do a
- `roslaunch squirrel_placement squirrel_placement_metahand.launch` for the Metahand
and
- `roslaunch squirrel_placement squirrel_placement_softhand.launch` for the SoftHand.

Both of the server provide actions for
- dropping and object
- and placing it at a specific location.

For the former, the respective action servers are called
- `softhand_drop_server` and
- `metahand_drop_server`.
If you just want to drop an object connect to the corresponding server and submit a `DropAction`.

For the latter, the respective action servers are called
- `softhand_place_server` and
- `metahand_place_server`.
If you just want to place an object connect to the corresponding server and submit a `PutDownAction`.

As a side note, in case of the `*place*` servers, the provided pose is not transformed into the proper frame (`origin`) as the server expects the pose to already be in that frame.

For a minimal working example please see `grasp_test.py` under `squirrel_grasping/test`.
