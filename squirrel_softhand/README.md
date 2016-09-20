# squirrel_softhand

## Contents

1. <a href="#Usage-instructions">Usage Instructions</a>
	1. <a href="#startup">Start Up</a>
	2. <a href="#usage">Usage</a>


## 1. <a href="Usage-instructions">Usage Instructions</a>

In the following the usage of the squirrel_softhand controller is described.

### 1.1. <a href="startup">Start-up</a>

You can easily start the controller by running 

	roslaunch squirrel_softhand squirrel_softhand.launch

In case, modify the launch file under launch and change the port to the corresponding device string. However, this should not be necessary, as the 
controller tries to find the proper device string on its own.

### 1.2. <a href="usage">Usage</a>

Starting up the controller provides one service for opening and closing the hand which requires a float paramtere in the range between 0 and 1 (inclusive). This essentially specifies how far to close the hand, e.g.,

	rosservice call /softhand_grasp "position: 1.0"	

fully closes the hand, whereas passing 0 opens it fully.