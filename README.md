# stabilize-test
An attempt to develop a orientation stabilization algorithm for bevy_rapier3d

This displays a cube that is actively torqued toward some orientation (quaternion).
After a few seconds, a torque impulse is applied, to test whether the orientation system can then stabilize it back to the target orientation.
