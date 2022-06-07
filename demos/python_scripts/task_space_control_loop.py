import state_representation as sr
import time
from datetime import timedelta
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE


def control_loop_step(pose, ds, dt):
    # get thew twist evaluated at the current pose
    desired_twist = sr.CartesianTwist(ds.evaluate(pose))
    # integrate the twist and add it to the current pose
    new_pose = dt * desired_twist + pose
    # print the new pose
    print(new_pose)
    return new_pose


def control_loop(dt, tolerance):
    # set a desired target and a point attractor DS toward the target
    target = sr.CartesianPose.Random("frame")
    ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
    ds.set_parameter(sr.Parameter("attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE))
    # set a starting pose
    current_pose = sr.CartesianPose.Random("frame")
    # loop until target is reached
    distance = sr.dist(current_pose, target, sr.CartesianStateVariable.POSE)
    while distance > tolerance:
        current_pose = control_loop_step(current_pose, ds, dt)
        distance = sr.dist(current_pose, target, sr.CartesianStateVariable.POSE)
        print(f"Distance to attractor: {distance}")
        print("-----------")
        time.sleep(dt.total_seconds())

    print("##### TARGET #####")
    print(target)
    print("##### CURRENT POSE #####")
    print(current_pose)


def main():
    control_loop(timedelta(milliseconds=10), 1e-3)


if __name__ == "__main__":
    main()
