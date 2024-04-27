import bvhio
hierarchy = bvhio.readAsHierarchy(
    "Vicon-sword_attack-0e6d7941-a867-4133-86b9-348542537958.bvh"
)

### get all positionWorld and Rotation
frame = 0
for (joint, index, depth) in hierarchy.loadPose(frame).layout():
    print(f"===================== frame: {frame}")
    print(joint.Name, index, depth)
    print("Parent:",joint.Parent)
    print(joint.Rotation)
    print(joint.RotationWorld)
    print(joint.PositionWorld)
