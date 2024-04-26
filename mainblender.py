import bpy

print(bpy.context.object.name)

empty: bpy.types.Object = bpy.data.objects.get("Empty")
skeleton: bpy.types.Object = bpy.data.objects.get("Armature")

print(empty.matrix_world)
print(skeleton.pose.bones["Hips"].matrix)

def get_rest_pose_transforms(object:bpy.types.Object, bone_name:str):
    return object.pose.bones[bone_name].matrix
    

empty.matrix_world = skeleton.pose.bones["Hips"].matrix