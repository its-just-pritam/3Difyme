import bpy
from math import acos
import numpy as np
from collections import OrderedDict
from mathutils import Vector, Matrix


# roll 2 bones with half of the angle between 2 given vectors
def boneRoll(bone1_name: str, vector1: np.mat, vector2: np.mat, direction: int, axis: str) -> None:
    unit_vector1 = vector1 / np.linalg.norm(vector1)
    unit_vector2 = vector2 / np.linalg.norm(vector2)
    dot_product = np.dot(unit_vector1, unit_vector2)
    angle = np.arccos(dot_product)

    bpy.context.scene.transform_orientation_slots[0].type = 'NORMAL'
    ob = bpy.context.active_object
    bone = ob.pose.bones[bone1_name]
    bone.rotation_mode = 'XYZ'
    bone.rotation_euler.rotate_axis(axis, direction * angle)
    

# making pose from the given vertices
def armaturePose(vertices: np.mat) -> None:
    
    print([x.name for x in bpy.context.scene.objects])

    ARMS_LEGS_FOOT = OrderedDict({
                'mixamorig:LeftShoulder': (12, 11), 'mixamorig:RightShoulder': (11, 12),
                'mixamorig:LeftArm': (11, 13), 'mixamorig:RightArm': (12, 14),
                'mixamorig:LeftForeArm': (13, 15), 'mixamorig:RightForeArm': (14, 16),
#                'mixamorig:LeftHand': (15, 19), 'mixamorig:RightHand': (16, 20),
                'mixamorig:LeftUpLeg' : (23, 25), 'mixamorig:RightUpLeg': (24, 26),
                'mixamorig:LeftLeg': (25, 27), 'mixamorig:RightLeg': (26, 28), 
                'mixamorig:LeftFoot': (27, 31), 'mixamorig:RightFoot': (28, 32),
                'mixamorig:LeftToeBase': (29, 31), 'mixamorig:RightToeBase': (30, 32),
                'mixamorig:LeftToe_End': (29, 31), 'mixamorig:RightToe_End': (30, 32)
                })

    SPINE = OrderedDict({
            'mixamorig:Hips': (23, 24, 11, 12), 'mixamorig:Spine': (23, 24, 11, 12),
            'mixamorig:Spine1': (23, 24, 11, 12), 'mixamorig:Spine2': (23, 24, 11, 12)
                })


    # bones to add roll to
    ROLL = {}
    updated_pos_bones = {}
    
    ob = bpy.context.active_object

    act_arm = bpy.context.object
    bpy.ops.object.mode_set(mode='EDIT')
    
    md_face_vector = (vertices[3] + vertices[6])/2 - (vertices[7] + vertices[8])/2
    
    right_vector = np.array(ob.pose.bones['mixamorig:RightShoulder'].head - ob.pose.bones['mixamorig:Head'].head)
    left_vector = np.array(ob.pose.bones['mixamorig:LeftShoulder'].head - ob.pose.bones['mixamorig:Head'].head)
    
    face_normal_vector = np.cross(right_vector, left_vector)
    cross_1 = np.cross(md_face_vector, face_normal_vector)
    print('///////////', cross_1)
    
    if cross_1[2] < 0:
        boneRoll('mixamorig:Neck', face_normal_vector, md_face_vector, 1, 'Y')
    else:
        boneRoll('mixamorig:Neck', face_normal_vector, md_face_vector, -1, 'Y')
    if cross_1[0] > 0:
        boneRoll('mixamorig:Head', face_normal_vector, md_face_vector, -1, 'X')
    else:
        boneRoll('mixamorig:Head', face_normal_vector, md_face_vector, 1, 'X')
    
    '''
    SPINE
    '''
    for (bone, index) in (SPINE).items():
        
        print("-"*80)
        act_arm.data.edit_bones[bone].use_inherit_rotation = False
        myBone = ob.pose.bones[bone]
        print(bone)
        print('Head: ', myBone.head)
        print('Tail: ', myBone.tail)
        
        point_1 = (vertices[index[0]] + vertices[index[1]])/2
        point_2 = (vertices[index[2]] + vertices[index[3]])/2
        mp_bone_vector = point_2 - point_1
        mp_bone_unit_vector = mp_bone_vector / np.linalg.norm(mp_bone_vector)
                        
        rotation = myBone.vector.rotation_difference(mp_bone_unit_vector)
        print(rotation)
        M = (
        Matrix.Translation(myBone.head) @
        rotation.to_matrix().to_4x4() @
        Matrix.Translation(-myBone.head)
        )
        print('New Tail: ', myBone.tail)
        myBone.matrix = M @ myBone.matrix  
    
    '''
    LIMBS
    '''
    for (bone, index) in (ARMS_LEGS_FOOT).items():
        
        print("-"*80)
        act_arm.data.edit_bones[bone].use_inherit_rotation = False
        myBone = ob.pose.bones[bone]
        print(bone)
        print('Head: ', myBone.head)
        print('Tail: ', myBone.tail)
        
        mp_bone_vector = vertices[index[1]] - vertices[index[0]]
        mp_bone_unit_vector = mp_bone_vector / np.linalg.norm(mp_bone_vector)   

        rotation = myBone.vector.rotation_difference((mp_bone_unit_vector[0], mp_bone_unit_vector[1], mp_bone_unit_vector[2]))
        print(rotation)
        M = (
        Matrix.Translation(myBone.head) @
        rotation.to_matrix().to_4x4() @
        Matrix.Translation(-myBone.head)
        )
        myBone.matrix = M @ myBone.matrix
        
        
# wrist roll (remember the order of vectors in cross product)
    md_left_wrist_index = vertices[19] - vertices[15]
    md_left_wrist_pinky = vertices[17] - vertices[15]
    left_wrist_index = np.array((ob.pose.bones['mixamorig:LeftHandIndex4'].tail - ob.pose.bones['mixamorig:LeftHand'].tail))
    left_wrist_pinky = np.array((ob.pose.bones['mixamorig:LeftHandPinky4'].tail - ob.pose.bones['mixamorig:LeftHand'].tail))
    left_hand_normal_vector = np.cross(left_wrist_pinky, left_wrist_index)
    md_left_hand_normal_vector = np.cross(md_left_wrist_pinky, md_left_wrist_index)
    boneRoll('mixamorig:LeftHand', left_hand_normal_vector, md_left_hand_normal_vector, -1/2, 'Y')
    boneRoll('mixamorig:LeftForeArm', left_hand_normal_vector, md_left_hand_normal_vector, -1/2, 'Y')

    md_right_wrist_index = vertices[20] - vertices[16]
    md_right_wrist_pinky = vertices[18] - vertices[16]
    right_wrist_index = np.array((ob.pose.bones['mixamorig:RightHandIndex4'].tail - ob.pose.bones['mixamorig:RightHand'].tail))
    right_wrist_pinky = np.array((ob.pose.bones['mixamorig:RightHandPinky4'].tail - ob.pose.bones['mixamorig:RightHand'].tail))
    right_hand_normal_vector = np.cross(right_wrist_index, right_wrist_pinky)
    md_right_hand_normal_vector = np.cross(md_right_wrist_index, md_right_wrist_pinky)
    boneRoll('mixamorig:RightHand', right_hand_normal_vector, md_right_hand_normal_vector, 1/2, 'Y')
    boneRoll('mixamorig:RightForeArm', right_hand_normal_vector, md_right_hand_normal_vector, 1/2, 'Y')
    
    bpy.ops.object.mode_set(mode='OBJECT')
    

# driver code
if __name__ == "__main__":
    
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False, confirm=False)
    # Import the source object and shift it to x=-1
    src_FBXfilePath = 'C:/Users/srita/Desktop/3dify_task/rupak/temp.obj'
    src_obj = bpy.ops.import_scene.obj( filepath = src_FBXfilePath, split_mode='OFF' )
    selected_obj = bpy.context.selected_objects
    for obj in selected_obj:
        obj.scale[0] = 0.03
        obj.scale[1] = 0.03
        obj.scale[2] = 0.03
        obj.location[0] = -1
    
    # source.fbx
    src_OBJfilePath = 'C:/Users/srita/Desktop/3dify_task/rupak/avatar_rigged.fbx'
    src_obj = bpy.ops.import_scene.fbx( filepath = src_OBJfilePath )
    selected_obj = bpy.context.active_object
    import_path = 'C:\\Users\\srita\\Desktop\\3dify_task\\rupak\\temp.obj'
    vertices = np.zeros([33, 3])
    i = 0
    try:
        with open(import_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if line[0] == 'v':
                    _, x, y, z = list(line.split(' '))
#                    print(x, y, z)
#                    print()
                    vertices[i][0], vertices[i][1], vertices[i][2] = x, y, z
                    i = i+1
    
    except:
        print('file not found')
    armaturePose(vertices)