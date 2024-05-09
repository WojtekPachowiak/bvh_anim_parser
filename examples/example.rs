use bvh_anim_parser::parse::{load_bvh_from_file, load_bvh_from_string};
use bvh_anim_parser::types::{Endsite, Joint};
use bvh_anim_parser::visualize::visualize_skeleton;

fn main() {
    ////////////////////////////// loading .bvh ///////////////////////////////////////////
    // load bvh from a file
    let (bvh_metadata, bvh_data) = load_bvh_from_file("./examples/test_anim_sword_attack.bvh");

    // or from a string
    // (`include_str` works at compile time and so has a different base path than `load_bvh_from_file` - ignore the difference)
    let bvh_string: &str = include_str!("./test_anim_sword_attack.bvh");
    let (bvh_metadata, bvh_data) = load_bvh_from_string(bvh_string);


    //////////////////////////////// fields of BvhMetadata ////////////////
    {
        let fps: u32 = bvh_metadata.fps;
        let frame_time: f64 = bvh_metadata.frame_time;
        let num_frames: usize = bvh_metadata.num_frames;
    
        assert_eq!(fps, 120);
        // fps is the reciprocal of frame_time (rounded to nearest integer, though)
        assert_eq!((1.0 / frame_time - fps as f64).round() as i32, 0);
        assert_eq!(num_frames, 598);
    }
    //////////////////////////////// fields of Joint ////////////////
    {
        // joint with index 5 is "Spine3"
        let joint: &Joint = &bvh_metadata.joints[4];
        let name: &String = &joint.name;
        let index: usize = joint.index;
        let parent_index: isize = joint.parent_index;
        let depth: usize = joint.depth;
        let children: &Vec<usize> = &joint.children;
        let is_leaf: bool = joint.is_leaf;
        let endsite: &Option<Endsite> = &joint.endsite;
    
        // fields of endsite
        if let Some(endsite) = endsite {
            // Endsite's offset is just to know the length of the leaf bone.
            // Therefore, it's not stored in BvhData
            let offset: cgmath::Vector3<f64> = endsite.offset;
            panic!("Spine3 is not a leaf joint!");
        }
    
        assert_eq!(name, "Spine3");
        assert_eq!(index, 4);
        assert_eq!(parent_index, 3);
        assert_eq!(depth, 4);
        // children of Spine3 are "LeftShoulder", "RightShoulder", "Neck"
        assert_eq!(children, &vec![5, 7, 14]);
        assert_eq!(is_leaf, false);
        assert_eq!(endsite.is_none(), true);
    }

    
    bvh_metadata.joints.iter().for_each(|joint| {
        // if is leaf then has endsite and no children
        if joint.is_leaf {
            assert!(joint.endsite.is_some());
            assert!(joint.children.is_empty());
        }

        // each joint's parent is 1 level above
        if joint.parent_index != -1 {
            let parent: &Joint = bvh_metadata.find_joint_by_index(joint.parent_index as usize);
            assert_eq!(joint.depth, parent.depth + 1);
        }
    });

    {
        // root has no parent and so its parent index is -1
        let root: &Joint = bvh_metadata.find_joint_by_index(0);
        assert_eq!(root.parent_index, -1);
        // its depth is 0
        assert_eq!(root.depth, 0);
    }

    {
        // get the parent of Head joint
        let neck: &Joint = bvh_metadata.find_joint_by_name("Head");
        let parent: &Joint = bvh_metadata.find_joint_by_index(neck.parent_index as usize);
        assert_eq!(parent.name, "Neck");
    }

    //////////////////////////////// methods of BvhData ////////////////
    // some printing utilities
    bvh_data.print_rest_local();
    bvh_data.print_rest_global();

    //////////////////////////////// fields of BvhData ////////////////
    {
        // rest pose data (size: num_joints) (frame invariant) (edit mode in Blender)
        let rest_local_positions: &Vec<cgmath::Vector3<f64>> = &bvh_data.rest_local_positions;
        let rest_local_rotations: &Vec<cgmath::Quaternion<f64>> = &bvh_data.rest_local_rotations;
        let rest_global_positions: &Vec<cgmath::Vector3<f64>> = &bvh_data.rest_global_positions;
        let rest_global_rotations: &Vec<cgmath::Quaternion<f64>> = &bvh_data.rest_global_rotations;
    
        // pose data (size: num_frames * num_joints) (aka "keyframe" data) (pose mode in Blender)
        let pose_local_positions: &Vec<Vec<cgmath::Vector3<f64>>> = &bvh_data.pose_local_positions;
        let pose_local_rotations: &Vec<Vec<cgmath::Quaternion<f64>>> = &bvh_data.pose_local_rotations;
        let pose_global_rotations: &Vec<Vec<cgmath::Quaternion<f64>>> = &bvh_data.pose_global_rotations;
        let pose_global_positions: &Vec<Vec<cgmath::Vector3<f64>>> = &bvh_data.pose_global_positions;
    
        //////////////////////////////// getting values from BvhData ////////////////
    
        // get global position of hips joint at frame 23
        let hips_index = bvh_metadata.find_joint_by_name("Hips").index;
        let hips_pos: cgmath::Vector3<f64> = pose_global_positions[hips_index][23];
    
        // get global rest pose position of LeftShoulder joint at frame 100
        let left_shoulder_index = bvh_metadata.find_joint_by_name("LeftShoulder").index;
        let left_shoulder_rest_pos: cgmath::Vector3<f64> = rest_global_positions[left_shoulder_index];
        let left_shoulder_rest_rot: cgmath::Quaternion<f64> =
            rest_global_rotations[left_shoulder_index];
    
        // destructre position
        let cgmath::Vector3 { x, y, z } = left_shoulder_rest_pos;
    
        // destructre quaternion (s=w, v=(x, y, z)
        let cgmath::Quaternion { s, v } = left_shoulder_rest_rot;
        let cgmath::Vector3 { x, y, z } = v;
    }

    //////////////////////////////// visualize skeleton ////////////////
    // with "visualize" feature enabled you can visualize the skeleton in a bevy app
    // (use scale when your skeleton is in different units than meters, e.g. centimeters)
    visualize_skeleton(bvh_data, bvh_metadata, 0.01);
}
