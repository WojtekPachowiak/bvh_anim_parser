use cgmath::{InnerSpace, One, Rad, Rotation3};
use regex::Regex;
use std::fs::File;
use std::io::{self, BufRead};
use crate::types::*;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/// Returns the kinematic chain of a bvh like \[\[0,1,2,3\],\[4,5,6,7,8\],\[9,10,11\],\[12,13,14,15\],\[16,17,18\]\]
/// Usually the chains are: left leg, right leg, left arm, right arm and spine+head.
pub fn get_kinematic_chains(bvh : &BvhMetadata) -> Vec<Vec<Index>> {
    let mut kinematic_chains: Vec<Vec<Index>> = Vec::new();
    let mut chain: Vec<Index> = Vec::new();
    let mut last_depth: isize = -1;
    for joint in bvh.joints.iter() {
        if last_depth != joint.depth as isize -1  {
            kinematic_chains.push(chain.clone());
            chain.clear();
        }
        last_depth = joint.depth as isize;
        chain.push(joint.index);
    }
    kinematic_chains.push(chain.clone());
    kinematic_chains
}


/// Used during joint creation to fill in its parent index (after parent index has been assigned, this function becomes redundant).
/// Alogirthm: searches for the joint with depth 1 less than the current joint's depth.
fn find_parent_joint_index_by_depth(joint_depth:Depth, joint_index:ParentIndex, joints: &Vec<Joint>) -> ParentIndex {
    // detect root joint
    if joints.is_empty() {
        return -1;
    }
    let mut i = joint_index-1;
    while i >= 0 {
        if joints[i as Index].depth == joint_depth - 1 {
            return i as isize;
        }
        i -= 1;
    }
    panic!("BUG: Parent joint not found. Report this bug.");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
/// Get the tail offset of a joint (i.e the vector pointing from joint's head to its tail (in rest pose)).
/// It's used to calculate joint's rest pose rotation.
fn get_tail_offset(joint: &Joint, offsets: &Vec<Position>) -> Position {
    let num_children = joint.children.len();

    if num_children == 1 {
        // return the offset of the only child
        return offsets[joint.children[0] as Index];
    } else if num_children > 1 {
        // return the average of all children's offsets (i.e the average tail offset);
        // This is very important - it lets us calculate the rotation of hips and the highest spine joint.
        return joint
            .children
            .iter()
            .map(|&child_index| offsets[child_index])
            .sum::<Position>()
            / num_children as f64;
    } else if joint.endsite.is_some() {
        // return an arbitrary vector (e.g. pointing upwards) if joint has no children (i.e. it's an endsite joint) - it doesn't matter
        return joint.endsite.as_ref().unwrap().offset;
    } else {
        panic!("Joint has no children and is not a leaf (i.e. has no endsites) joint.");
        // return an arbitrary vector (e.g. pointing upwards) if joint has no children (i.e. it's an endsite joint) - it doesn't matter    
    }
}

/// Calculate the global rest pose of a joint.
fn calc_global_rest_pose(bvh: &BvhMetadata, GLOBAL: &mut BvhData) {


    for joint in bvh.joints.iter() {
        if joint.parent_index == -1 {
            GLOBAL.rest_global_positions[joint.index] = GLOBAL.offsets[joint.index];
        } else {
            GLOBAL.rest_global_positions[joint.index] = GLOBAL.offsets[joint.index] + GLOBAL.offsets[joint.parent_index as Index];
        }

        // code source: https://github.com/Wasserwecken/bvhio/blob/c91641e3e41ab5e1281b200a754399ae082f95dd/bvhio/lib/bvh/BvhJoint.py#L48
        let tail_offset = get_tail_offset(joint, &GLOBAL.offsets);
        let dir = tail_offset.normalize();
        let axs = Position {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let dot = dir.dot(axs);
        let rest_global_rotation: Quaternion = if dot < -0.9999 {
            Quaternion::new(0.0, 0.0, 0.0, 1.0)
        } else if dot > 0.9999 {
            Quaternion::new(1.0, 0.0, 0.0, 0.0)
        } else {
            let angle = (dot).acos();
            let axis = axs.cross(dir).normalize();
            Quaternion::from_axis_angle(axis, Rad(angle))
        };
        GLOBAL.rest_global_rotations[joint.index] = rest_global_rotation;
    }
        
   
}

/// Calculate the global pose position of a joint. Basically forward kinematics.
fn calc_global_pose(bvh: &BvhMetadata, GLOBAL: &mut BvhData){
    for joint in bvh.joints.iter() {
        let mut pose_positions = vec![Position::default(); bvh.num_frames];
        let mut pose_rotations: Vec<Quaternion> = vec![Quaternion::default(); bvh.num_frames];
        for frame in 0..bvh.num_frames {
            let mut i: usize = joint.index;
            let mut transform = cgmath::Decomposed::one();
            
            loop {
                if i as ParentIndex == -1 {
                    break;
                }
                transform = cgmath::Decomposed {
                    scale: 1.0,
                    rot: GLOBAL.pose_local_rotations[i][frame],
                    disp: GLOBAL.offsets[i],
                } * transform;
                i = bvh.joints[i].parent_index as Index;
            }
            pose_positions[frame] = transform.disp;
            pose_rotations[frame] = transform.rot;
        }
        GLOBAL.pose_global_positions[joint.index] = pose_positions;
        GLOBAL.pose_global_rotations[joint.index] = pose_rotations;
    }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Main function for end to end .bvh parsing.
pub fn load_bvh(file_path: &str) -> (BvhMetadata,BvhData) {
    let file  = File::open(file_path).unwrap();
    let reader = io::BufReader::new(file);

    let mut data = BvhData::new();

    let mut bvh = BvhMetadata {
        joints: Vec::new(),
        num_frames: 0,
        frame_time: 0.0,
        fps: 0
    };
    let mut joints: Vec<Joint> = Vec::new();
    let mut motion_data = Vec::new();
    let mut rotation_order = String::new();
    let mut positional_channels: Vec<Vec<Index>> = Vec::new();
    let mut rotational_channels: Vec<Vec<Index>> = Vec::new();

    let mut parsing_motion = false;
    let mut parsing_endsite = false;
    let mut channels_index = 0;
    let mut depth:Depth = 0;
    let mut begin_new_branch = false;

    let re_joint = Regex::new(r"(ROOT|JOINT) (\w+)").unwrap();
    let re_offset = Regex::new(r"OFFSET (.+)").unwrap();
    let re_channels = Regex::new(r"CHANNELS (\d) (.+)").unwrap();

    for line in reader.lines() {
        let line = line.unwrap();
        let line = line.trim();
        if line.starts_with("HIERARCHY") || line.is_empty(){
            continue;
        } 
        else if line.starts_with("ROOT") || line.starts_with("JOINT") {
            //// Create joint
            let captures = re_joint.captures(&line);

            

            if let Some(captures) = captures {
                let name = captures.get(2).unwrap().as_str().to_string();
                //// index of parent joint depends on whether we are starting a new branch or not
                let joint_index = joints.len() as Index;
                let parent_index = find_parent_joint_index_by_depth(depth, joint_index as ParentIndex, &joints);
                let joint = Joint {
                    name,
                    index: joint_index,
                    parent_index,
                    children: Vec::new(),
                    is_leaf: false,
                    endsite: None,
                    depth: depth,
                };
                positional_channels.push(Vec::new());
                rotational_channels.push(Vec::new());
                //// If joint has a parent, add this joint to its parent's children
                if joint.parent_index != -1 {
                    if let Some(parent) = joints.get_mut(joint.parent_index as Index) {
                        parent.children.push(joint.index);
                    }
                }
                joints.push(joint);
            }
            data.offsets.push(Position::default());
            data.rest_global_positions.push(Position::default());
            data.rest_global_rotations.push(Quaternion::default());
            data.pose_global_positions.push(Vec::new());
            data.pose_global_rotations.push(Vec::new());
            data.pose_local_rotations.push(Vec::new());
        } else if line.to_lowercase().starts_with("end"){
            //// Create endsite
            parsing_endsite = true;
            let endsite = Endsite {
                offset: Position::default(),
            };
            if let Some(joint) = joints.last_mut() {
                joint.endsite = Some(endsite);
            }
        } else if line == "{" {
            //// Increase parent_index
            depth += 1;
        } else if line == "}" {
            //// Decrease parent_index
            depth -= 1;
        }else if line.starts_with("OFFSET") {
            //// Parse offset
            let captures = re_offset.captures(&line);
            if let Some(captures) = captures {
                let offset: Vec<f64> = captures
                    .get(1)
                    .unwrap()
                    .as_str()
                    .split_whitespace()
                    .map(|s| s.parse::<f64>().unwrap())
                    .collect();
                let offset: Position = Position {
                    x: offset[0],
                    y: offset[1],
                    z: offset[2],
                };
                
                if let Some(joint) = joints.last_mut() {
                    if parsing_endsite {
                        joint.endsite = Some(Endsite { offset });
                        joint.is_leaf = true;
                        parsing_endsite = false;
                    } else {
                        data.offsets[joint.index] = offset;
                    }
                }
            }
        } else if line.starts_with("CHANNELS") {
            //// Parse channels
            let captures = re_channels.captures(&line);
            if let Some(captures) = captures {
                // let num_channels = captures.get(1).unwrap().as_str().parse::<usize>().unwrap();
                let channel_names = captures.get(2).unwrap().as_str().split_whitespace().collect::<Vec<&str>>();
                //// if rotation order hasn't been assigned yet assign it with the first letter of each channel name
                if rotation_order.is_empty() && channel_names.len() == 3 {
                    rotation_order = channel_names
                        .iter().map(|s| s.chars().next().unwrap())
                        .collect::<String>();
                }
                if let Some(joint) = joints.last_mut() {
                    for channel_name in channel_names {
                        match channel_name {
                            "Xposition" | "Yposition" | "Zposition" => {
                                positional_channels[joint.index].push(channels_index);
                                channels_index += 1;
                            }
                            "Xrotation" | "Yrotation" | "Zrotation" => {
                                rotational_channels[joint.index].push(channels_index);
                                channels_index += 1;
                            }
                            _ => {}
                        }
                    }
                    
                }
            }
        } else if line.starts_with("MOTION") {
            //// Note the start of motion data
            parsing_motion = true;
        } else if line.starts_with("Frames:") {
            //// Parse number of frames
            bvh.num_frames = line
                .split_whitespace()
                .nth(1)
                .unwrap()
                .parse::<usize>()
                .unwrap();
        } else if line.starts_with("Frame Time:") {
            //// Parse frame time
            bvh.frame_time = line
                .split_whitespace()
                .nth(2)
                .unwrap()
                .parse::<f64>()
                .unwrap();
            bvh.fps = (1.0 / bvh.frame_time) as u32;
        } else if parsing_motion {
            //// Parse motion data
            let motion_line: Vec<f64> = line
                .split_whitespace()
                .map(|s| s.parse::<f64>().unwrap())
                .collect();
            motion_data.push(motion_line);
        }
    }

    //// for each joint fill it's local_rotations and global_positions fields with data from motion_data
    for joint in joints.iter_mut() {
        let local_rotations: Vec<Quaternion> = motion_data
            .iter_mut()
            .map(|frame| {
                Euler::new(
                    frame[rotational_channels[joint.index][0]],
                    frame[rotational_channels[joint.index][1]],
                    frame[rotational_channels[joint.index][2]],
                    &rotation_order,
                )
                .to_quat()
            })
            .collect();
        data.pose_local_rotations[joint.index] = local_rotations;

        //// if joint has no parent then it's root/hips and it has positional channel
        if joint.parent_index == -1 {
            let global_positions: Vec<Position> = motion_data
                .iter_mut()
                .map(|frame| Position {
                    x: frame[positional_channels[joint.index][0]],
                    y: frame[positional_channels[joint.index][1]],
                    z: frame[positional_channels[joint.index][2]],
                })
                .collect();
            data.pose_global_positions[joint.index] = global_positions;
        }
    }

    //// fill bvh.joints with filled in joints
    bvh.joints = joints;

    //// for each joint fill it's rest_global_position and rest_global_rotation fields
    calc_global_rest_pose(&bvh, &mut data);
    calc_global_pose(&bvh, &mut data);
   
    (bvh,data)
}