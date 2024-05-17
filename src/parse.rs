use crate::types::*;
use crate::utils;
use cgmath::{Decomposed, InnerSpace, Rad, Rotation3, Transform, Vector3, Zero};
use core::panic;
use regex::Regex;
use std::str::Lines;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Used during joint creation to fill in its parent index (after parent index has been assigned, this function becomes redundant).
/// Alogirthm: searches for the joint with depth 1 less than the current joint's depth.
fn __find_parent_joint_index_by_depth(
    joint_depth: Depth,
    joint_index: ParentIndex,
    joints: &Vec<Joint>,
) -> ParentIndex {
    // detect root joint
    if joints.is_empty() {
        return -1;
    }
    let mut i = joint_index - 1;
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
fn __get_tail_offset(joint: &Joint, offsets: &Vec<Position>) -> Position {
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
fn __calc_rest_pose(bvh: &BvhMetadata, data: &mut BvhData) {
    for joint in bvh.joints.iter() {
        //// CALCULATE REST GLOBAL POSITIONS
        data.rest_global_positions[joint.index] = if joint.parent_index != -1 {
            data.rest_local_positions[joint.index]
                + data.rest_global_positions[joint.parent_index as Index]
        } else {
            data.rest_local_positions[joint.index]
        };

        //// CALCULATE REST GLOBAL ROTATIONS (this quite specific to .bvh files as they don't specify the rest post orientation of joints, so we have to calculate it ourselves)
        // code source: https://github.com/Wasserwecken/bvhio/blob/c91641e3e41ab5e1281b200a754399ae082f95dd/bvhio/lib/bvh/BvhJoint.py#L48
        let tail_offset = __get_tail_offset(joint, &data.rest_local_positions);
        let dir = if tail_offset != Position::zero() {
            tail_offset.normalize()
        } else {
            Position::new(0.0, 1.0, 0.0)
        }; // prevent NaNs in case of zero offset joints
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
            cgmath::Quaternion::from_axis_angle(axis, Rad(angle))
        };
        data.rest_global_rotations[joint.index] = rest_global_rotation;

        //// CALCULATE REST LOCAL ROTATIONS
        data.rest_local_rotations[joint.index] = if joint.parent_index == -1 {
            rest_global_rotation
        } else {
            let r = Decomposed {
                scale: 1.0,
                rot: data.rest_global_rotations[joint.parent_index as Index],
                disp: Position::identity(),
            }
            .inverse_transform()
            .expect("Error during inverting a matrix. This shouldn't have happened.")
                * Decomposed {
                    scale: 1.0,
                    rot: data.rest_global_rotations[joint.index],
                    disp: Position::identity(),
                };
            r.rot
        };
    }
}

/// Calculate the global pose position of a joint. Basically forward kinematics.
fn __calc_pose(bvh: &BvhMetadata, data: &mut BvhData) {
    fn ____recursive_transform(
        joint_index: Index,
        frame: usize,
        metadata: &BvhMetadata,
        data: &mut BvhData,
    ) {
        let i: usize = joint_index;
        let transform = Decomposed {
            scale: 1.0,
            rot: data.pose_local_rotations[i][frame],
            disp: if i == 0 {
                data.pose_global_positions[i][frame]
            } else {
                data.rest_local_positions[i]
            },
        };
        
        let parent_index = metadata.joints[i].parent_index;

        let parent_transform = if parent_index == -1 {
            Decomposed {
                scale: 1.0,
                rot: Quaternion::identity(),
                disp: Position::identity(),
            }
        } else {
            Decomposed {
                scale: 1.0,
                rot: data.pose_global_rotations[parent_index as Index][frame],
                disp: data.pose_global_positions[parent_index as Index][frame],
            }
        };

        let transform = parent_transform * transform ;
        data.pose_global_positions[i][frame] = transform.disp;
        data.pose_global_rotations[i][frame] = transform.rot;
    }

    for joint in bvh.joints.iter() {
        for frame in 0..bvh.num_frames {
            ____recursive_transform(joint.index, frame, bvh, data);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

fn parse_bvh(lines: Lines) -> (BvhMetadata, BvhData) {
    let mut rest_local_positions: Vec<Position> = Vec::new();

    let mut num_frames = 0;
    let mut frame_time = 0.0;
    let mut fps = 0;
    let mut joints: Vec<Joint> = Vec::new();

    let mut rotation_order = String::new();
    // let mut positional_channels: Vec<Vec<Index>> = Vec::new();
    let mut rotational_channels: Vec<Vec<Index>> = Vec::new();

    let mut parsing_motion = false;
    let mut parsing_endsite = false;
    let mut channels_index = 0;
    let mut depth: Depth = 0;

    let re_joint = Regex::new(r"(ROOT|JOINT) (\w+)").unwrap();
    let re_offset = Regex::new(r"OFFSET (.+)").unwrap();
    let re_channels = Regex::new(r"CHANNELS (\d) (.+)").unwrap();

    //// PARSING LINE BY LINE
    let mut it = lines.into_iter();
    loop {
        let line = it
            .next()
            .expect("Error parsing bvh file. Unexpected end of file.");
        let line = line.trim();

        if line.starts_with("HIERARCHY") || line.is_empty() {
            continue;
        } else if line.starts_with("ROOT") || line.starts_with("JOINT") {
            //// Create joint
            let captures = re_joint.captures(&line);
            if let Some(captures) = captures {
                let name = captures.get(2).unwrap().as_str().to_string();
                //// index of parent joint depends on whether we are starting a new branch or not
                let joint_index = joints.len() as Index;
                let parent_index =
                    __find_parent_joint_index_by_depth(depth, joint_index as ParentIndex, &joints);
                let joint = Joint {
                    name,
                    index: joint_index,
                    parent_index,
                    children: Vec::new(),
                    is_leaf: false,
                    endsite: None,
                    depth: depth,
                };
                // positional_channels.push(Vec::new());
                rotational_channels.push(Vec::new());

                //// If joint has a parent, add this joint to its parent's children
                if joint.parent_index != -1 {
                    if let Some(parent) = joints.get_mut(joint.parent_index as Index) {
                        parent.children.push(joint.index);
                    }
                }
                joints.push(joint);
            } else {
                panic!("Error parsing joint name. Line starts with ROOT or JOINT but joint name was not found.");
            }
        } else if line.to_lowercase().starts_with("end") {
            //// Create endsite
            parsing_endsite = true;
            let endsite = Endsite {
                offset: Position::identity(),
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
        } else if line.starts_with("OFFSET") {
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
                        rest_local_positions.push(offset);
                    }
                }
            }
        } else if line.starts_with("CHANNELS") {
            //// Parse channels
            let captures = re_channels.captures(&line);
            if let Some(captures) = captures {
                // let num_channels = captures.get(1).unwrap().as_str().parse::<usize>().unwrap();
                let channel_names = captures
                    .get(2)
                    .unwrap()
                    .as_str()
                    .split_whitespace()
                    .collect::<Vec<&str>>();
                //// if rotation order hasn't been assigned yet assign it with the first letter of each channel name
                if rotation_order.is_empty() && channel_names.len() == 3 {
                    rotation_order = channel_names
                        .iter()
                        .map(|s| s.chars().next().unwrap())
                        .collect::<String>();
                }
                if let Some(joint) = joints.last_mut() {
                    for channel_name in channel_names {
                        match channel_name {
                            "Xposition" | "Yposition" | "Zposition" => {
                                // positional_channels[joint.index].push(channels_index);
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
        } else if line.starts_with("Frames:") {
            //// Parse number of frames
            num_frames = line
                .split_whitespace()
                .nth(1)
                .unwrap()
                .parse::<usize>()
                .unwrap();
        } else if line.starts_with("Frame Time:") {
            //// Parse frame time
            frame_time = line
                .split_whitespace()
                .nth(2)
                .unwrap()
                .parse::<f64>()
                .unwrap();
            fps = (1.0 / frame_time) as u32;
            break; // jump to parsing Motion
        }
    }

    //// initialize fields which will be filled in later
    let rest_local_rotations: Vec<Quaternion> = vec![Quaternion::identity(); joints.len()];
    let rest_global_positions: Vec<Position> = vec![Position::identity(); joints.len()];
    let rest_global_rotations: Vec<Quaternion> = vec![Quaternion::identity(); joints.len()];

    let mut pose_global_positions: Vec<Vec<Position>> =
        vec![vec![Position::identity(); num_frames]; joints.len()];
    let pose_global_rotations: Vec<Vec<Quaternion>> =
        vec![vec![Quaternion::identity(); num_frames]; joints.len()];
    let pose_local_positions: Vec<Vec<Position>> =
        vec![vec![Position::identity(); num_frames]; joints.len()];
    let mut pose_local_rotations: Vec<Vec<Quaternion>> =
        vec![vec![Quaternion::identity(); num_frames]; joints.len()];

    /////////////////////////////////// PARSING MOTION ///////////////////////////////////

    for (i, line) in it.enumerate() {
        //// Parse motion data (positional_channels and rotational_channels are already fully filled in)
        let motion_line: Vec<f64> = line
            .split_whitespace()
            .map(|s| s.parse::<f64>().expect("Error parsing motion data"))
            .collect();

        //// Parse positional channels
        pose_global_positions[0][i] = Position::new(motion_line[0], motion_line[1], motion_line[2]);

        //// Parse rotational channels
        for (joint_index, motion_indices) in rotational_channels.iter().enumerate() {
            let eul = (
                motion_line[motion_indices[0]],
                motion_line[motion_indices[1]],
                motion_line[motion_indices[2]],
            );
            let eul = utils::__reorder_vector(eul.0, eul.1, eul.2, &rotation_order);
            let quat = utils::__from_euler_to_quat(eul.0, eul.1, eul.2, &rotation_order);
            pose_local_rotations[joint_index][i] = quat;
        }
    }

    let mut data = BvhData {
        rest_local_positions,
        rest_local_rotations,
        rest_global_positions,
        rest_global_rotations,
        pose_local_positions,
        pose_local_rotations,
        pose_global_positions,
        pose_global_rotations,
    };

    let metadata = BvhMetadata {
        joints,
        num_frames,
        frame_time,
        fps,
    };

    //// for each joint fill it's global rest pose and global pose
    __calc_rest_pose(&metadata, &mut data);
    __calc_pose(&metadata, &mut data);

    return (metadata, data);
}

//////////////////////////////////////////////////////////////// PUBLIC ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// load a bvh file from a file path
pub fn load_bvh_from_file(file_path: &str) -> (BvhMetadata, BvhData) {
    let contents = std::fs::read_to_string(file_path).expect("Error reading file");
    return __load_bvh(contents.lines());
}

/// load a bvh file from a string
pub fn load_bvh_from_string(bvh_string: &str) -> (BvhMetadata, BvhData) {
    return __load_bvh(bvh_string.lines());
}

fn __load_bvh(lines: Lines) -> (BvhMetadata, BvhData) {
    //// parse bvh file
    let (metadata, data) = parse_bvh(lines);
    return (metadata, data);
}
