use regex::Regex;
use std::fs::File;
use std::io::{self, BufRead};
mod structs;
use structs::{Euler, Index, Position, Quaternion, Bvh, Joint, Default, calc_global_pose, calc_global_rest_pose};

fn main() -> io::Result<()> {
    let file = File::open("Vicon-sword_attack-0e6d7941-a867-4133-86b9-348542537958.bvh")?;
    let reader = io::BufReader::new(file);

    let mut bvh = Bvh {
        joints: Vec::new(),
        num_frames: 0,
        frame_time: 0.0,
        fps: 0
    };
    let mut joints: Vec<Joint> = Vec::new();
    let mut motion_data = Vec::new();
    let mut rotation_order = String::new();

    let mut parsing_motion = false;
    let mut parsing_endsite = false;
    let mut channels_index = 0;
    let mut parent_index = -1;

    let re_joint = Regex::new(r"(ROOT|JOINT) (\w+)").unwrap();
    let re_offset = Regex::new(r"OFFSET (.+)").unwrap();
    let re_channels = Regex::new(r"CHANNELS (\d) (.+)").unwrap();

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();
        if line.starts_with("HIERARCHY") {
            continue;
        } else if line == "{" {
            //// Increase parent_index
            parent_index += 1;
        } else if line == "}" {
            //// Decrease parent_index
            parent_index -= 1;
        } 
        else if line.starts_with("ROOT") || line.starts_with("JOINT") {
            //// Create joint
            let captures = re_joint.captures(&line);
            if let Some(captures) = captures {
                let name = captures.get(2).unwrap().as_str().to_string();
                let joint = Joint {
                    name,
                    index: joints.len() as Index,
                    offset: Position::default(),
                    endsite: None,
                    positional_channels: Vec::new(),
                    rotational_channels: Vec::new(),
                    parent_index,
                    parent: None,
                    children: Vec::new(),
                    rest_global_position: Position::default(),
                    rest_global_rotation: Quaternion::default(),
                    pose_global_positions: Vec::new(),
                    pose_global_rotations: Vec::new(),
                    pose_local_rotations: Vec::new(),
                };
                //// If joint has a parent, add this joint to its parent's children
                if parent_index != -1 {
                    if let Some(parent) = joints.get_mut(parent_index as Index) {
                        parent.children.push(joint.index);
                    }
                }
                joints.push(joint);
            }
        } else if line.starts_with("OFFSET") {
            //// Parse offset
            let captures = re_offset.captures(&line);
            if let Some(captures) = captures {
                let offset: Vec<&str> = captures
                    .get(1)
                    .unwrap()
                    .as_str()
                    .split_whitespace()
                    .collect();
                let offset: Vec<f64> = offset.iter().map(|s| s.parse::<f64>().unwrap()).collect();
                let offset: Position = Position {
                    x: offset[0],
                    y: offset[1],
                    z: offset[2],
                };
                if parsing_endsite {
                    if let Some(joint) = joints.last_mut() {
                        joint.endsite = Some(offset);
                    }
                    parsing_endsite = false;
                } else {
                    if let Some(joint) = joints.last_mut() {
                        joint.offset = offset;
                    }
                }
            }
        } else if line.to_lowercase().starts_with("end") {
            //// Parse end site
            if let Some(joint) = joints.last_mut() {
                parsing_endsite = true;
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
                                joint.positional_channels.push(channels_index);
                                channels_index += 1;
                            }
                            "Xrotation" | "Yrotation" | "Zrotation" => {
                                joint.rotational_channels.push(channels_index);
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
                    frame[joint.rotational_channels[0]],
                    frame[joint.rotational_channels[1]],
                    frame[joint.rotational_channels[2]],
                    &rotation_order,
                )
                .to_quat()
            })
            .collect();
        joint.pose_local_rotations = local_rotations;

        //// if joint has no parent then it's root/hips and it has positional channel
        if joint.parent_index == -1 {
            let global_positions: Vec<Position> = motion_data
                .iter_mut()
                .map(|frame| Position {
                    x: frame[joint.positional_channels[0]],
                    y: frame[joint.positional_channels[1]],
                    z: frame[joint.positional_channels[2]],
                })
                .collect();
            joint.pose_global_positions = global_positions;
        }
    }

    bvh.joints = joints;

    //// for each joint fill it's rest_global_position and rest_global_rotation fields
    calc_global_rest_pose(&mut bvh);
    calc_global_pose(&mut bvh);
   


    Ok(())
}
