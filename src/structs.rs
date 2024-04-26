use cgmath::{
    InnerSpace, One, Quaternion as CgQuaternion, Rad, Rotation, Rotation3, SquareMatrix, Vector3,
};

pub trait Default {
    fn default() -> Self;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

pub struct Euler {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/////////////////////////////////////////////////////////////////////////////////////////////////

pub type Index = usize;
pub type ParentIndex = isize; // can be -1 if joint has no parent
pub type Quaternion = CgQuaternion<f64>;
pub type Position = Vector3<f64>;

/////////////////////////////////////////////////////////////////////////////////////////////////

pub struct Joint<'a> {
    pub name: String,
    pub index: Index,
    pub offset: Position,
    pub endsite: Option<Position>,
    pub positional_channels: Vec<Index>,
    pub rotational_channels: Vec<Index>,
    pub parent_index: ParentIndex,
    pub parent: Option<&'a Joint<'a>>,
    pub children: Vec<Index>,

    pub rest_global_position: Position,
    pub rest_global_rotation: Quaternion,

    pub pose_global_positions: Vec<Position>,
    pub pose_global_rotations: Vec<Quaternion>,
    pub pose_local_rotations: Vec<Quaternion>,

}

pub struct Bvh<'a> {
    pub joints: Vec<Joint<'a>>,
    pub num_frames: usize,
    pub frame_time: f64,
    pub fps: u32,
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

///default fro cgmath::Quaternion
impl Default for Quaternion {
    fn default() -> Self {
        Quaternion::new(999.0, 999.0, 999.0, 999.0)
    }
}
///default for cgmath::Vector3
impl Default for Position {
    fn default() -> Self {
        Position {
            x: 999.0,
            y: 999.0,
            z: 999.0,
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/// 
impl Default for Euler {
    fn default() -> Self {
        Euler {
            x: -999.0,
            y: -999.0,
            z: -999.0,
        }
    }
}
impl Euler {
    pub fn new(x: f64, y: f64, z: f64, order: &str) -> Euler {
        match order {
            "XYZ" => Euler { x, y, z },
            "XZY" => Euler { x, y: z, z: y },
            "YXZ" => Euler { x: y, y: x, z },
            "YZX" => Euler { x: z, y: x, z: y },
            "ZXY" => Euler { x: y, y: z, z: x },
            "ZYX" => Euler { x: z, y: y, z: x },
            _ => Euler::default(),
        }
    }

    ///TODO: check if correct
    pub fn to_quat(&self) -> Quaternion {
        let c1 = (self.x * 0.5).cos();
        let c2 = (self.y * 0.5).cos();
        let c3 = (self.z * 0.5).cos();
        let s1 = (self.x * 0.5).sin();
        let s2 = (self.y * 0.5).sin();
        let s3 = (self.z * 0.5).sin();

        return Quaternion::new(
            c1 * c2 * c3 - s1 * s2 * s3,
            s1 * c2 * c3 + c1 * s2 * s3,
            c1 * s2 * c3 - s1 * c2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
        );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


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
    } else {
        // return an arbitrary vector (e.g. pointing upwards) if joint has no children (i.e. it's an endsite joint) - it doesn't matter
        return Position {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
    }
}

/// Calculate the global rest pose of a joint.
pub fn calc_global_rest_pose(bvh: &mut Bvh) {

    let offsets: Vec<Position> = bvh.joints.iter().map(|joint| joint.offset).collect();

    for joint in bvh.joints.iter_mut() {
        if joint.parent_index == -1 {
            joint.rest_global_position = joint.offset;
        } else {
            joint.rest_global_position = joint.offset + offsets[joint.parent_index as Index];
        }

        // code source: https://github.com/Wasserwecken/bvhio/blob/c91641e3e41ab5e1281b200a754399ae082f95dd/bvhio/lib/bvh/BvhJoint.py#L48
        let tail_offset = get_tail_offset(joint, &offsets);
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
        joint.rest_global_rotation = rest_global_rotation;
    }
        
   
}

/// Calculate the global pose position of a joint. Basically forward kinematics.
pub fn calc_global_pose(bvh: &mut Bvh){
    for joint in bvh.joints.iter_mut() {
        let mut pose_positions = vec![Position::default(); bvh.num_frames];
        let mut pose_rotations: Vec<CgQuaternion<f64>> = vec![Quaternion::default(); bvh.num_frames];
        for frame in 0..bvh.num_frames {
            let mut i: usize = joint.index;
            let mut transform = cgmath::Decomposed::one();

            loop {
                if joint.parent_index == -1 {
                    break;
                }
                transform = cgmath::Decomposed {
                    scale: 1.0,
                    rot: joint.pose_local_rotations[frame],
                    disp: joint.offset,
                } * transform;
                i = joint.parent_index as usize;
            }
            pose_positions[frame] = transform.disp;
            pose_rotations[frame] = transform.rot;
        }
        joint.pose_global_positions = pose_positions;
        joint.pose_global_rotations = pose_rotations;
    }
}