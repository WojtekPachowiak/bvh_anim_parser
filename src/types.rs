use cgmath::{
    InnerSpace, One, Quaternion as CgQuaternion, Rad, Rotation, Rotation3, SquareMatrix, Vector3,
};


pub trait Default {
    fn default() -> Self;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#[derive(Debug)]
pub struct BvhData{
    pub offsets: Vec<Position>,
    pub parents : Vec<ParentIndex>,

    pub rest_global_positions: Vec<Position>,
    pub rest_global_rotations: Vec<Quaternion>,

    pub pose_global_positions: Vec<Vec<Position>>,
    pub pose_global_rotations: Vec<Vec<Quaternion>>,
    pub pose_local_rotations: Vec<Vec<Quaternion>>,

}

impl BvhData{
    pub fn new() -> Self{
        BvhData{
            offsets: Vec::new(),
            parents: Vec::new(),
            rest_global_positions: Vec::new(),
            rest_global_rotations: Vec::new(),
            pose_global_positions: Vec::new(),
            pose_global_rotations: Vec::new(),
            pose_local_rotations: Vec::new(),
        }
    }
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
pub type Depth = usize;

/////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub index: Index,
    pub parent_index: ParentIndex,
    pub depth: Depth,
    pub children: Vec<Index>,
    pub is_leaf: bool,
    pub endsite: Option<Endsite>,
}

#[derive(Debug)]
pub struct Endsite {
    pub offset: Position,
}

#[derive(Debug)]
pub struct BvhMetadata {
    pub joints: Vec<Joint>,
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