use cgmath::{One, Quaternion, Vector3};

/////////////////////////////////////////////////////////////////////////////////////////////////
#[derive(Debug)]
pub struct BvhData {
    /// This is the same as OFFSET in the HIERARCHY of .bvh file
    pub rest_local_positions: Vec<Position>,

    pub rest_local_rotations: Vec<Quat>,
    pub rest_global_positions: Vec<Position>,
    pub rest_global_rotations: Vec<Quat>,

    pub pose_global_positions: Vec<Vec<Position>>,
    pub pose_global_rotations: Vec<Vec<Quat>>,
    pub pose_local_rotations: Vec<Vec<Quat>>,
    pub pose_local_positions: Vec<Vec<Position>>,
}

impl BvhData {
    pub fn new() -> Self {
        BvhData {
            rest_local_positions: Vec::new(),
            rest_local_rotations: Vec::new(),
            rest_global_positions: Vec::new(),
            rest_global_rotations: Vec::new(),
            pose_global_positions: Vec::new(),
            pose_global_rotations: Vec::new(),
            pose_local_rotations: Vec::new(),
            pose_local_positions: Vec::new(),
        }
    }

    pub fn print_rest_local(&self) {
        println!("==== REST LOCAL ====");
        for i in 0..self.rest_local_positions.len() {
            println!(
                "{}: {:<20.2?} {:<20.2?}",
                i, self.rest_local_positions[i], self.rest_local_rotations[i]
            );
        }
        Position::identity();
    }

    pub fn print_rest_global(&self) {
        println!("==== REST GLOBAL ====");
        for i in 0..self.rest_local_positions.len() {
            println!(
                "{}: {:<20.2?} {:<20.2?}",
                i, self.rest_global_positions[i], self.rest_global_rotations[i]
            );
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////

pub type Index = usize;
pub type ParentIndex = isize; // can be -1 if joint has no parent
pub type Quat = Quaternion<f64>;
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait Identity {
    fn identity() -> Self;
}

///identity fro cgmath::Quaternion
impl Identity for Quat {
    fn identity() -> Self {
        Quat::one()
    }
}

///identity for cgmath::Vector3
impl Identity for Position {
    fn identity() -> Self {
        Position {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// reorder vector based on euler angles order string
pub(crate) fn reorder_vector(e1: f64, e2: f64, e3: f64, order: &str) -> (f64, f64, f64) {
    match order {
        "ZXY" => (e2, e3, e1),
        "ZYX" => (e3, e2, e1),
        "YXZ" => (e2, e1, e3),
        "YZX" => (e3, e1, e2),
        "XZY" => (e1, e3, e2),
        "XYZ" => (e1, e2, e3),
        _ => panic!("Invalid euler angles order!"),
    }
}

/// Convert euler angles in DEGREES to quaternion
pub(crate) fn __from_euler_to_quat(x: f64, y: f64, z: f64, order: &str) -> Quat {
    let x = x.to_radians();
    let y = y.to_radians();
    let z = z.to_radians();

    let c1 = (x / 2.0).cos();
    let c2 = (y / 2.0).cos();
    let c3 = (z / 2.0).cos();

    let s1 = (x / 2.0).sin();
    let s2 = (y / 2.0).sin();
    let s3 = (z / 2.0).sin();

    let quaternion = match order {
        "XYZ" => Quat::new(
            c1 * c2 * c3 - s1 * s2 * s3,
            s1 * c2 * c3 + c1 * s2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
            c1 * c2 * s3 + s1 * s2 * c3,
        ),
        "YXZ" => Quat::new(
            c1 * c2 * c3 + s1 * s2 * s3,
            s1 * c2 * c3 + c1 * s2 * s3,
            c1 * s2 * c3 - s1 * c2 * s3,
            c1 * c2 * s3 - s1 * s2 * c3,
        ),
        "ZXY" => Quat::new(
            c1 * c2 * c3 - s1 * s2 * s3,
            s1 * c2 * c3 - c1 * s2 * s3,
            c1 * s2 * c3 + s1 * c2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
        ),
        "ZYX" => Quat::new(
            s1 * c2 * c3 - c1 * s2 * s3,
            c1 * s2 * c3 + s1 * c2 * s3,
            c1 * c2 * s3 - s1 * c2 * c3,
            c1 * c2 * c3 + s1 * s2 * s3,
        ),
        "YZX" => Quat::new(
            s1 * c2 * c3 + c1 * s2 * s3,
            c1 * s2 * c3 + s1 * c2 * s3,
            c1 * c2 * s3 - s1 * s2 * c3,
            c1 * c2 * c3 - s1 * s2 * s3,
        ),
        "XZY" => Quat::new(
            s1 * c2 * c3 - c1 * s2 * s3,
            c1 * s2 * c3 - s1 * c2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
            c1 * c2 * c3 + s1 * s2 * s3,
        ),
        _ => panic!("Invalid euler angles order!"),
    };

    return quaternion;
}
