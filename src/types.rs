use cgmath::One;

/////////////////////////////////////////////////////////////////////////////////////////////////
#[derive(Debug)]
pub struct BvhData {
    /// This is the same as OFFSET in the HIERARCHY of .bvh file
    pub rest_local_positions: Vec<Position>, // for root joint it's the same thing as its rest_global_positions

    pub rest_local_rotations: Vec<Quaternion>, 
    pub rest_global_positions: Vec<Position>, 
    pub rest_global_rotations: Vec<Quaternion>, 

    pub pose_global_positions: Vec<Vec<Position>>, 
    pub pose_global_rotations: Vec<Vec<Quaternion>>, 
    pub pose_local_rotations: Vec<Vec<Quaternion>>, // every joint has it
    pub pose_local_positions: Vec<Vec<Position>>, // usually root joint has it, but other joints don't (they have (0,0,0) as their Vec<Position>)
}

impl BvhData {
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
pub type Quaternion = cgmath::Quaternion<f64>;
pub type Position = cgmath::Vector3<f64>;
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
impl BvhMetadata {
    /// Find joint by name.
    /// 
    ///  Speed: O(n)
    /// 
    /// (TEMPORARY: will be replaced by HashMap in the future)
    pub fn find_joint_by_name(&self, name: &str) -> &Joint {
        self
            .joints
            .iter()
            .find(|joint| joint.name == name)
            .expect(&format!("Joint {} not found", name))
    }

    /// Find joint by index.
    /// 
    ///  Speed: O(n)
    /// 
    /// (TEMPORARY: will be replaced by HashMap in the future)
    pub fn find_joint_by_index(&self,index: Index) -> &Joint {
        self
            .joints
            .iter()
            .find(|joint| joint.index == index)
            .expect(&format!("Joint with parent index {} not found", index))
    }

    /// Returns the kinematic chain of a bvh like \[\[0,1,2,3\],\[4,5,6,7,8\],\[9,10,11\],\[12,13,14,15\],\[16,17,18\]\]
    /// Usually the chains are: left leg, right leg, left arm, right arm and spine+head.
    pub fn get_kinematic_chains(&self) -> Vec<Vec<Index>> {
        let mut kinematic_chains: Vec<Vec<Index>> = Vec::new();
        let mut chain: Vec<Index> = Vec::new();
        let mut last_depth: isize = -1;
        for joint in self.joints.iter() {
            if last_depth != joint.depth as isize - 1 {
                kinematic_chains.push(chain.clone());
                chain.clear();
            }
            last_depth = joint.depth as isize;
            chain.push(joint.index);
        }
        kinematic_chains.push(chain.clone());
        
        return kinematic_chains
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait Identity {
    fn identity() -> Self;
}

///identity fro cgmath::Quaternion
impl Identity for Quaternion {
    fn identity() -> Self {
        Quaternion::one()
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
