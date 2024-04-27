
mod bvh_parsing;
use bvh_parsing::{load_bvh};
mod skeleton_drawing;
use skeleton_drawing::visualize_skeleton;
mod types;

fn main() {
    let (bvh_metadata,bvh_data) = load_bvh("./Vicon-sword_attack-0e6d7941-a867-4133-86b9-348542537958.bvh");
    visualize_skeleton(bvh_data, bvh_metadata, 0.01);
}
