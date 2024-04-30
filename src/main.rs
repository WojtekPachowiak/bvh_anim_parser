
mod bvh_parsing;
use bvh_parsing::{load_bvh};
mod skeleton_drawing;
use skeleton_drawing::visualize_skeleton;
mod types;

fn main() {
    let (bvh_metadata,bvh_data) = load_bvh("./test_anim_sword_attack.bvh");

    bvh_data.print_rest_local();
    bvh_data.print_rest_global();


    visualize_skeleton(bvh_data, bvh_metadata, 0.01);
}
