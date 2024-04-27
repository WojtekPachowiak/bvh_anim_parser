//! This example demonstrates Bevy's immediate mode drawing API intended for visual debugging.

use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy::prelude::*;
use crate::types::{BvhData, Position,Quaternion, BvhMetadata, Index};
use crate::bvh_parsing::get_kinematic_chains;

#[derive(Debug,Resource)]
pub struct AppGlobalData {
    pub anim_data: BvhData,
    pub bvh: BvhMetadata,
    pub kinematic_chain: Vec<Vec<Index>>,
    pub rest_pose_mode: bool,
    pub playing: bool,
    pub frame: usize,
    pub scale: f32,
}

pub fn visualize_skeleton(anim_data: BvhData, bvh: BvhMetadata, scale:f32) {
    let kinematic_chain = get_kinematic_chains(&bvh);

    App::new()
        .insert_resource(AppGlobalData {
            anim_data,
            bvh,
            kinematic_chain,
            rest_pose_mode: false,
            playing: true,
            frame: 0,
            scale: 1.0/scale, // reciprocal for historical/laziness reasons xdd
        })
        .add_plugins(DefaultPlugins)
        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Startup, setup)
        // .add_systems(Update, change_frame)
        .add_systems(Update, (draw_skeleton, update_main))
        .run();
}



fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    //// Orbit camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0., 1.5, 6.).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        PanOrbitCamera::default(),
    ));
    // plane
    commands.spawn(MaterialMeshBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(5.0, 5.0)),
        material: materials.add(Color::WHITE),
        ..default()
    });
  
    // instructions
    commands.spawn(
        TextBundle::from_section(
            "Press 'R' to toggle rest pose mode\n\
            Press 'Left' or 'Right' to increment frames\n\
            Hold 'Down' or 'Up' to change the line width of straight gizmos\n\
            Hold 'Up' or 'Down' to change the line width of round gizmos\n",
            TextStyle {
                font_size: 17.,
                ..default()
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        }),
    );
}


/// Draw joint axes (red, green, blue) at the joint position.
fn draw_joint_axes(gizmos: &mut Gizmos, rotation: &Quaternion, position: &Position,scale:f32) {
    let quat: cgmath::Quaternion<f64> = rotation.clone();
    let position = Vec3::new(position.x as f32, position.y as f32, position.z as f32) / scale ;
    // convert quaternion to matrix 3x3
    let mut rotation: cgmath::Matrix3<f64> = cgmath::Matrix3::from(quat);
    //  increase scale for longer axes
    rotation.x = rotation.x * 5.0;
    rotation.y = rotation.y * 5.0;
    rotation.z = rotation.z * 5.0;
    // draw axes
    let x_axis = Vec3::new(rotation.x.x as f32, rotation.x.y as f32, rotation.x.z as f32)/ scale + position;
    let y_axis = Vec3::new(rotation.y.x as f32, rotation.y.y as f32, rotation.y.z as f32)/ scale + position;
    let z_axis = Vec3::new(rotation.z.x as f32, rotation.z.y as f32, rotation.z.z as f32)/ scale + position;

    gizmos.line(position, x_axis, Color::RED);
    gizmos.line(position, y_axis, Color::GREEN);
    gizmos.line(position, z_axis, Color::BLUE);
}

/// Draw a sphere at the joint position.
fn draw_joint_sphere(gizmos: &mut Gizmos, position: &Position, scale:f32) {
    let position = Vec3::new(position.x as f32, position.y as f32, position.z as f32) /scale;
    gizmos.sphere(position, Quat::IDENTITY, 0.1/scale, Color::WHITE);
}

fn draw_skeleton(
    mut gizmos: Gizmos,
    time: Res<Time>,
    appdata: Res<AppGlobalData>,
) {
    let frame = appdata.frame;
    let scale = appdata.scale;

    /// Get the global position and rotation of a joint at a specific frame.
    /// If in rest pose mode, return the rest pose global position and rotation independent of frame.
    fn get_global_position_rotation(joint_index: Index, frame: usize, app_data: &AppGlobalData) -> (&Position, &Quaternion) {
        let rest_pose_mode = app_data.rest_pose_mode;
        if rest_pose_mode {
            let global_positions = &app_data.anim_data.rest_global_positions;
            let global_rotations = &app_data.anim_data.rest_global_rotations;
            return (&global_positions[joint_index], &global_rotations[joint_index]);
        }
        let global_positions = &app_data.anim_data.pose_global_positions;
        let global_rotations = &app_data.anim_data.pose_global_rotations;
        return (&global_positions[joint_index][frame], &global_rotations[joint_index][frame])
    }
    
    //// Draw the skeleton lines
    let kinematic_chain = &appdata.kinematic_chain;
    let kinematic_chain_length = kinematic_chain.len();
    for (i,chain) in kinematic_chain.iter().enumerate() {
        let positions = chain.iter().map(|&joint_index| {
            let (pos,_) = get_global_position_rotation(joint_index, frame, &appdata);
            Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32) /scale
        }).collect::<Vec<_>>();
        gizmos.linestrip(positions, Color::hsl(i as f32 / kinematic_chain_length as f32, 0.5, 0.5));
    }

    //// Draw the joints as spheres. Draw the axes of the joints.
    let bvh = &appdata.bvh;
    for joint in &bvh.joints {
        let joint_index = joint.index;
        let (pos,rot) = get_global_position_rotation(joint_index, frame, &appdata);
        draw_joint_sphere(&mut gizmos, pos, scale);
        draw_joint_axes(&mut gizmos, rot, pos,scale);
    }
    
    
}

fn update_main(
    mut config_store: ResMut<GizmoConfigStore>,
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut appdata: ResMut<AppGlobalData>
) {

    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    if keyboard.pressed(KeyCode::ArrowUp) {
        config.line_width += 5. * time.delta_seconds();
        config.line_width = config.line_width.clamp(0., 50.);
    }
    if keyboard.pressed(KeyCode::ArrowDown) {
        config.line_width -= 5. * time.delta_seconds();
        config.line_width = config.line_width.clamp(0., 50.);
    }

    if keyboard.just_released(KeyCode::KeyR) {
        appdata.rest_pose_mode = !appdata.rest_pose_mode;
    }

    if keyboard.just_released(KeyCode::Space) {
        appdata.playing = !appdata.playing;
    }

    if appdata.playing {
        appdata.frame = (time.elapsed_seconds_f64() * appdata.bvh.fps as f64).round() as usize % appdata.bvh.num_frames;
    }

    if keyboard.just_released(KeyCode::ArrowRight) {
        appdata.playing = false;
        appdata.frame += 1;
        appdata.frame = appdata.frame % appdata.bvh.num_frames;
    }

    if keyboard.just_released(KeyCode::ArrowLeft) {
        appdata.playing = false;
        if appdata.frame == 0 {
            appdata.frame = appdata.bvh.num_frames-1;
        }else{
            appdata.frame -= 1;
        }
    }


}
