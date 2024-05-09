use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy::prelude::*;
use crate::types::*;
use crate::parse::get_kinematic_chains;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Debug,Resource)]
pub struct AppGlobalData {
    pub anim_data: BvhData,
    pub bvh: BvhMetadata,
    pub kinematic_chain: Vec<Vec<Index>>,
    pub rest_pose_mode: bool,
    pub playing: bool,
    pub frame: usize,
    pub real_frame: f64,
    pub scale: f32,
    pub debug_text:bool
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
            real_frame: 0.0,
            scale: 1.0 / scale, // reciprocal for historical/laziness reasons xdd
            debug_text: false
        })
        .add_plugins(DefaultPlugins)
        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Startup, setup)
        // .add_systems(Update, change_frame)
        .add_systems(Update, (draw_skeleton, update_main, update_debug_text))
        .run();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// A unit struct to help identify the FPS UI component, since there may be many Text components
#[derive(Component)]
struct DebugText;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    // draw plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(5.0, 5.0)),
        material: materials.add(StandardMaterial {
            base_color: Color::rgba(1.,1.,1., 0.5),
            // Mask sets a cutoff for transparency. Alpha values below are fully transparent,
            // alpha values above are fully opaque.
            alpha_mode: AlphaMode::Blend,
            double_sided: true,
            cull_mode: None,
            ..default()
        }),
        ..default()
    });
  
    // draw instructions
    commands.spawn(
        TextBundle::from_section(
            "Press 'R' to toggle rest pose mode\n\
            Press 'D' to toggle debug text\n\
            Press 'Left' or 'Right' to increment frames\n\
            Hold 'Down' or 'Up' to change the line width of straight gizmos\n\
            Hold 'Up' or 'Down' to change the line width of round gizmos\n",
            TextStyle {
                font_size: 15.,
                ..default()
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            right: Val::Px(12.0),
            // max_width: Val::Px(200.0),
            ..default()
        })
    );

    // draw debug text
    commands.spawn((
        TextBundle::from_section(
            "Debug text",
            TextStyle {
                font_size: 17.,
                color: Color::rgba(1.0, 1.0, 1.0, 0.5),
                ..default()
            },
        )
        // place top right corner of the screen with text rigth aligned
        .with_style(Style {
            position_type: PositionType::Absolute,
            display: Display::Flex,
        
            // overflow: Overflow::scroll(),

            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        })
        ,DebugText
    ));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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

/// Get the global position and rotation of a joint at a specific frame.
/// If in rest pose mode, return the rest pose global position and rotation independent of frame.
fn get_global_position_rotation(joint_index: Index, frame: usize, app_data: &AppGlobalData) -> (&Position, &Quaternion) {
    let rest_pose_mode = app_data.rest_pose_mode;
    if rest_pose_mode {
        let global_positions = &app_data.anim_data.rest_global_positions;
        let global_rotations = &app_data.anim_data.rest_global_rotations; //TODO CHANGE
        return (&global_positions[joint_index], &global_rotations[joint_index]);
    }
    let global_positions = &app_data.anim_data.pose_global_positions;
    let global_rotations = &app_data.anim_data.pose_global_rotations;
    return (&global_positions[joint_index][frame], &global_rotations[joint_index][frame])
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

fn draw_skeleton(
    mut gizmos: Gizmos,
    appdata: Res<AppGlobalData>,
) {
    let frame = appdata.frame;
    let scale = appdata.scale;

    
    
    //// Draw the skeleton lines
    let kinematic_chain = &appdata.kinematic_chain;
    for chain in kinematic_chain.iter() {
        let positions = chain.iter().map(|&joint_index| {
            let (pos,_) = get_global_position_rotation(joint_index, frame, &appdata);
            Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32) /scale
        }).collect::<Vec<_>>();
        gizmos.linestrip(positions, Color::YELLOW);
    }

    //// Draw the joints as spheres. Draw the axes of the joints.
    let bvh = &appdata.bvh;
    for joint in &bvh.joints {
        let joint_index = joint.index;
        let (pos,rot) = get_global_position_rotation(joint_index, frame, &appdata);
        draw_joint_sphere(&mut gizmos, pos, scale);
        draw_joint_axes(&mut gizmos, rot, pos,scale);
    }

    //// draw identity axes for reference
    draw_joint_axes(&mut gizmos, &Quaternion::identity(), &Position::identity(), 10.0);

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// 
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
        appdata.real_frame += time.delta_seconds_f64() * appdata.bvh.fps as f64;
        
    }

    if keyboard.just_released(KeyCode::ArrowRight) {
        appdata.playing = false;
        appdata.real_frame += 1.0;
    }

    if keyboard.just_released(KeyCode::ArrowLeft) {
        appdata.playing = false;
        appdata.real_frame -= 1.0;
    }

    if keyboard.just_released(KeyCode::KeyD) {
        appdata.debug_text = !appdata.debug_text;
    }

    appdata.real_frame = appdata.real_frame.rem_euclid(appdata.bvh.num_frames as f64);
    appdata.frame = appdata.real_frame.floor() as usize;


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

fn update_debug_text(
    mut query: Query<&mut Text, With<DebugText>>,
    appdata: Res<AppGlobalData>
) {
    // Update the debug text with current global position and rotation data for each joint
    let mut t :String = "".to_string();
    t += &format!("Frame: {}", appdata.frame);
    t += "\n";
    t += "=============== GLOBAL POSITIONS AND ROTATIONS ===============\n";
    for joint in &appdata.bvh.joints {
        let joint_index = joint.index;
        let (pos,rot) = get_global_position_rotation(joint_index, appdata.frame, &appdata);
        // create 3 column table with joint name, position, rotation
        t += &format!("{:.<20} {: ^40} {: ^40}\n", joint.name, format!("{:6.2?}", pos), format!("{:6.2?}", rot));
        t += "\n";
    }
    for mut text in &mut query {
        if appdata.debug_text {
            text.sections[0].value = t.clone();
        } else {
            text.sections[0].value = "".to_string();
        }
    }
}