use bevy::{core::FixedTimestep, prelude::*};
use bevy_rapier3d::{
    physics::{RapierConfiguration, RapierPhysicsPlugin, RigidBodyHandleComponent},
    render::RapierRenderPlugin,
};
use rapier3d::{
    dynamics::{RigidBodyBuilder, RigidBodySet},
    geometry::ColliderBuilder,
    math::Vector,
};

#[bevy_main]
fn main() {
    App::build()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin)
        .add_plugin(RapierRenderPlugin)
        .add_startup_system(spawn_object.system())
        .add_startup_system(add_lighting.system())
        .add_startup_system(add_camera.system())
        .add_startup_system(turn_off_gravity.system())
        .add_system(stabilize.system())
        .add_resource(Perturbed(false))
        .add_resource(TargetOrientation(
            Quat::from_xyzw(1.0, 2.0, 3.0, 4.0).normalize(),
            // Quat::default(), // this allows stabilization
        ))
        .add_stage_after(
            stage::UPDATE,
            "perturb",
            SystemStage::parallel()
                .with_run_criteria(FixedTimestep::step(3.0))
                .with_system(perturb.system()),
        )
        .run();
}

const ITEM_SIZE: f32 = 1.0;

fn spawn_object(commands: &mut Commands) {
    commands.spawn((
        RigidBodyBuilder::new_dynamic(), //.angular_damping(0.1),
        ColliderBuilder::cuboid(ITEM_SIZE / 2.0, ITEM_SIZE / 2.0, ITEM_SIZE / 2.0),
    ));
}

fn add_lighting(commands: &mut Commands) {
    commands.spawn(LightBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 8.0, 0.0)), // meters
        ..Default::default()
    });
}

fn add_camera(commands: &mut Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
            Vec3::new(3.0, 3.0, 3.0),
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::unit_y(),
        )),
        ..Default::default()
    });
}

fn turn_off_gravity(mut physics_config: ResMut<RapierConfiguration>) {
    physics_config.gravity *= 0.0;
}

struct Perturbed(bool);

fn perturb(
    mut perturbed: ResMut<Perturbed>,
    mut bodies: ResMut<RigidBodySet>,
    items: Query<&RigidBodyHandleComponent>,
) {
    if !perturbed.0 {
        perturbed.0 = true;
        for rb_handle in items.iter() {
            if let Some(rb) = bodies.get_mut(rb_handle.handle()) {
                let inertia_sqrt = rb.effective_world_inv_inertia_sqrt.inverse_unchecked();

                let angular_acceleration = Vector::new(10.0, 0.0, 0.0);
                let torque = inertia_sqrt * (inertia_sqrt * angular_acceleration);
                rb.apply_torque_impulse(torque, true);
            }
        }
    }
}

struct TargetOrientation(Quat);

fn stabilize(
    target_orientation: Res<TargetOrientation>,
    parts: Query<(&Transform, &RigidBodyHandleComponent)>,
    mut bodies: ResMut<RigidBodySet>,
) {
    for (transform, part_rb_handle) in parts.iter() {
        if let Some(rb) = bodies.get_mut(part_rb_handle.handle()) {
            let stiffness = 30f32;
            let damping = 2.0 * stiffness.sqrt(); // critically damped
            let rotation_between = target_orientation.0 * transform.rotation.conjugate();
            let (rotation_axis, rotation_angle) = rotation_between.to_axis_angle();
            let rotation_vector = rotation_axis * rotation_angle;
            let rotation_vector =
                Vector::new(rotation_vector.x, rotation_vector.y, rotation_vector.z);

            let angular_acceleration = rotation_vector * stiffness - damping * rb.angvel();
            let inertia_sqrt = rb.effective_world_inv_inertia_sqrt.inverse_unchecked();

            let torque = inertia_sqrt * (inertia_sqrt * angular_acceleration);
            rb.apply_torque(torque, true);
        }
    }
}
