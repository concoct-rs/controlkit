use crate::{from_fn, Task};
use nalgebra::{Vector3, Vector4};
use std::f64::consts::SQRT_2;

/// `kappa`: drag/thrust ratio
pub fn motor_commands<M>(
    collective_thrust_cmd: f64,
    moment_cmd: Vector3<f64>,
    l: f64,
    kappa: f64,
) -> impl Task<M, Output = Vector4<f64>> {
    from_fn(move |_| {
        let length = l / SQRT_2;
        let a = moment_cmd.x / length;
        let b = moment_cmd.y / length;
        let c = -1. * moment_cmd.z / kappa;
        let d = collective_thrust_cmd;
        Vector4::new(
            (a + b + c + d) / 4.,
            (-a + b - c + d) / 4.,
            (a - b - c + d) / 4.,
            (-a - b + c + d) / 4.,
        )
    })
}
