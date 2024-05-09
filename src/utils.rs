use crate::types::{BvhMetadata, Joint, Quaternion};


/// reorder vector based on euler angles order string
pub(crate) fn __reorder_vector(e1: f64, e2: f64, e3: f64, order: &str) -> (f64, f64, f64) {
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
pub(crate) fn __from_euler_to_quat(x: f64, y: f64, z: f64, order: &str) -> Quaternion {
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
        "XYZ" => Quaternion::new(
            c1 * c2 * c3 - s1 * s2 * s3,
            s1 * c2 * c3 + c1 * s2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
            c1 * c2 * s3 + s1 * s2 * c3,
        ),
        "YXZ" => Quaternion::new(
            c1 * c2 * c3 + s1 * s2 * s3,
            s1 * c2 * c3 + c1 * s2 * s3,
            c1 * s2 * c3 - s1 * c2 * s3,
            c1 * c2 * s3 - s1 * s2 * c3,
        ),
        "ZXY" => Quaternion::new(
            c1 * c2 * c3 - s1 * s2 * s3,
            s1 * c2 * c3 - c1 * s2 * s3,
            c1 * s2 * c3 + s1 * c2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
        ),
        "ZYX" => Quaternion::new(
            s1 * c2 * c3 - c1 * s2 * s3,
            c1 * s2 * c3 + s1 * c2 * s3,
            c1 * c2 * s3 - s1 * c2 * c3,
            c1 * c2 * c3 + s1 * s2 * s3,
        ),
        "YZX" => Quaternion::new(
            s1 * c2 * c3 + c1 * s2 * s3,
            c1 * s2 * c3 + s1 * c2 * s3,
            c1 * c2 * s3 - s1 * s2 * c3,
            c1 * c2 * c3 - s1 * s2 * s3,
        ),
        "XZY" => Quaternion::new(
            s1 * c2 * c3 - c1 * s2 * s3,
            c1 * s2 * c3 - s1 * c2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
            c1 * c2 * c3 + s1 * s2 * s3,
        ),
        _ => panic!("Invalid euler angles order!"),
    };

    return quaternion;
}


