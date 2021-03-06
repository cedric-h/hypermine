#![allow(clippy::op_ref)]

use std::ops::Mul;

use na::RealField;
use serde::{Deserialize, Serialize};

use super::{distance, origin, translate, translate_along};

/// A hyperbolic translation followed by a rotation; a direct isometry.
#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq)]
pub struct Isometry<N: RealField> {
    pub translation: na::Vector4<N>,
    pub rotation: na::UnitQuaternion<N>,
}

impl<N: RealField> Isometry<N> {
    pub fn identity() -> Self {
        Self {
            translation: origin(),
            rotation: na::one(),
        }
    }

    #[inline]
    pub fn translation(translation: na::Vector4<N>) -> Self {
        Self::from_parts(translation, na::one())
    }

    #[inline]
    pub fn rotation(rotation: na::UnitQuaternion<N>) -> Self {
        Self::from_parts(origin(), rotation)
    }

    #[inline]
    pub fn from_parts(translation: na::Vector4<N>, rotation: na::UnitQuaternion<N>) -> Self {
        debug_assert!(translation.w != N::zero());
        Self {
            translation,
            rotation,
        }
    }

    /// Construct from a direct isometry encoded as a 4x4 matrix
    pub fn from_homogeneous(m: &na::Matrix4<N>) -> Self {
        let dest = m.index((.., 3));
        let norm = dest.xyz().norm();
        let boost_length = (dest.w + norm).ln();
        let rotation = na::Rotation3::from_matrix_unchecked(if norm == N::zero() {
            m.fixed_slice::<na::U3, na::U3>(0, 0).clone_owned()
        } else {
            let direction = na::Unit::new_unchecked(dest.xyz() / norm);
            let inverse_boost = translate_along(&direction, -boost_length);

            (inverse_boost * m)
                .fixed_slice::<na::U3, na::U3>(0, 0)
                .clone_owned()
        });
        Self {
            translation: dest.clone_owned(),
            rotation: na::UnitQuaternion::from_rotation_matrix(&rotation),
        }
    }

    pub fn to_homogeneous(&self) -> na::Matrix4<N> {
        translate(&origin(), &self.translation) * self.rotation.to_homogeneous()
    }

    pub fn inverse(&self) -> Self {
        let inverse_rot = self.rotation.inverse();
        let inverse_trans = translate(&self.translation, &origin())
            .index((.., 3))
            .clone_owned();
        let v = inverse_rot * inverse_trans.xyz();
        Self {
            rotation: inverse_rot,
            translation: na::Vector4::new(v.x, v.y, v.z, inverse_trans.w),
        }
    }
}

impl<'a, 'b, N: RealField> Mul<&'b na::Vector4<N>> for &'a Isometry<N> {
    type Output = na::Vector4<N>;
    fn mul(self, rhs: &'b na::Vector4<N>) -> Self::Output {
        let rotated = self.rotation * rhs.xyz();
        translate(&origin(), &self.translation)
            * na::Vector4::new(rotated.x, rotated.y, rotated.z, rhs.w)
    }
}

impl<'a, N: RealField> Mul<&'a na::Vector4<N>> for Isometry<N> {
    type Output = na::Vector4<N>;
    #[inline]
    fn mul(self, rhs: &'a na::Vector4<N>) -> Self::Output {
        &self * rhs
    }
}

impl<'a, N: RealField> Mul<na::Vector4<N>> for &'a Isometry<N> {
    type Output = na::Vector4<N>;
    #[inline]
    fn mul(self, rhs: na::Vector4<N>) -> Self::Output {
        self * &rhs
    }
}

impl<N: RealField> Mul<na::Vector4<N>> for Isometry<N> {
    type Output = na::Vector4<N>;
    #[inline]
    fn mul(self, rhs: na::Vector4<N>) -> Self::Output {
        &self * &rhs
    }
}

impl<'a, 'b, N: RealField> Mul<&'b Isometry<N>> for &'a Isometry<N> {
    type Output = Isometry<N>;
    fn mul(self, rhs: &'b Isometry<N>) -> Self::Output {
        let x = rhs
            .rotation
            .inverse_transform_vector(&self.translation.xyz());
        let x = na::Vector4::new(x.x, x.y, x.z, self.translation.w);
        let translation = translate(&origin(), &x) * rhs.translation;

        let (axis, magnitude) = na::Unit::new_and_get(translation.xyz().cross(&x.xyz()));
        let rotation = if magnitude == N::zero() {
            self.rotation * rhs.rotation
        } else {
            let defect = triangle_defect(&x, &translation, &origin());
            self.rotation * rhs.rotation * na::UnitQuaternion::from_axis_angle(&axis, defect)
        };
        Isometry {
            translation,
            rotation,
        }
    }
}

impl<'a, N: RealField> Mul<Isometry<N>> for &'a Isometry<N> {
    type Output = Isometry<N>;
    #[inline]
    fn mul(self, rhs: Isometry<N>) -> Self::Output {
        self * &rhs
    }
}

impl<'a, N: RealField> Mul<&'a Isometry<N>> for Isometry<N> {
    type Output = Isometry<N>;
    #[inline]
    fn mul(self, rhs: &'a Isometry<N>) -> Self::Output {
        &self * rhs
    }
}

impl<N: RealField> Mul<Isometry<N>> for Isometry<N> {
    type Output = Isometry<N>;
    #[inline]
    fn mul(self, rhs: Isometry<N>) -> Self::Output {
        &self * &rhs
    }
}

#[cfg(test)]
impl<N: RealField> approx::AbsDiffEq for Isometry<N> {
    type Epsilon = N;

    fn default_epsilon() -> N {
        N::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: N) -> bool {
        self.translation.abs_diff_eq(&other.translation, epsilon)
            && self.rotation.abs_diff_eq(&other.rotation, epsilon)
    }
}

fn triangle_defect<N: RealField>(
    p0: &na::Vector4<N>,
    p1: &na::Vector4<N>,
    p2: &na::Vector4<N>,
) -> N {
    let a = distance(p0, p1);
    let b = distance(p1, p2);
    let c = distance(p2, p0);
    if a == N::zero() || b == N::zero() || c == N::zero() {
        return N::zero();
    }
    let angle_sum = loc_angle(a, b, c) + loc_angle(b, c, a) + loc_angle(c, a, b);
    N::pi() - angle_sum
}

/// Compute angle at the vertex opposite side `a` using the hyperbolic law of cosines
fn loc_angle<N: RealField>(a: N, b: N, c: N) -> N {
    // cosh a = cosh b cosh c - sinh b sinh c cos θ
    // θ = acos ((cosh b cosh c - cosh a) / (sinh b sinh c))
    let denom = b.sinh() * c.sinh();
    if denom == N::zero() {
        return N::zero();
    }
    na::clamp(
        (b.cosh() * c.cosh() - a.cosh()) / denom,
        -N::one(),
        N::one(),
    )
    .acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    fn simple() {
        assert_abs_diff_eq!(
            Isometry::<f64>::identity() * Isometry::identity(),
            Isometry::identity()
        );

        assert_abs_diff_eq!(
            Isometry::<f64>::identity().to_homogeneous(),
            na::Matrix4::identity()
        );

        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(
            (Isometry::translation(a)).to_homogeneous(),
            translate(&origin(), &a),
            epsilon = 1e-5
        );
    }

    #[test]
    fn rotation_composition() {
        let q = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), 1.0);
        assert_abs_diff_eq!(
            (Isometry::rotation(q) * Isometry::rotation(q)).to_homogeneous(),
            (q * q).to_homogeneous(),
            epsilon = 1e-5
        );
    }

    #[test]
    fn homogenize_distributes() {
        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(0.0, 0.5, 0.0, 1.0);
        assert_abs_diff_eq!(
            (Isometry::translation(a) * Isometry::translation(b)).to_homogeneous(),
            Isometry::translation(a).to_homogeneous() * Isometry::translation(b).to_homogeneous(),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translation_composition() {
        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(0.0, 0.5, 0.0, 1.0);
        assert_abs_diff_eq!(
            (Isometry::translation(a) * Isometry::translation(b)).to_homogeneous(),
            translate(&origin(), &a) * translate(&origin(), &b),
            epsilon = 1e-3
        );
    }

    #[test]
    fn mixed_composition() {
        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let q = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), f64::pi() / 3.0);

        assert_abs_diff_eq!(
            (Isometry::rotation(q) * Isometry::translation(a)).to_homogeneous(),
            q.to_homogeneous() * translate(&origin(), &a),
            epsilon = 1e-3
        );
    }

    #[test]
    fn defect() {
        assert_abs_diff_eq!(triangle_defect::<f64>(&origin(), &origin(), &origin()), 0.0);
        let a = 3.11;
        let b = 4.39;
        let c = 1.95;
        let sum = loc_angle(a, b, c) + loc_angle(b, c, a) + loc_angle(c, a, b);
        assert_abs_diff_eq!(sum, 1.94, epsilon = 1e-2);
    }

    #[test]
    fn compose_identity() {
        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(
            Isometry::translation(a).to_homogeneous(),
            (&Isometry::translation(a) * &Isometry::identity()).to_homogeneous()
        );
        assert_abs_diff_eq!(
            Isometry::translation(a).to_homogeneous(),
            (&Isometry::identity() * &Isometry::translation(a)).to_homogeneous()
        );
    }

    #[test]
    fn from_homogeneous() {
        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let q = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), f64::pi() / 3.0);
        let m = translate(&origin(), &a) * q.to_homogeneous();
        assert_abs_diff_eq!(Isometry::from_homogeneous(&m).to_homogeneous(), m);
    }

    #[test]
    fn from_parts() {
        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let q = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), f64::pi() / 3.0);
        assert_abs_diff_eq!(
            Isometry::from_parts(a, q).to_homogeneous(),
            translate(&origin(), &a) * q.to_homogeneous()
        );
    }

    #[test]
    fn inverse() {
        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let q = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), f64::pi() / 3.0);
        assert_abs_diff_eq!(
            (Isometry::from_parts(a, q) * Isometry::from_parts(a, q).inverse()).to_homogeneous(),
            na::Matrix4::identity()
        );
        assert_abs_diff_eq!(
            (Isometry::from_parts(a, q).inverse() * Isometry::from_parts(a, q)).to_homogeneous(),
            na::Matrix4::identity()
        );
    }
}
