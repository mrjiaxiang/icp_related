#include "pose2d.h"

#include <stdio.h>
#include <string.h>

Pose2D::Pose2D() { x = y = phi_ = 0; }

Pose2D::Pose2D(const float _x, const float _y, const float phi) {
    x = _x;
    y = _y;
    SetPhi(phi);
}

// a_c_R = a_b_R * b_c_R
// Pose2D a_c_R = Pose2D(a_b_R)+ Pose2D(b_c_R)

Pose2D Pose2D::operator-() {
    Pose2D b_;
    b_.InverseComposeFrom(Pose2D(0, 0, 0), *this);
    return b_;
}

void Pose2D::operator*=(const float s) {
    x *= s;
    y *= s;
    phi_ *= s;
    NormalizePhi();
}

void Pose2D::NormalizePhi() { phi_ = WrapToPi(phi_); }

void Pose2D::Reset(float _x, float _y, float phi) {
    x    = _x;
    y    = _y;
    phi_ = phi;
}
