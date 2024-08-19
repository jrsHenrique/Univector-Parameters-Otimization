//
// Created by igorribeiro on 10/27/16.
//

#include "Ally.h"

namespace representations {

Ally::Ally(int id, RoleList role) {
    this->id = id;
    this->role = role;
    this->pose = Pose2D(0, 0, 0);
    this->velocity = Vector2<double>(0, 0);
    this->visionFound = false;
    this->angleControlledByUnivector = 0;
}
const Vector2<double> &Ally::getVelocity() const {
    return velocity;
}

void Ally::setVelocity(Vector2<double> velocity) {
    this->velocity = velocity;
    double orientationError = fabs(MathUtils::normalizeAngle(velocity.angle() - pose.rotation));
    if (orientationError < M_PI_2)
        linear = velocity.abs();
    else
        linear = -velocity.abs();
}

void Ally::setPosition(Vector2<double> position) {
    this->pose.translation = position;
}

void Ally::setRotation(double rotation) {
    this->pose.rotation = rotation;
}

const double Ally::getRotation() {
    return pose.rotation;
}

void Ally::invert() {
    this->pose.rotation += M_PI;
    this->pose.translation.x *= -1.0;
    this->pose.translation.y *= -1.0;
    this->velocity.x *= -1.0;
    this->velocity.y *= -1.0;
    this->angular *= -1.0;
}

const Vector2<double> &Ally::getPosition() const {
    return pose.translation;
}

bool Ally::operator == (const Ally &other) const {
    return (pose == other.pose         and
            velocity == other.velocity and
            id == other.id             and
            wheelSpeed == other.wheelSpeed);
}

} /* namespace representation */
