//
// Created by igorribeiro on 10/27/16.
//

#ifndef CORE_REPRESENTATIONS_PLAYER_H_
#define CORE_REPRESENTATIONS_PLAYER_H_

#include "math/Vector2.h"
#include "math/Pose2D.h"
#include "math/MathUtils.h"
#include "representations/wheel/WheelSpeed.h"
#include "decision_making/coaches/RoleList.h"
#include "Eigen/Dense"

namespace representations {

using itandroids_lib::math::Vector2;
using itandroids_lib::math::Pose2D;
using itandroids_lib::math::MathUtils;
using decision_making::RoleList;

class Ally {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 5, 1> fullState;

    explicit Ally(int id = 1, RoleList role = RoleList::STRIKER);
    virtual ~Ally() = default;

    // Sets the pose of the player in cartesian coordinates relative to the field center
    // translation of the player and rotation are set to 0
    void setPosition(Vector2<double> position);

    void setRotation(double rotation);

    // Gets the position of the player in cartesian coordinates relative to the field center
    const Vector2<double> &getPosition() const;

    // Gets the linear velocity of the player as a vector2
    const Vector2<double> &getVelocity() const;

    // Sets the linear velocity of the player as a vector2
    void setVelocity(Vector2<double> velocity);

    // Gets the rotation of the player in radians
    const double getRotation();
    void invert();

    // Comparison of another Player with this one.
    bool operator == (const Ally &other) const;

    constexpr static int PLAYERS_PER_SIDE = 5; // Number of players on each team

    int id; // Identifier of the player. Non-identified players value set to 0.
    RoleList role;
    Pose2D pose; // Position of the player in cartesian coordinates relative to the field center
    double angular; // Angular velocity
    double linear;  // Linear velocity
    WheelSpeed wheelSpeed;
    bool visionFound;
    double angleControlledByUnivector;

private:
    Vector2<double> velocity;
};

} // representation

#endif
