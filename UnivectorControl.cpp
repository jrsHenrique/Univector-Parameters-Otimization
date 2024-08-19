//
// Created by mmaximo on 7/3/18.
//

#include "control/controllers/univector/UnivectorControl.h"

namespace control {

UnivectorControl::UnivectorControl() {
    this->reset(this->params);
}

UnivectorControl::UnivectorControl(const UnivectorControlParams &params) {
    this->reset(params);
}

void UnivectorControl::reset(const UnivectorControlParams& params) {
    this->prevError = 0;
    this->params = params;
}

double UnivectorControl::calcLinearSpeed(double dist, double diffAngle, double aproxSpeed, double linearSpeed) {
    double minDist = 0.10;
    double maxDist = 0.17;

    if (dist > maxDist or diffAngle > 0.85 or aproxSpeed < -0.05)
        return linearSpeed;

    if (dist < minDist)
        return 0.0;

    return (sin((dist - minDist) / (maxDist - minDist) * M_PI - M_PI / 2) + 1) * linearSpeed / 2;
}

void UnivectorControl::compute(shp<ControlRequest> data) {
    static std::ofstream file;
    static const bool SHOULD_LOG = false;

    if (SHOULD_LOG and !file.is_open() and data->player->id == 1)
        file.open("univector_control.txt");

//    std::cout << "Univector control" << std::endl;

    shp<UnivectorControlRequest> univectorData = data;

    univectorData->player->angleControlledByUnivector = univectorData->referenceAngle;

    //Limiting speed
    double linearSpeed = fabs(univectorData->player->linear*10);

    if(linearSpeed < params.maxLinearSpeed)
        linearSpeed = linearSpeed+params.integrationStep;
    if(linearSpeed > params.maxLinearSpeed)
        linearSpeed = params.maxLinearSpeed;

    double rotation = univectorData->player->getRotation();
    Vector2 <double> position = univectorData->player->getPosition();

    //Getting angle error
    double angleError = MathUtils::normalizeAngle(univectorData->referenceAngle - rotation);
    if (std::fabs(angleError) > M_PI_2) {
        angleError = MathUtils::normalizeAngle(angleError + M_PI);
        linearSpeed *= -1.0;
    }

    double dist,
           diffAngle,
           aproxSpeed;
    univectorData->referenceAngle = MathUtils::normalizeAngle(univectorData->referenceAngle);


    //WALL CONDITIONS
    //TOP WALL
    if(univectorData->referenceAngle>=0 || univectorData->referenceAngle<=-M_PI) {
        dist = Field::TOP_WALL_Y - position.y;
        diffAngle = std::min(fabs(rotation - M_PI / 2), fabs(rotation + M_PI / 2));
        aproxSpeed = univectorData->player->getVelocity().y;
        linearSpeed = calcLinearSpeed(dist, diffAngle, aproxSpeed, linearSpeed);

    }
    //BOTTOM WALL
    if(univectorData->referenceAngle<=0 || univectorData->referenceAngle>=M_PI) {
        dist = -Field::BOTTOM_WALL_Y + position.y;
        diffAngle = std::min(fabs(rotation - M_PI / 2), fabs(rotation + M_PI / 2));
        aproxSpeed = -univectorData->player->getVelocity().y;
        linearSpeed = calcLinearSpeed(dist, diffAngle, aproxSpeed, linearSpeed);
    }
    //LEFT WALL
    if(univectorData->referenceAngle>=M_PI/2 || univectorData->referenceAngle<=-M_PI/2) {
        dist = -Field::LEFT_WALL_X + position.x;
        diffAngle = std::min(fabs(rotation - M_PI),
                             std::min(fabs(rotation + M_PI),
                                      fabs(rotation)));
        aproxSpeed = -univectorData->player->getVelocity().x;
        linearSpeed = calcLinearSpeed(dist, diffAngle, aproxSpeed, linearSpeed);
    }

    double linear, angular;

    if (std::fabs(angleError) < params.fullSpeedAngleThreshold) {
        double linearBoost = params.boostGain*(linearSpeed-10 * univectorData->player->linear);
        MathUtils::clamp(linearBoost, -params.maxLinearBoost, params.maxLinearBoost);
        linear = linearSpeed + linearBoost;

//        std::cout << "full speed threshold" << std::endl;

    } else if (std::fabs(angleError) < params.completeStopAngleThreshold) {
        double m = -linearSpeed / (params.completeStopAngleThreshold - params.fullSpeedAngleThreshold);
        linear = linearSpeed + m * (std::abs(angleError) - params.fullSpeedAngleThreshold);

//        std::cout << "complete stop" << std::endl;
    } else
        linear = 0.0;

    //Calculating feed forward
//    double feedForwardCorrection = feedForward(player, univectorPlanner);
    double feedForwardCorrection = params.feedforwardGain * data->angularFeedforward;

    //Calculating commanded angular speed
    angular = params.angleFeedbackGain * angleError + params.derivativeGain*(angleError-prevError)
            - params.rateFeedbackGain * univectorData->player->angular+feedForwardCorrection;

    //Saving error
    this->prevError = angleError;

    Vector2d input;
    input << linear, angular;
    linear  = input(0);
    angular = input(1);

    desiredWheelSpeed.setRobotSpeed(linear/ (30/M_PI), angular/ (30/M_PI));
}

} // control
