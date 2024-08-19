//
// Created by mmaximo on 7/3/18.
//

#ifndef ITANDROIDS_VSS_UNIVECTORCONTROL_H
#define ITANDROIDS_VSS_UNIVECTORCONTROL_H

#include "control/controllers/AbstractControl.h"
#include "control/controllers/univector/UnivectorControlParams.h"
#include "trajectory_planner/univector/vss/UnivectorVSS.h"
#include "representations/Data.h"
#include "representations/Field.h"
#include "representations/wheel/WheelSpeed.h"
#include "representations/player/Ally.h"

namespace control {

using representations::Data;
using representations::Field;
using representations::Ally;
using representations::WheelSpeed;
using control::AbstractControl;
using decision_making::UnivectorVSS;

class UnivectorControl : public AbstractControl {
public:
    UnivectorControl();
    explicit UnivectorControl(const UnivectorControlParams &params);

    void compute(shp<ControlRequest> data) override;
    void reset(const UnivectorControlParams &params);

private:
    double calcLinearSpeed(double dist, double diffAngle, double aproxSpeed, double linearSpeed);

    UnivectorControlParams params;
    double prevError;
};

} // control

#endif