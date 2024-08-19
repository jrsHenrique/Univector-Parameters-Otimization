//
// Created by thay
//

#include <tools/simulator/server/manager/Manager.h>
#include "representations/Data.h"
#include "representations/Field.h"
#include "physics/group/SimulatorField.h" // this should be elsewhere
#include "gui/OgreManager.h"              // this should be elsewhere
#include "utils/Text.h"
#include "utils/PathHandler.hpp"
#include "utils/Timer.h"
#include "../core/modeling/WorldModel.h"
#include "representations/player/Ally.h"
//#include "representations/player/Opponent.h"
#include "OgreMesh.h"

using itandroids_lib::math::Vector2;
using representations::Data;
using representations::Field;

double randomDouble();
void chooseStartPositions();
void ODEMessageHandler(int errnum, const char *msg, va_list ap);

class UnivectorExtremeAttackersOptimization_AgentTest : public simulator::Manager {
public:
    UnivectorExtremeAttackersOptimization_AgentTest(simulator::Flags flags, int maxTime);
    ~UnivectorExtremeAttackersOptimization_AgentTest() = default;
    void mainThread() override;
    int getMaxTime();
private:
    int maxTime;
};

UnivectorExtremeAttackersOptimization_AgentTest::UnivectorExtremeAttackersOptimization_AgentTest(simulator::Flags flags, int maxTime) : Manager(flags) {
    this->maxTime = maxTime;
}

int UnivectorExtremeAttackersOptimization_AgentTest::getMaxTime() {
    return this->maxTime;
}
//Here starts the reward engineering

//First we start by building up function that will calculate the differences between the angles and the angles of the trajectorys

double calculateAngleDifference(double angle1, double angle2) {
        double diff = std::abs(angle1 - angle2);
        if (diff > M_PI) {
            diff = 2 * M_PI - diff;
        }
        return diff;
}

double calculateDeviation(const std::vector<double>& angles1, const std::vector<double>& angles2) {
        if (angles1.size() != angles2.size()) {
            throw std::runtime_error("Trajectories must have the same length for deviation calculation.");
        }

        double deviation = 0.0;
        for (std::size_t i = 0; i < angles1.size(); ++i) {
            double angleDiff = calculateAngleDifference(angles1[i], angles2[i]);
            deviation += angleDiff;
        }

        return deviation;
}

void UnivectorExtremeAttackersOptimization_AgentTest::mainThread() {
    int maxTime = this->getMaxTime();
    int goalsScored = 0;
    double accumulatedTimeToScore = 0.0;
    // Here we initializate the variables that we will be needing
    double accumulatedDeviation = 0.0;
    double angle1 = 0.0;
    double angle2 = 0.0;
    double angle3 = 0.0;
    double angle4 = 0.0;

    for(int i = 0; i < numberOfMatches and this->running; ++i){
        this->statsManager.addMatch();

        chooseStartPositions();
        this->gameManager.setPreference(simulator::SIDE::LEFT);
        this->resetState();

        bool scoredGoal = false;
        bool concededGoal = false;
        double timeToCompleteSimulation = 0.0;
        //Creates the trajectory of a match
        std::vector<double> trajectory1;
        std::vector<double> trajectory2;
        std::vector<double> trajectory3;
        std::vector<double> trajectory4;

        double iniTime = this->gameManager.getPhysicsTime();
        //Take the simulator info
        modeling::WorldModel& worldModel = modeling::WorldModel::getInstance();
        //Declaring the angles that Univector predicts
        double previousUnivectorControlAngle = 0.0;
        double secondPreviousUnivectorControlAngle = 0.0;

        while(this->gameManager.getPhysicsTime() - iniTime < maxTime and this->running){
            if(this->statsManager.getLeftScore() > 0){
                scoredGoal = true;
                goalsScored++;
                timeToCompleteSimulation = this->gameManager.getPhysicsTime() - iniTime;
                break;
            }
            if(this->statsManager.getRightScore() > 0){
                concededGoal = true;
                timeToCompleteSimulation = 2*maxTime;
                break;
            }
            //Taking the angles
            int robotId = 1; // ID of the robot you're interested in
            shp<representations::Ally> robot = worldModel.getPlayerById(robotId);
            int secondRobotId = 2; // ID of the robot you're interested in
            shp<representations::Ally> secondRobot = worldModel.getPlayerById(secondRobotId);
            angle1 = robot->getRotation();
            angle2 = previousUnivectorControlAngle;
            angle3 = secondRobot->getRotation();
            angle4 = secondPreviousUnivectorControlAngle;
            //Taking the trajectories
            trajectory1.push_back(angle1);
            trajectory2.push_back(angle2);
            trajectory3.push_back(angle3);
            trajectory4.push_back(angle4);

            this->runSimulator();
            //Getting the Univector predict angles for the next iteration
            previousUnivectorControlAngle = robot->angleControlledByUnivector;
            secondPreviousUnivectorControlAngle = secondRobot->angleControlledByUnivector;

        }
        if (not concededGoal and not scoredGoal){
            timeToCompleteSimulation = 1.2*maxTime;
        }
        // Using the implemented functions to calculate the deviations
        accumulatedDeviation += calculateDeviation(trajectory1, trajectory2);
        accumulatedDeviation += calculateDeviation(trajectory3, trajectory4);
        accumulatedTimeToScore += timeToCompleteSimulation;


        this->statsManager.handleMatchData();
    }


    accumulatedDeviation /= numberOfMatches;
    accumulatedTimeToScore /= numberOfMatches;

    std::ofstream outputFile;
    outputFile.open("../source/tools/optimization/univector/cost_extreme_optimization.conf");
    outputFile << accumulatedTimeToScore+accumulatedDeviation << " " << goalsScored << std::endl;

    this->running = false;
}

int main(int argc, char **argv) {
    std::srand(time(NULL));

    simulator::Flags flags = {
                    3,
                    3,
                    3,
                    25,
                    simulator::SIMULATOR_SPEED::ACCELERATED,
                    false,
                    false,
                    simulator::State::CUSTOM
    };

    std::string numberOfMatches = "-matches";
    std::string rightTeamOff = "-rightTeamOff";
    std::string noGUI = "-noGUI";
    std::string silent = "-silent";
    std::string maxTime = "-maxTime";
    int maxTimeValue = 10;

    for(int i = 1; i < argc; ++i){
        if(argv[i] == numberOfMatches and i + 1 < argc){
            flags.numberOfMatches = std::stoi(argv[++i]);
        } else if(argv[i] == rightTeamOff){
            flags.rightPlayers = 0;
        } else if(argv[i] == silent){
            flags.silence = true;
        } else if(argv[i] == noGUI){
            flags.noGUI = true;
        } else if(argv[i] == maxTime and i + 1 < argc){
            maxTimeValue = std::stoi(argv[++i]);
        }
    }

    // sets ODE message handler, to avoid "LCP internal error" message on the terminal :(
    dSetMessageHandler(ODEMessageHandler);

    Data::loadFromFile();
    Data::N_PLAYERS = flags.numberOfPlayers;

    Field::loadFromFile(representations::FIELD_MODE::M3V3);

    if(flags.silence){
        std::cout.setstate(std::ios_base::failbit);
    }

    simulator::OgreManager &ogreManager = simulator::OgreManager::getInstance();
    if (not flags.noGUI)
        ogreManager.setUp();

    simulator::SimulatorField field; // this should be handled
    field.createField();  // elsewhere

    UnivectorExtremeAttackersOptimization_AgentTest extremeAttackersOpt(flags, maxTimeValue);

    std::thread simulationThread;
    simulationThread = std::thread(&UnivectorExtremeAttackersOptimization_AgentTest::mainThread,  &extremeAttackersOpt);

    extremeAttackersOpt.startGUI();
    simulationThread.join();

    return EXIT_SUCCESS;
}

double randomDouble() {
    return (double)std::rand()/(RAND_MAX);
}

void chooseStartPositions(){
    simulator::PositionsFileParser parser("customPositions.conf");

    Vector2<double> ballPos(-Data::BALL_DIAMETER/2 + randomDouble()*(Field::GOAL_LEFT_X + Data::BALL_DIAMETER),
                            Data::BALL_DIAMETER/2 + randomDouble()*(Field::FIELD_LENGTH_Y - Data::BALL_DIAMETER) - Field::FIELD_LENGTH_Y/2);

    parser.ballPositionToParseOut = {ballPos.x, ballPos.y};

    std::vector<Vector2<double>> left(3);
    left[0].x = -0.720;
    left[0].y = 0.0;
    parser.leftTeamToParseOut.emplace_back(-1.5708, left[0]);

    for(int i = 1; i < 3; ++i){
        bool possiblePosition = false;
        while(not possiblePosition){
            left[i].x = -Data::SIZE + randomDouble()*(Field::GOAL_LEFT_X + 2*Data::SIZE);
            left[i].y = Data::SIZE + randomDouble()*(Field::FIELD_LENGTH_Y - 2*Data::SIZE) - Field::FIELD_LENGTH_Y/2;
            if(left[i].distance(ballPos) < Data::BALL_DIAMETER/2 + Data::SIZE){
                possiblePosition = false;
            } else {
                possiblePosition = true;
                for (int j = 0; j < i; ++j) {
                    if (left[i].distance(left[j]) < 2 * Data::SIZE) {
                        possiblePosition = false;
                        break;
                    }
                }
            }
        }
        parser.leftTeamToParseOut.emplace_back(-M_PI + randomDouble()*(2*M_PI), left[i]);
    }

    parser.rightTeamToParseOut.emplace_back(1.5708, 0.720, 0);
    parser.rightTeamToParseOut.emplace_back(0, 0.230, 0);
    parser.rightTeamToParseOut.emplace_back(0, 0.475, 0);

    parser.parseOut();
}

void ODEMessageHandler(int errnum,
                       const char *msg,
                       va_list ap) {
    // Overload function, just to ignore ODE warning messages polluting the terminal :(
}
