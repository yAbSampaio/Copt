/********************************************************************************
 *  FARSA - Total99                                                             *
 *  Copyright (C) 2005-2011 Gianluca Massera <emmegian@yahoo.it>                *
 *                                                                              *
 *  This program is free software; you can redistribute it and/or modify        *
 *  it under the terms of the GNU General Public License as published by        *
 *  the Free Software Foundation; either version 2 of the License, or           *
 *  (at your option) any later version.                                         *
 *                                                                              *
 *  This program is distributed in the hope that it will be useful,             *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 *  GNU General Public License for more details.                                *
 *                                                                              *
 *  You should have received a copy of the GNU General Public License           *
 *  along with this program; if not, write to the Free Software                 *
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA  *
 ********************************************************************************/

#include <random>
#include <utility>
#include <chrono>

#include "evo/marxbotcleaningexperiment.h"
#include "utilitiesexceptions.h"
#include "randomgenerator.h"
#include "robots.h"
#include "world.h"
#include "phybox.h"
#include "wheeledexperimenthelper.h"
#include "logger.h"
#include "configurationhelper.h"
#include "helperresources.h"
#include "evoga.h"

#include "environment/room.h"
#include "environment/definitions/direction.h"
#include "environment/definitions/util.h"
// This is needed because the isnan and isinf
// functions are not present windows

#ifdef WIN32
#include <float.h>
#define isnan(x) _isnan(x)
#define isinf(x) (!_finite(x))
#else
#define isnan(x) std::isnan(x)
#define isinf(x) std::isinf(x)
#endif
MarxBotCleaningExperiment::MarxBotCleaningExperiment() : farsa::EvoRobotExperiment(),
                                                         m_wallThickness(0.002f),
                                                         m_objectHeights(0.15f),
                                                         m_wallsvar(0.5f),
                                                         m_shortwallmlength(3.0f),
                                                         m_longwallmlength(3.5f),
                                                         m_cellsize(0.2),
                                                         m_recreateWorld(true),
                                                         nbcyl(10),
                                                         m_placedCyl(0),
                                                         m_numIndividuals(3),
                                                         m_individualSelected(0),
                                                         m_testedIndividual(-1),
                                                         m_numStepsActive(0),
                                                         m_changeBehaviours(1),
                                                         m_numStepsChange(750),
                                                         m_energyLevel(1000),
                                                         m_rechargeRate(0.01),
                                                         m_stepsCounter(0),
                                                         m_changePenaltyFitness(0),
                                                         m_changePenaltyEnergy(0),
                                                         m_changeProbability(100),
                                                         m_generalCounter(0),
                                                         m_considerEnergy(true),
                                                         m_multiRooms(1),
                                                         m_visitedXcell(-1),
                                                         m_visitedYcell(-1),
                                                         m_minPolicyOperation(0),
                                                         m_maxPolicyOperation(0),
                                                         m_saveTestStats(false),
                                                         m_varyNonLinear(0),
                                                         enable_grid_(false),
                                                         data_dir_(""),
                                                         save_data_(false),
                                                         append_data_(false),
                                                         fk_raise_function_("linear"),
                                                         fk_fall_function_("linear"),
                                                         fk_steps_raise_(0),
                                                         fk_steps_fall_(0),
                                                         clock_up_(false),
                                                         threshold_(0.75)
{
    // No need to add resources, they are added by EvoRobotExperiment
    // this is required for the fake sensor encoding
    // the amount of recently collected garbage
    addUsableResource("agent[0]:additionalInputs");
    addUsableResource("agent[0]:additionalOutputs");
}
MarxBotCleaningExperiment::~MarxBotCleaningExperiment() {}
void MarxBotCleaningExperiment::configure(
    farsa::ConfigurationParameters &params, QString prefix)
{
    // Calling parent function
    farsa::EvoRobotExperiment::configure(params, prefix);
    //std::cout << farsa::PhyMarXbot::wheelr << std::endl;
    // Loading our parameters.
    // Checks on m_playgroundWidth and m_playgroundHeight
    // will be done in setupArena() because
    // we need a valid object
    m_shortwallmlength = farsa::ConfigurationHelper::getDouble(params,
                                                               prefix + "shortwallmlength", m_shortwallmlength);
    m_longwallmlength = farsa::ConfigurationHelper::getDouble(params,
                                                              prefix + "longwallmlength", m_longwallmlength);
    m_changeBehaviours = farsa::ConfigurationHelper::getInt(params,
                                                            prefix + "changeBehaviours", m_changeBehaviours);
    m_numStepsChange = farsa::ConfigurationHelper::getInt(params,
                                                          prefix + "numStepsChange", m_numStepsChange);
    m_energyLevel = farsa::ConfigurationHelper::getInt(params,
                                                       prefix + "energyLevel", m_energyLevel);
    m_rechargeRate = farsa::ConfigurationHelper::getDouble(params,
                                                           prefix + "rechargeRate", m_rechargeRate);
    m_changePenaltyFitness = farsa::ConfigurationHelper::getDouble(params,
                                                                   prefix + "changePenaltyFitness", m_changePenaltyFitness);
    m_changePenaltyEnergy = farsa::ConfigurationHelper::getDouble(params,
                                                                  prefix + "changePenaltyEnergy", m_changePenaltyEnergy);
    m_changeProbability = farsa::ConfigurationHelper::getInt(params,
                                                             prefix + "changeProbability", m_changeProbability);
    m_considerEnergy = farsa::ConfigurationHelper::getBool(params,
                                                           prefix + "considerEnergy", m_considerEnergy);
    m_multiRooms = farsa::ConfigurationHelper::getInt(params,
                                                      prefix + "multipleRooms", m_multiRooms);
    m_minPolicyOperation = farsa::ConfigurationHelper::getInt(params,
                                                              prefix + "minPolicyOperation", m_minPolicyOperation);
    m_maxPolicyOperation = farsa::ConfigurationHelper::getInt(params,
                                                              prefix + "maxPolicyOperation", m_maxPolicyOperation);
    m_saveTestStats = farsa::ConfigurationHelper::getBool(params,
                                                          prefix + "saveTestStats", m_saveTestStats);
    m_varyNonLinear = farsa::ConfigurationHelper::getInt(params,
                                                         prefix + "varyTemporalNonLinear", m_varyNonLinear);
    enable_grid_ = farsa::ConfigurationHelper::getBool(params,
                                                       prefix + "enable_grid", enable_grid_);
    save_data_ = farsa::ConfigurationHelper::getBool(params,
                                                     prefix + "save_data", save_data_);
    append_data_ = farsa::ConfigurationHelper::getBool(params,
                                                       prefix + "append_data", append_data_);
    data_dir_ = farsa::ConfigurationHelper::getString(params,
                                                      prefix + "data_dir", data_dir_);
    fk_raise_function_ = farsa::ConfigurationHelper::getString(params,
                                                               prefix + "fk_raise_function", fk_raise_function_);
    fk_fall_function_ = farsa::ConfigurationHelper::getString(params,
                                                              prefix + "fk_fall_function", fk_fall_function_);
    fk_steps_raise_ = farsa::ConfigurationHelper::getInt(params,
                                                         prefix + "fk_steps_raise", fk_steps_raise_);
    fk_steps_fall_ = farsa::ConfigurationHelper::getInt(params,
                                                        prefix + "fk_steps_fall", fk_steps_fall_);
    threshold_ = farsa::ConfigurationHelper::getDouble(params,
                                                       prefix + "threshold", threshold_);
    rechargeStation = NULL;
    m_testFilename = farsa::ConfigurationHelper::getString(params,
                                                           prefix + "testFilename", m_testFilename);
}
void MarxBotCleaningExperiment::save(farsa::ConfigurationParameters &params,
                                     QString prefix)
{
    // Calling parent function
    farsa::EvoRobotExperiment::save(params, prefix);
    farsa::Logger::error("NOT IMPLEMENTED (MarxBotCleaningExperiment::save)");
    abort();
}
void MarxBotCleaningExperiment::describe(QString type)
{
    // Calling parent function
    farsa::EvoRobotExperiment::describe(type);

    Descriptor d = addTypeDescription(type, "A MarxBot has to visit the "
                                            "largest possible number of location in the environment ");
    d.describeReal("shortwallmlength").def(1.5).limits(0.0, +Infinity).help("The minimum length of short walls", "This is the minimum length of short walls. The default is 1.5m");
    d.describeReal("longwallmlength").def(2.0).limits(0.0, +Infinity).help("The minimum length of long walls", "This is the minimum length of long walls. The default is 1.9m");
    d.describeReal("m_wallsvar").def(0.5).limits(0.0, +Infinity).help("The range of variation of walls", "This is the range of variation of the length of walls."
                                                                                                         " The default is 0.5m");
    d.describeInt("changeBehaviours").def(0).limits(0, 3).help("How behaviours should be changed over trials.", "0 - do not change; 1 - changing each N steps"
                                                                                                                "(specializer steady state must be selected);"
                                                                                                                " 2 - changing according to gating neuron (SSS);"
                                                                                                                " 3 - gating and counting steps");
    d.describeInt("numStepsChange").def(750).limits(0, MaxInteger).help("How many timesteps each policy should operate", "This parameter is useful when the parameter"
                                                                                                                         " changeBehaviours is set to 1");
    d.describeInt("energyLevel").def(1000).limits(0, MaxInteger).help("The energy level that the Robot has to operate.", "The energy consumption is determined by the motor"
                                                                                                                         " activation.");
    d.describeReal("rechargeRate").def(0.01).limits(0, 1).help("How much will be recharged each time step when the"
                                                               " robot is at the recharge station",
                                                               "This value is the percentage of total energy level that"
                                                               " will be recharged at each time step");
    d.describeReal("changePenaltyFitness").def(0.0).limits(0.0, +Infinity).help("Penalty fitness for changing behaviour", "How much fitness is lost each time the behaviour is changed");
    d.describeReal("changePenaltyEnergy").def(0.0).limits(0.0, +Infinity).help("Penalty energy for changing behaviour", "How much energy is lost each time the behaviour is changed");
    d.describeInt("changeProbability").def(100).limits(0, 100).help("The probability of changing behaviour.", "The probability of changing behaviour when the gating"
                                                                                                              " mechanism say that it should be changed.");
    d.describeBool("considerEnergy").def(true).help("Wheather the energy level should be used"
                                                    " to stop the simulation or not.");
    d.describeInt("multipleRooms").def(1).limits(1, 4).help("1 = convex and concave environments,"
                                                            " 2 = only convex with obstacles,"
                                                            " 3 = only concave, 4 = only convex without obstacles.");
    d.describeInt("minPolicyOperation").def(350).help("The minimum number of steps that a policy can operate.");
    d.describeInt("maxPolicyOperation").def(2667).help("The maximum number of steps that a policy can operate.");
    d.describeString("testFilename")
        .help("The file name in which the test results will be saved.");
    d.describeBool("enable_grid").def(false).help("Show monochrome grid with progress of room cleaning"
                                                  " (White = has been cleared, Black = has not been cleaned).");
    d.describeBool("save_data").def(false).help("Save robot status over trials");
    d.describeBool("append_data").def(false).help("Append status over trials in file or create from scratch.");
    d.describeString("data_dir")
        .help("Directory where the simulation data will be saved.");
    d.describeInt("fk_steps_raise").def(2000).help("Numeric Interval");
    d.describeInt("fk_steps_fall").def(20000).help("Numeric Interval");
    d.describeEnum("fk_raise_function")
        .def("Linear")
        .values(QStringList() << "linear"
                              << "exponential"
                              << "heaviside")
        .props(IsMandatory)
        .help("function change clock internal type.");
    d.describeEnum("fk_fall_function")
        .def("Linear")
        .values(QStringList() << "linear"
                              << "exponential")
        .props(IsMandatory)
        .help("function change clock internal type.");
    d.describeReal("threshold").def(0.75).limits(0.5, 0.99).help("Treshold to save robot positions");
}
void MarxBotCleaningExperiment::postConfigureInitialization()
{
    // Calling parent function
    EvoRobotExperiment::postConfigureInitialization();
    debug = 0;
    // Here we create the arena and the objects
    trials_.resize(3);
    trials_[0].resize(20);
    trials_[1].resize(2 * getNSteps());
    createRooms();
    expseed = 1;
}
void MarxBotCleaningExperiment::initGeneration(int generation)
{
    expseed = generation + 1;
    if (debug == 1)
    {
        expseed = 1;
        std::cout << QString("Generation %1\n").arg(generation).toUtf8().constData();
    }
    // getting the network weights from individuals
}
void MarxBotCleaningExperiment::initIndividual(int individual)
{
    if (debug == 1)
    {
        std::cout << QString("Individual %1\n").arg(individual).toUtf8().constData();
    }
    // Resetting the world to avoid numerical problems
    m_recreateWorld = true;
    m_testedIndividual = individual;
}
void MarxBotCleaningExperiment::initTrial(int trial)
{
    Turning = False;
	Running = False;
	effect = 0; //what the robot need to do on current step
	PositionInTheCorner = 0; //an assist for the 'effect' when 'effect' is equal zero
	last_track = -1;//the value of 'last_track' is '-1' for right, and '1' for left
	deg = 180;
	DegStep = 90;
	effect2 = 6;
	walk = 1;
	teste = 1;
    
    m_trial = trial;
    this->trial = trial;
    // destroy arena objects
    destroyRooms();
    clock_up_ = false;
    // inicializando as funcoes
    fk_a_ = 0;
    fk_b_ = fk_steps_fall_;
    // reset the s>eed so that all individuals experience the same randomly
    // generated environment in a given generation
    // also individual of successive generations mostly experience
    // the same environments (only the environment of one trial change).
    if (getActivityPhase() == INTEST)
    {
        locRNG.setSeed(99 + trial);
        srand(99 + trial);
    }
    else
    {
        locRNG.setSeed(expseed + trial);
    }
    if (debug == 1)
    {
        printf("trial seed (world) %d - ", expseed + trial);
        // the seed that impact on noise generation
        farsa::globalRNG->setSeed(1);
    }
    trialFitnessValue = 0.0;
    int recreate = 0;
    // For some reason it generate segmentation faults,
    // at least time to time, commented.
    if (m_recreateWorld)
    {
        recreateWorld();
        recreateRobot();
        recreateArena();
        m_recreateWorld = false;
        recreate = 1;
    }
    // Reset inital energy.
    // Energy is decrease directly by the modular wheel motor
    m_energy = m_energyLevel;
    // reset step counter
    m_modstep = 0;
    // Recreate the arena at the beginning of every trial
    createRooms();
    // initializeGrid();
    placeRobot(trial);
    // Resetting fitness for the current trial
    trialFitnessValue = 0.0;
    farsa::ResourcesLocker locker(this);
    farsa::ResourceVector<farsa::real> *m_additionalInputs =
        getResource<farsa::ResourceVector<farsa::real>>("agent[0]:additionalInputs");
    // put one to the time counter
    m_generalCounter = 0;
    // when the fake neuron is being directly used to modulate the behaviour
    if (m_additionalInputs->size() >= 1)
    {
        // temporal neuron associated with the dust sensor
        (*m_additionalInputs)[0] = 1.0;
        clock_up_ = false;
    }
    if (m_additionalInputs->size() == 2)
    {
        if (m_multiRooms == 1)
        {
            (*m_additionalInputs)[1] = trial % 2;
        }
        else
        {
            (*m_additionalInputs)[1] = 0;
        }
    }
    else if (m_additionalInputs->size() >= 3)
    {
        // when there is a hidden neuron that modulates the behaviour
        // based on the sensory system and the fake neurons.
        (*m_additionalInputs)[1] = 1.0; // dust sensor.
        (*m_additionalInputs)[2] = 1.0; // steps in the same cell.
    }
    m_stepsSameCell = 0;
    m_numStepsActive = 0;
}

void MarxBotCleaningExperiment::initStep(int step)
{
    // Checking we don't have any NaN.
    // We just check a value in a matrix because NaN spreads rapidly.
    // If there is a NaN, we simply restart the trial from scratch.
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    // farsa::Arena* arena = getResource<farsa::Arena>("arena");
    farsa::ResourceVector<farsa::real> *m_additionalInputs =
        getResource<farsa::ResourceVector<farsa::real>>("agent[0]:additionalInputs");

    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);

    const farsa::real valueToCheck = robot->position().x;
    if (isnan(valueToCheck) || isinf(valueToCheck))
    {
        farsa::Logger::warning("Found a NaN value, recreating world and restart"
                               "ing trial from scratch");
        restartTrial();
        m_recreateWorld = true;
        return;
    }
    environment::Room *actualy_room = nullptr;
    for (auto room : rooms)
    {
        int query = room->contains(robot);
        if (query == environment::definitions::NO_OVERLAP)
        {
            continue;
        }
        else
        {
            actualy_room = room;
            break;
        }
    }
    // cleaned_rooms_[rooms.first()] = true; // set up cleaned first room;
    if (!actualy_room)
    {
        std::cout << "Error!!!\n";
        restartTrial();
        return;
    }
    // when the fake neuron is being directly used to modulate the behaviour
    // comportamento manipulado.
    if (m_additionalInputs->size() >= 1)
    {
        farsa::real *clock = &(*m_additionalInputs)[0];
        // *clock += 1;
        if (old_room_ != actualy_room) {
            if (!cleaned_rooms_[actualy_room]) {
                fk_a_ = step;
                fk_b_ = step + fk_steps_raise_;
                clock_up_ = true;
                cleaned_rooms_[actualy_room] = true;
            }
        }

        double t = static_cast<double>(step - fk_a_)/static_cast<double>(fk_b_ - fk_a_);
        if (step <= fk_b_) {
            if (clock_up_) {
                if (fk_raise_function_ == "exponential") {
                    *clock = pow(2, t) - 1;
                } else if (fk_raise_function_ == "linear") {
                    *clock = t;
                } else {
                    *clock = 1;
                }
            } else {
                if (fk_fall_function_ == "exponential") {
                    *clock = pow(0.15, t);
                } else {
                    *clock = -t + 1;
                }
            }
        } else {
            *clock = 0.5;
        }
        if (*clock >= 0.99 && clock_up_) {
            fk_a_ = step;
            fk_b_ = step+ fk_steps_fall_;
            clock_up_ = false;
        }
    }

    // when there is a hidden neuron that modulates the behaviour
    // based on the sensory system and the fake neurons
    if (m_additionalInputs->size() >= 3)
    {
        if ((*m_additionalInputs)[1] <= 0)
        {
            (*m_additionalInputs)[1] = 1.0;
        }
        else
        {
            (*m_additionalInputs)[1] -= 2.0 * (1.0 / getNSteps());
        }
        if ((*m_additionalInputs)[2] <= 0)
        {
            (*m_additionalInputs)[2] = 1.0;
        }
        else
        {
            (*m_additionalInputs)[2] -= 3.0 * (1.0 / getNSteps());
        }
    }
    old_room_ = actualy_room;
}
void MarxBotCleaningExperiment::afterSensorsUpdate() {}
void MarxBotCleaningExperiment::beforeMotorsUpdate() {}
void MarxBotCleaningExperiment::beforeWorldAdvance() {}
void MarxBotCleaningExperiment::endStep(int step)
{
    int cx, cy;
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    farsa::ResourceVector<farsa::real> *m_additionalInputs;
    m_additionalInputs = getResource<farsa::ResourceVector<farsa::real>>("agent[0]:additionalInputs");
    // PRINT_DEV << "position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << PRINTEND_DEV;
    // CleamRoomHardMode();
    Cleaning();

    environment::Room *root = nullptr;
    int query;
    // Stop the trial when the robot runs out of energy
    if (m_energy <= 0.0 && m_considerEnergy)
    {
        stopTrial();
        return;
    }
    // compute the fitness
    for (environment::Room *room : rooms)
    {
        query = room->contains(robot);
        if (query == environment::definitions::NO_OVERLAP)
        {
            continue;
        }
        else
        {
            room->overlap_cords(robot, query, cx, cy);
            if (query == environment::definitions::OVERLAP)
            {
                query = 0;
            }
            else
            {
                query++;
            }
            root = room;
            break;
        }
    }
    // check if the robot is in the same cell
    // and increment the counter or make it zero
    if (cx == m_visitedXcell && cy == m_visitedYcell)
    {
        m_stepsSameCell++;
    }
    else
    {
        m_stepsSameCell = 0;
    }
    if (root != nullptr)
    {
        if (!root->visited_floor(cx, cy, query))
        {
            root->visite_floor(cx, cy, query);
            trialFitnessValue += 1.0;
            laststepfit[step % 25] = 1;
            if (getActivityPhase() == INTEST && root->enable_grid())
            {
                if (cx < root->floor_grid()[query].size())
                {
                    if (root->floor_grid()[query][cx].size() > cy)
                    {
                        if (root->floor_grid()[query][cx][cy] != nullptr)
                        {
                            root->floor_grid()[query][cx][cy]
                                ->setColor(Qt::white);
                        }
                    }
                }
            }
            m_generalCounter = 0;
            root = nullptr;
        }
        else
        {
            laststepfit[step % 25] = 0;
            m_generalCounter = -(1.0 / getNSteps()) / 4;
            // check if the robot is in a new cell and decrease the temporal
            // counter since the robot is visiting an already visited area,
            // so the probability of changing behaviour should increase.
            if (cx != m_visitedXcell || cy != m_visitedYcell)
            {
                // if dust was not found decrement the temporal inpu
                m_generalCounter -= 15 * (1.0 / getNSteps());
            }
        }
    }
    m_visitedXcell = cx;
    m_visitedYcell = cy;
    m_modstep++;
    //if (trials_.size() > m_trial-1) {
    trials_[1][2 * step] = robot->position().x;
    trials_[1][2 * step + 1] = robot->position().y;
    //}
    double degrees = robot->orientation(arena->getPlane()) + (PI_GRECO / 2);
    // if (step == 0)
    //     std::cout << "Orientation: " <<  degrees << std::endl;
}
void MarxBotCleaningExperiment::endTrial(int trial)
{
    double total_fitness = 0;
    if (getActivityPhase() == INTEST)
    {
        int cont = 0;
        for (int i = 0; i < rooms.size(); i++)
        {
            auto room = rooms[i];
            farsa::Logger::info(QString("Area %1% of room %2 cleaned.")
                                    .arg(100.0 * room->fit())
                                    .arg(++cont));
            // Add room infos
            trials_[0][4 * i] = room->geometry().x();
            trials_[0][4 * i + 1] = room->geometry().y();
            trials_[0][4 * i + 2] = room->size_w();
            trials_[0][4 * i + 3] = room->size_h();
            total_fitness += room->fit();
        }
        if (save_data_)
        {
            savefile(total_fitness / static_cast<double>(rooms.size()));
        }
    }
}

void MarxBotCleaningExperiment::savefile(double total_fitness)
{
    QString trial_divisor("=");
    std::cout << QString("Fitness : %1").arg(total_fitness).toUtf8().constData() << std::endl;

    QString atual_dir = QFileInfo(".").absolutePath() + "/";
    QFile position(atual_dir + data_dir_ + "position_robot.csv");
    QFile trial_info(atual_dir + data_dir_ + "trial_info.csv");

    auto open_mode = QIODevice::Append | QIODevice::Text;
    if (!append_data_)
    {
        auto open_mode = QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text;
    }
    if (!position.open(open_mode) || !trial_info.open(open_mode))
    {
        return;
    }

    QTextStream out_position(&position);
    QTextStream out_trial_info(&trial_info);

    QString out_pos;
    QString out_infos_header = "";
    QString out_infos_tail = "";

    out_infos_header += "total_fitness;rooms;";
    for (int i = 0; i < 5; i++)
    {
        out_infos_header += QString("room_%1_x;room_%1_y;room_%1_w;room_%1_h;").arg(i + 1);
    }

    out_infos_tail += QString("%1;%2;").arg(total_fitness).arg(rooms.size());
    for (int j = 0; j < rooms.size() * 4; j++)
    {
        out_infos_tail += QString("%1;").arg(trials_[0][j]);
    }
    for (int j = 20 - 4 * rooms.size(); j > 0; j--)
    {
        out_infos_tail += ";";
    }
    out_infos_tail += "\n";
    out_infos_header += "\n";
    if (m_trial < 1)
    {
        if (!append_data_)
        {
            out_trial_info << out_infos_header;
        }
    }

    if (total_fitness >= threshold_)
    {
        //  threshold to save position robot
        out_position << "=x;y\n"; //  Header
        for (int i = 0; i < getNSteps(); i++)
        {
            double x = trials_[1][2 * i];
            double y = trials_[1][2 * i + 1];
            out_position << x << ";" << y << "\n";
        }
        out_position << out_pos;
    }
    out_trial_info << out_infos_tail;
}

void MarxBotCleaningExperiment::endIndividual(int /*individual*/)
{
    totalFitnessValue = totalFitnessValue / farsa::real(getNTrials());
    if (debug == 1)
    {
        std::cout << QString("tot fit %1\n").arg(totalFitnessValue).toUtf8().constData();
    }
}
void MarxBotCleaningExperiment::endGeneration(int /*generation*/)
{
    expseed++;
}
farsa::real
MarxBotCleaningExperiment::ffitness(farsa::RobotOnPlane *robot) const
{
    const farsa::wVector robotPosition(robot->position().x,
                                       robot->position().y, 0.0);
    return 0.0;
}
void MarxBotCleaningExperiment::destroyRooms()
{
    farsa::ResourcesLocker locker(this);
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    for (auto room : rooms)
    {
        room->undraw();
    }
    rooms.resize(0);
}
void MarxBotCleaningExperiment::placeRobot(int trial)
{
    int nattempts;
    int locationinside;
    double x, y;
    environment::Room *root_room = rooms.first();
    farsa::ResourcesLocker locker(this);
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    // Now situating the robot in a random location
    // providing that it does not collide with obstacles
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    nattempts = 0;
    farsa::wVector robotP;
    // srand (static_cast <unsigned> (time(0)));
    // int i = 0;
    // for (auto room : rooms){
    //     i++;
    // }
    // int numrand = (rand()%i); 
    // //rand
    // i = 0;
    // for (auto room : rooms){
    //     if (i == numrand)// == rand
    //     {
    //         root_room = room;
    //     }
        
    //     i++;
    // }

    do
    {
        x = locRNG.getDouble(root_room->walls()[environment::definitions::Direction::LEFT]->points()[0][0] + 1.6,
                             root_room->walls()[environment::definitions::Direction::RIGHT]->points()[0][0] -
                                 (robot->robotRadius() + 0.07));
        y = locRNG.getDouble(root_room->walls()[environment::definitions::Direction::BOTTOM]->points()[0][1] +
                                 (robot->robotRadius() + 0.77),
                             root_room->walls()[environment::definitions::Direction::TOP]->points()[0][1] -
                                 (robot->robotRadius() + 0.07));
        locationinside = 0;
        robotP.x = x;
        robotP.y = y;
        for (auto room : rooms)
        {
            bool find = false;
            // verify colission with placed cylinders
            for (auto cylinder : room->cylinders())
            {
                if ((robotP - cylinder->position()).norm() < 0.10)
                {
                    locationinside = 1;
                    find = true;
                    break;
                }
            }
            if (find)
            {
                break;
            }
        }
        nattempts++;
    } while (locationinside == 1 && nattempts < 10000);
    if (nattempts >= 10000)
    {
        farsa::Logger::error("Unable to place the robot in random non-colliding"
                             " location after 10000 attempts");
    }
    else
    {
        //robot->setPosition(arena->getPlane(), x, y);
        // robot->setOrientation(arena->getPlane(),0);
    }
    //                 locRNG.getDouble(-PI_GRECO, PI_GRECO));
    //robot->setPosition(arena->getPlane(), 0, 0);
    //robot->setOrientation(arena->getPlane(), 0);

    float x0_roomX = root_room->geometry().x() - root_room->size_w() / 2 + 0.17;
    float y0_roomX = root_room->geometry().y() - root_room->size_h() / 2 + 0.17;
    float xpos = x0_roomX + ( ((float)rand()/(float)(RAND_MAX)) * (root_room->size_w() -0.17) );
    float ypos = y0_roomX + ( ((float)rand()/(float)(RAND_MAX)) * (root_room->size_h() -0.17) );
    // std::cout<< "SetPosition: (" << xpos << ", " << ypos << ")" << std::endl;
    // std::cout<< "Point Zero: (" << x0_roomX << ", " << x0_roomX << ")" << std::endl;
    // std::cout<<((float)rand()/(float)(RAND_MAX))<<std::endl;

    // robot->setPosition(arena->getPlane(), xpos, ypos);
    robot->setPosition(arena->getPlane(), -1, 1);
    robot->setOrientation(arena->getPlane(), 0);
}
void MarxBotCleaningExperiment::createRooms()
{
    farsa::ResourcesLocker locker(this);
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    // inicio das constantes
    std::random_device rd;
    double mean = static_cast<double>(rd.max() - rd.min()) / 2;
    const float min_w = 3;
    const float min_h = 3;
    const float door_size = 0.25;
    const float min_dist_between_rooms = 1;
    const bool top_r = rd() > mean;
    const bool bottom_r = rd() > mean;
    const QVector<double> sigma{1.5, 1.5, 0};
    // fim das constantes
    environment::Room *center;
    environment::Room *left;
    environment::Room *top;
    environment::Room *bottom;
    environment::Room *right;
    center = new environment::Room(QRect(0, 0, min_w, min_h),
                                   arena,
                                   m_wallThickness,
                                   m_objectHeights,
                                   m_cellsize,
                                   door_size,
                                   sigma);
    left = new environment::Room(QRect(-min_w - min_dist_between_rooms -
                                           sigma[0],
                                       0, min_w, min_h),
                                 arena, m_wallThickness, m_objectHeights,
                                 m_cellsize, door_size, sigma);
    right = new environment::Room(QRect(min_w + min_dist_between_rooms +
                                            sigma[0],
                                        0, min_w, min_h),
                                  arena, m_wallThickness, m_objectHeights,
                                  m_cellsize, door_size, sigma);
    top = new environment::Room(QRect(0, min_h + min_dist_between_rooms + sigma[1], min_w, min_h),
                                arena, m_wallThickness, m_objectHeights,
                                m_cellsize, door_size, sigma);
    bottom = new environment::Room(QRect(0, -min_h - min_dist_between_rooms - sigma[1], min_w, min_h),
                                   arena, m_wallThickness, m_objectHeights,
                                   m_cellsize, door_size, sigma);
    center->set_enable_grid(enable_grid_);
    center->connect(left, environment::definitions::Direction::LEFT);
    center->connect(right, environment::definitions::Direction::RIGHT);
    left->set_enable_grid(enable_grid_);
    right->set_enable_grid(enable_grid_);
    center->draw();
    right->draw();
    left->draw();
    rooms.append(center);
    rooms.append(right);
    rooms.append(left);
    if (top_r)
    {
        center->connect(top, environment::definitions::Direction::TOP);
        top->set_enable_grid(enable_grid_);
        top->draw();
        rooms.append(top);
    }
    if (bottom_r)
    {
        center->connect(bottom, environment::definitions::Direction::BOTTOM);
        bottom->set_enable_grid(enable_grid_);
        bottom->draw();
        rooms.append(bottom);
    }
    for (auto room : rooms)
    {
        cleaned_rooms_[room] = false;
    }
}

// Functions by Lucas T. G.{

int MarxBotCleaningExperiment::TurnRobot2(float degrees)
{ //here degrees is the final orientation of the robot
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    farsa::ResourceVector<farsa::real> *m_additionalInputs =
        getResource<farsa::ResourceVector<farsa::real>>("agent[0]:additionalInputs");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");

    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    Clock = evonet->getInput(8);
    if (degrees < 0)
    {
        degrees += 360;
    }
    if (degrees == 360)
    {
        degrees = 0;
    }

    //std::cout<< "Orientation: " << getAngle() << std::endl;

    //std::cout<< "Clock: " << Clock << "  ";
    if (Turning && ((abs(getAngle() - degrees) >= 1) && (abs(getAngle() - degrees) <= 359)))
    {
        //std::cout<< "Girando" << std::endl;
        double vel = (getAngle() - degrees) * VelPerStepForOneDregrees;
        if (vel > 10)
        {
            vel = 10;
        }
        //std::cout<< "vel: " << vel << std::endl;
        m_robot->wheelsController()->setSpeeds(vel, -vel);
        return 0;
    }
    else if (Turning)
    {
        if ((abs(getAngle() - degrees) < 1) && (abs(getAngle() - degrees) > FAKE_ZERO))
        {
            //printf("if1\n");
            double vel = VelPerStepForOneDregrees * (getAngle() - degrees);
            //PRINT_DEV << "delta(getAngle - degrees): " << getAngle() << " - " << degrees << " = " << getAngle()-degrees << "  vel: "<< vel << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(vel, -vel);
            return 0;
        }
        else if ((abs(getAngle() - 360 - degrees) > FAKE_ZERO) && (abs(getAngle() - 360 - degrees) < 1))
        {
            //printf("if2\n");
            double vel = VelPerStepForOneDregrees * (getAngle() - 360 - degrees);
            //PRINT_DEV << "delta(getAngle - degrees): " << getAngle() << " - " << degrees << " = " << getAngle()-degrees << "  vel: "<< vel << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(vel, -vel);
            return 0;
        }
        else if ((abs(getAngle() - degrees) <= FAKE_ZERO))
        {
            Turning = False;
            m_robot->wheelsController()->setSpeeds(1, 1);
            PRINT_DEV << "Finda a girada, Orientation: " << getAngle() << PRINTEND_DEV;
            return 1;
        }
    }
    else
    {
        PRINT_DEV << "Inicia a girada" << PRINTEND_DEV;
        Turning = True;
        return 0;
    }
    return 0;
}

float MarxBotCleaningExperiment::getAngle()
{
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    farsa::ResourceVector<farsa::real> *m_additionalInputs;
    m_additionalInputs = getResource<farsa::ResourceVector<farsa::real>>("agent[0]:additionalInputs");
    // float ang = ((robot->orientation(arena->getPlane()) + PI_3_2_RAD) * (180 / M_PIf128)) + 90;
    float ang = ((robot->orientation(arena->getPlane()) ) * (180 / M_PIf128));
    if( ang < 0 ){
        ang += 360;
    }
    else if (ang == 360){
        ang = 0;
    }
    else if (ang > 360){
        ang -= 360;
    }

    return ang;
}

int MarxBotCleaningExperiment::RunRobot(float distance)
{
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    farsa::ResourceVector<farsa::real> *m_additionalInputs =
        getResource<farsa::ResourceVector<farsa::real>>("agent[0]:additionalInputs");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    Clock = evonet->getInput(8);
    PRINT_DEV << "Clock: " << Clock << PRINTEND_DEV;
    PRINT_DEV << "Orientation: " << getAngle() << PRINTEND_DEV;
    if (Running && Clock <= endClock)
    {
        if (evonet->getInput(5) > NEAR_SENSOR && evonet->getInput(6) > NEAR_SENSOR)
        {
            //PRINT_DEV <<<< "Tem parede, parou de andar" << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(-VelPerStepForOneDistance / 2, -VelPerStepForOneDistance / 2);
            Running = False;
            return 1;
        }
        else
        {
            m_robot->wheelsController()->setSpeeds(VelPerStepForOneDistance, VelPerStepForOneDistance);
            PRINT_DEV << "(" << robot->position().x << ", " << robot->position().y << ")" << PRINTEND_DEV;
            return 0;
        }
    }
    else if (Running && Clock > endClock)
    {
        PRINT_DEV << "Finda a caminhada (" << robot->position().x << ", " << robot->position().y << ")" << PRINTEND_DEV;
        Running = False;
        m_robot->wheelsController()->setSpeeds(-VelPerStepForOneDistance / 1.5, -VelPerStepForOneDistance / 1.5);
        return 1;
    }
    else
    {
        PRINT_DEV << "Inicia a caminhada" << PRINTEND_DEV;
        Running = True;
        NecessaryClocks = (distance / DistanceCycles); // how many clocks are necessary to go through the amount of distance requested
        endClock = Clock + NecessaryClocks;
        //std::cout<< "clock: " << Clock << " NC: " << NecessaryClocks << "  endClock:  " << endClock << std::endl;
        return 0;
    }
}

void MarxBotCleaningExperiment::CleamRoomHardMode()
{ //this function positions the robot on the corner
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    farsa::ResourceVector<farsa::real> *m_additionalInputs;
    m_additionalInputs = getResource<farsa::ResourceVector<farsa::real>>("agent[0]:additionalInputs");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    environment::Room *actualy_room = nullptr;
    for (auto room : rooms)
    {
        int query = room->contains(robot);
        if (query == environment::definitions::NO_OVERLAP)
        {
            continue;
        }
        else
        {
            actualy_room = room;
            break;
        }
    }

    // //_____________Logic___________________
    int v = 1; // the variable 'v' represents the percent of velocity will be used, need to stay in range [0,1]
    //int last_track = -1 ;//the value of 'last_track' is '-1' for right, and '1' for left
    
    // std::cout<<"Effect1 "<<effect<<" Effect2 "<<effect2<<std::endl;
    PRINT_DEV << " Orientation: " << getAngle() << PRINTEND_DEV;

    //go find some corner, and go to the corner
    if (effect == 0)
    {
        if (PositionInTheCorner == 0)
        {   
            float vetX, vetY;
            // Room's variables
            x0_room = actualy_room->geometry().x() - actualy_room->size_w() / 2;
            y0_room = actualy_room->geometry().y() - actualy_room->size_h() / 2;
            x1_room = actualy_room->geometry().x() + actualy_room->size_w() / 2;
            y1_room = actualy_room->geometry().y() + actualy_room->size_h() / 2;
            width_room = actualy_room->size_w();
            height_room = actualy_room->size_h();
            vetX = m_robot->position().x - (x0_room + diameter_robot/2 ); // creating the direction vector of way to be covered
            vetY = m_robot->position().y - (y0_room + diameter_robot/2 );

            PRINT_DEV << "variaveis da sal: (" << actualy_room->geometry().x() << "," << actualy_room->geometry().y() << ") (" << actualy_room->size_w() / 2 << "," << actualy_room->size_h() / 2 << ")" << PRINTEND_DEV;
            PRINT_DEV << "variaveis da sala: (" << x0_room << "," << y0_room << ") (" << x1_room << "," << y1_room << ")" << PRINTEND_DEV;

            theta = (atan2f(vetY, vetX)) * (180 / M_PIf128); //Calculates the angle of the direction vector of way to be covered

            distance = (sqrt(pow(vetX, 2) + pow(vetY, 2))); //Calculates the distance that robot need travel to reach the corner]
            // precisa testar
            PRINT_DEV << "vetX: " << vetX << " vetY: " << vetY << " theta: " << theta << " distance: " << distance << PRINTEND_DEV;

            // std::cout << "position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
            // std::cout << "vetX: " << vetX << " vetY: " << vetY << " theta: " << theta << " distance: " << distance << std::endl;
            // std::cout << "variaveis da sal: (" << actualy_room->geometry().x() << "," << actualy_room->geometry().y() << ") (" << actualy_room->size_w() / 2 << "," << actualy_room->size_h() / 2 << ")" << std::endl;
            // std::cout << "variaveis da sala: (" << x0_room << "," << y0_room << ") (" << x1_room << "," << y1_room << ")" << std::endl;


            PositionInTheCorner = 1;
        }
        if (PositionInTheCorner == 1)
        {
            if ( TurnRobot2(180+theta) )
            {   //The Turn function turns the robot in 'argument' degrees clockwise
                // The getAngle function return the degrees of robot in clockwise, regarding of Right side
                PositionInTheCorner = 2;
            }
        }

        if (PositionInTheCorner == 2)
        {
            ;
            if (RunRobot(distance))
            { //Travel to the corner
                PositionInTheCorner = 3;
            }
        }

        if (PositionInTheCorner == 3)
        {
            // std::cout << "corner 3 fora position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
            if (TurnRobot3(0, 0, 1, 1) )
            { // Turn the robot to the right
                PositionInTheCorner = 0;
                effect = 1;
                //std::cout<< "Orientation: " << getAngle() << std::endl;
                //last_track = -1;
                // std::cout << "corner 3 position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
            }
        }
    }
    // NOW THE ROBOT IS POSITIONED

    // while(Room_is_clean == False || m_robot->position().y < height_room - diameter_robot/2){ //Stop when the room is clear or the robot arrive the end of room

    if (effect == 1)
    {
        //go to the wall
        if (RunRobot(width_room - 0.17))
        { //subtrair o diametro pra não haver colisões
            effect = 2;
            if (((m_robot->position().x - diameter_robot) <= x0_room) && ((m_robot->position().y + diameter_robot) >= y1_room))
            {
                PRINT_DEV << "ultima carreira da sala" << PRINTEND_DEV;
                DegStep = last_track * 90;
                effect = 5;
                walk = 0;
                deg = 180;
                cleaned_rooms_[actualy_room] = true;
                old_room_ = actualy_room;
            }
            else if ((((m_robot->position().x) + diameter_robot) >= (x1_room)) && (((m_robot->position().y) + diameter_robot) >= y1_room))
            {
                PRINT_DEV << "ultima carreira da sala" << PRINTEND_DEV;
                DegStep = last_track * 90;
                effect = 5;
                walk = 0;
                deg = 0;
                cleaned_rooms_[actualy_room] = true;
                old_room_ = actualy_room;
            }
        }
    }

    else if (effect == 2)
    {
        //turn 90 degrees, to stand parallel with the wall
        if (TurnRobot2(90))
        {
            effect = 3;
        }
    }

    else if (effect == 3)
    {
        //travel the diameter of the robot
        if (RunRobot(diameter_robot))
        {
            effect = 4;
        }
    }

    else if (effect == 4)
    {
        //turn 90 degrees, to stand with the back to the wall
        if (last_track == -1)
        {
            if (TurnRobot2(180))
            {
                effect = 1;
                // so that in the next loop the robot turn to the other side
                last_track *= -1;
            }
        }
        else if (last_track == 1)
        {
            if (TurnRobot2(0))
            {
                effect = 1;
                // so that in the next loop the robot turn to the other side
                last_track *= -1;
            }
        }
    }
    else if (effect == 5)
    {
        // findRoom();
    }
}

void MarxBotCleaningExperiment::findRoom()
{
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Arena *arena = getResource<farsa::Arena>("arena");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    environment::Room *actualy_room = nullptr;
    for (auto room : rooms)
    {
        int query = room->contains(robot);
        if (query == environment::definitions::NO_OVERLAP)
        {
            continue;
        }
        else
        {
            actualy_room = room;
            break;
        }
    }

    if (effect2 == 5)
    {
        if (deg == 360)
        {
            deg = 0;
        }
        PRINT_DEV << "Clock Fe 5 : " << Clock << PRINTEND_DEV;
        if (TurnRobot2(deg))
        {
            effect2 = 6;
        }
    }

    else if (effect2 == 6)
    {
        Clock = evonet->getInput(8);
        PRINT_DEV << "Clock Fe 6 : " << Clock << PRINTEND_DEV;
        m_robot->wheelsController()->setSpeeds(VelPerStepForOneDistance, VelPerStepForOneDistance);
        if (evonet->getInput(5) > NEAR_SENSOR && evonet->getInput(6) > NEAR_SENSOR)
        {
            PRINT_DEV << "Parede frente" << PRINTEND_DEV;
            deg += DegStep;
            m_robot->wheelsController()->setSpeeds(-2.5, -2.5);
            effect2 = 5;
        }
        if (last_track == -1)
        {
            if (evonet->getInput(0) < 0.50 && evonet->getInput(7) < 0.50)
            {
                PRINT_DEV << "corredor lado esquerdo" << PRINTEND_DEV;
                m_robot->wheelsController()->setSpeeds(-5, -5);
                effect2 = 7;

            }
        }
        else
        {
            if (evonet->getInput(3) < 0.50 && evonet->getInput(4) < 0.50)
            {
                PRINT_DEV << "corredor lado direito" << PRINTEND_DEV;
                m_robot->wheelsController()->setSpeeds(-5, -5);
                effect2 = 7;
            }
        }
        
    }

    else if (effect2 == 7)
    {
        PRINT_DEV << "Clock Fe 7 : " << Clock << PRINTEND_DEV;
        PRINT_DEV << "deg-DegStep : " << deg - DegStep << PRINTEND_DEV;
        if (TurnRobot2(deg - DegStep))
        {
            deg -= DegStep;
            teste = 0;
        }
        if(teste == 0){
            
            if (RunRobot(diameter_robot)){
                m_robot->wheelsController()->setSpeeds(10, 10);
                effect2 = 8;
            }
            
        }
    }

    else if (effect2 == 8){
        teste = 1;
        PRINT_DEV << "Clock Fe 8 : " << Clock << PRINTEND_DEV;
        
        if (last_track == -1){
            PRINT_DEV << "3 :" <<evonet->getInput(0) << " 4: "<<evonet->getInput(7) << PRINTEND_DEV;
            if (evonet->getInput(0) > FAR_SENSOR && evonet->getInput(7) > FAR_SENSOR){
                m_robot->wheelsController()->setSpeeds(10, 10);
            }else{
                //m_robot->wheelsController()->setSpeeds(-5, -5);
                effect2 = 9;
            }
        }else{
            PRINT_DEV << "3 :" <<evonet->getInput(3) << " 4: "<<evonet->getInput(4) << PRINTEND_DEV;
            if (evonet->getInput(3) > FAR_SENSOR && evonet->getInput(4) > FAR_SENSOR){
                m_robot->wheelsController()->setSpeeds(10, 10);
            }else{
                //m_robot->wheelsController()->setSpeeds(-5, -5);
                effect2 = 9;
            
            }
        }

    }else if (effect2 == 9){
        PRINT_DEV << "Clock Fe 9 : " << Clock << PRINTEND_DEV;
        if (!cleaned_rooms_[actualy_room]){
            if (RunRobot(diameter_robot * 5)){
                effect = 0;
                last_track = -1;
                walk = 1;
                effect2 = 6;
            }
        }else{
            if (TurnRobot2(deg-DegStep)){
                deg -= DegStep;
                effect2 = 10;
                if (deg >= 360)
                {
                    deg -= 360;
                }
                if (deg < 0){
                    deg += 360;
                }
            }
        }
    }else if (effect2 == 10){
        if (RunRobot(diameter_robot)){
            m_robot->wheelsController()->setSpeeds(10, 10);
            effect2 = 6;
        }
    }
}

void MarxBotCleaningExperiment::Cleaning(){
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    //go find some corner, and go to the corner
    if (effect == 0)
    {
        if (PositionInTheCorner == 0)
        {
            if ( RunRobotF3(0, 0, 0, 0) )
            {
                std::cout << "corner 0 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
                PositionInTheCorner = 1;
            }
        }

        if (PositionInTheCorner == 1)
        {
            if ( TurnRobotJ3('l') ) 
            {
                std::cout << "corner 1 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
                PositionInTheCorner = 2;
            }
        }

        if (PositionInTheCorner == 2)
        {
            // PRINT_DEV << "corner 2 Clock: " << evonet->getInput(8) << " position:("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << "sensors " << frontSensor() << " - " << rightSensor() << " - " << backSensor() << " - " << leftSensor() << PRINTEND_DEV;
            // PRINT_DEV << "sensors back: " << evonet->getInput(1) << " - " << evonet->getInput(1) << PRINTEND_DEV;
            if ( RunRobotF3(0, 0, 0, 1) )
            {
                std::cout << "corner dentro 2 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
                PositionInTheCorner = 3;
            }
            // RunRobotF3(0, 0, 0, 1);
        }

        if (PositionInTheCorner == 3)
        {
            // std::cout << "corner 3 fora Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
            if ( TurnRobot3(0, 1, 1, 0) )
            {
                PositionInTheCorner = 0;
                effect = 1;
                std::cout << "corner 3 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
            }
        }
    }
    // NOW THE ROBOT IS POSITIONED

    else if(effect == 1 ){
        if( RunRobotF3(0,0,0,0) ){
            effect = 2;
        }
    }else if( effect == 2 ){
        if( last_track == -1 ){
            if( TurnRobotJ3('r') ){
                effect = 3;
            }
        }else{
            if( TurnRobotJ3('l') ){
                effect = 3;
            }
        }
    }else if( effect == 3 ){
        if( RunRobot(diameter_robot) ){
            effect = 4;
            PRINT_DEV << "effect 3 " << PRINTEND_DEV;
        }
        // m_robot->wheelsController()->setSpeeds(10, 10);
        // effect = 4;
        
    }else if( effect == 4 ){
        if( TurnRobotJ3('b') ){
            effect = 1;
            last_track *= -1;
        }
    }

    // else if ( effect == 1 ){
    //     // if ( rightSensor() == 1){    
    //     //     if (RunRobotF3(1, 1, 0, 0)){
    //     //         // if (last_Walk == 1){
    //     //         //     effect = 5;
    //     //         //     cleanRoom ++;
    //     //         //     last_Walk = 0;
    //     //         // }else{
    //     //         //     effect = 2;
    //     //         //     last_Walk += 1;
    //     //         // }
    //     //         effect = 2;
    //     //     }
    //     // }else if ( leftSensor() == 1 ){
    //     //     if (RunRobotF3(1, 0, 0, 1)){
    //     //         effect = 2;
    //     //     }
    //     // }else{
    //     if (RunRobotF3(0, 0, 0, 0)){
    //         effect = 2;
    //     }
    //     // }
    // }else if( effect == 2 ){
    //     if (last_track == 1){
    //         if ( rightSensor() == 1 ){//Teoricamente nao deveria entrar aki pois deveria entrar no effect 5 porem pra previnir eu fiz
    //             if ( TurnRobot3(0,1,0,1)){
    //                 effect = 3;
    //             } 
    //         }else{
    //             if( TurnRobot3(0, 0, 0, 1) ){
    //                 effect = 3;
    //             }
    //         }
    //     }else{
    //         // if ( rightSensor() == 1 ){
    //         //     if ( TurnRobot3(0,1,1,0)){
    //         //         effect = 3;
    //         //     }
                
    //         // }else 
    //         if ( leftSensor() == 1 ){
    //             if ( TurnRobot3(0,1,1,0) ){
    //                 effect = 3;
    //             }
                
    //         }else{
    //             if( TurnRobot3(0, 1, 0, 0) ){
    //                 effect = 3;
    //             }
    //         }
    //     }
        
    // }else if( effect == 3 ){
    //      if (RunRobot(diameter_robot)){
    //          effect = 4;
    //      }
         
    // }else if ( effect == 4 ){
    //     if (last_track == 1){
    //         if ( frontSensor() == 1 ){//Teoricamente nao deveria entrar aki pois deveria entrar no effect 5 porem pra previnir eu fiz
    //             if ( TurnRobot3(0,0,1,1)){
    //                 effect = 1;
    //             }
    //         }else{
    //             if( TurnRobot3(0, 0, 1, 0) ){
    //                 effect = 1;
    //             }
    //         }
    //         last_track = -1;
    //     }else{
    //         if ( frontSensor() == 1 ){
    //             if ( TurnRobot3(0,1,1,0)){
    //                 effect = 3;
    //             }
    //         }else{
    //             if( TurnRobot3(0, 0, 1, 0) ){
    //                 effect = 1;
    //             }
    //         }
    //     }
    // }else if (effect == 5){
    //     if ( cleanRoom > 2 ){
    //         cleanRoom = 2;
    //     }
    //     // demandRoom();
    // }
}
void MarxBotCleaningExperiment::demandRoom(){
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);

    if (effect2 == 5){
        if ( TurnRobot3(0,1,1,0) ){
            effect2 = 6;
        }
    }
    else if (effect2 == 6){
        Clock = evonet->getInput(8);
        PRINT_DEV << "Clock Fe 6 : " << Clock << PRINTEND_DEV;
        if ( RunRobotF3(1,1,0,0) ){//evonet->getInput(5) > NEAR_SENSOR && evonet->getInput(6) > NEAR_SENSOR){
            // PRINT_DEV << "Parede frente" << PRINTEND_DEV;
            deg += DegStep;
            // m_robot->wheelsController()->setSpeeds(-2.5, -2.5);
            effect2 = 5;
        }
        if (evonet->getInput(3) < 0.40 && evonet->getInput(4) < 0.40){
            PRINT_DEV << "corredor lado direito" << PRINTEND_DEV;
            //m_robot->wheelsController()->setSpeeds(-5, -5);
            effect2 = 7;
        }
    }

    else if (effect2 == 7){//  Trocar logica posteriamente rodar no eixo 90 graus e andar reto andar achar o corredor
        PRINT_DEV << "Clock Fe 7 : " << Clock << PRINTEND_DEV;
        PRINT_DEV << "deg-DegStep : " << deg - DegStep << PRINTEND_DEV;
        if (TurnRobot2(deg - DegStep)){
            deg -= DegStep;
            teste = 0;
        }
        if(teste == 0){
            if (RunRobot(diameter_robot)){
                m_robot->wheelsController()->setSpeeds(10, 10);
                effect2 = 8;
            }
        }
    }

    else if (effect2 == 8){
        teste = 1;
        PRINT_DEV << "Clock Fe 8 : " << Clock << PRINTEND_DEV;
        m_robot->wheelsController()->setSpeeds(10, 10);
        PRINT_DEV << "3 :" <<evonet->getInput(3) << " 4: "<<evonet->getInput(4) << PRINTEND_DEV;
        if (evonet->getInput(3) < 0.40 && evonet->getInput(4) < 0.40){
            hallPass += 1; 
            effect2 = 9;
        }

    }
    else if (effect2 == 9){
        PRINT_DEV << "Clock Fe 9 : " << Clock << PRINTEND_DEV;
        if ( cleanRoom == hallPass ){
            effect = 0;
            effect2 = 5;
            hallPass = 0;
            // nova sala voltar a effect 0
            // talvez mandar ele andar ou rodar
        }
        else{       // deixar assim pois curva aki eh sem canto e como quero testar o resto essa parte faz sentido deixar assim
            if (TurnRobot2(deg-DegStep))
            {
                
                deg -= DegStep;
                effect2 = 10;
                if (deg >= 360)
                {
                    deg -= 360;
                }
                if (deg < 0){
                    deg += 360;
                }
            }
        }
    }
    else if (effect2 == 10){
        if (RunRobot(diameter_robot)){
            m_robot->wheelsController()->setSpeeds(10, 10);
            effect2 = 6;
        }
    }
}

int MarxBotCleaningExperiment::RunRobotF3(int front, int right, int back, int left){ //turn the robot until just the asked sensors stay activated
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    // if( ( front && frontSensor() ) ||  ( right && rightSensor() ) || ( back && backSensor() ) || ( left && leftSensor() ) ){
    // if( (front && front == frontSensor() ) ||  ( right && right == rightSensor() ) || (back && back == backSensor() ) || (left && left == leftSensor() ) ){
    // if( ( ( front == frontSensor() ) &&  ( right == rightSensor() ) && ( back == backSensor() ) && ( left == leftSensor() ) ) || frontSensor() ){
    if ( (rightSensor() > NEAR_SENSOR && leftSensor() > NEAR_SENSOR)  ){
        // m_robot->wheelsController()->setSpeeds(10, 10);
        std::cout<< "RRF3 Corredor" << std::endl << std::endl << std::endl << std::endl << std::endl;
        return False;
    }//else
    if( frontSensor() || ( evonet->getInput(5) > NEAR_SENSOR && evonet->getInput(6) > NEAR_SENSOR ) ){
        // std::cout<< "sensors" << frontSensor() << " - " << rightSensor() << " - " << backSensor() << " - " << leftSensor() << std::endl;
        m_robot->wheelsController()->setSpeeds(-1, -1);
        return True;
    }else{
        if(right == 1){
            // float v1 = (6+log2(evonet->getInput(3)))/6;
            // float v2 = (6+log2(evonet->getInput(4)))/6;
            float v1 = evonet->getInput(3);
            float v2 = evonet->getInput(4);
            if( (v1 + v2 )/2 < 0.7 ){
                // PRINT_DEV << "FAR" << PRINTEND_DEV;
                v2 *= 0.7;
            }else if ( (v1 + v2) > 0.8 ){
                // PRINT_DEV << "NEAR" << PRINTEND_DEV;
                v1 *= 0.8;
            }
            // PRINT_DEV << " r " << v1 << ", " << v2 << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(10*v1, 10*v2 );
            return False;
        }else if (left == 1){
            float v1 = evonet->getInput(7);
            float v2 = evonet->getInput(0);
            if( (v1 + v2 )/2 < 0.7 ){
                // PRINT_DEV << "FAR" << PRINTEND_DEV;
                v1 *= 0.7;
            }
            else if( (v1 + v2 )/2 > 0.8){
                // PRINT_DEV << "NEAR" << PRINTEND_DEV;
                v2 *= 0.8;
            }
            // PRINT_DEV << " l " << v1 << ", " << v2 << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(10*v1, 10*v2 );
            return False;
        }else{
            if( rightSensor() ){
                float v1 = evonet->getInput(3);
                float v2 = evonet->getInput(4);
                if( (v1 + v2 )/2 > 0.8){
                    v1 *= 0.8;
                }
                // PRINT_DEV << " r2 " << v1 << ", " << v2 << PRINTEND_DEV;
                m_robot->wheelsController()->setSpeeds(10*v1, 10*v2 );
                return False;
            }else if ( leftSensor() ){
                float v1 = evonet->getInput(7);
                float v2 = evonet->getInput(0);
                if( (v1 + v2 )/2 > 0.8){
                    v2 *= 0.8;
                }
                // PRINT_DEV << " l2 " << v1 << ", " << v2 << PRINTEND_DEV;
                m_robot->wheelsController()->setSpeeds(10*v1, 10*v2 );
                return False;
            }else{
                m_robot->wheelsController()->setSpeeds(10, 10);
                return False;
            }
            m_robot->wheelsController()->setSpeeds(10, 10);
        }
        return False;
    }
}

int MarxBotCleaningExperiment::RunRobotW3(int front, int right, int back, int left){ //turn the robot until just the asked sensors stay activated
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    // if( ( front && frontSensor() ) ||  ( right && rightSensor() ) || ( back && backSensor() ) || ( left && leftSensor() ) ){
    // if( (front && front == frontSensor() ) ||  ( right && right == rightSensor() ) || (back && back == backSensor() ) || (left && left == leftSensor() ) ){
    if (rightSensor() > NEAR_SENSOR && leftSensor() > NEAR_SENSOR){
        std::cout<< "RRW3 Corredor" << std::endl;
        m_robot->wheelsController()->setSpeeds(-10, -10);
        return False;
    }else if( ( ( front == frontSensor() ) &&  ( right == rightSensor() ) && ( back == backSensor() ) && ( left == leftSensor() ) ) && !frontSensor() ){
        if( rightSensor() ){
            float v1 = evonet->getInput(3);
            float v2 = evonet->getInput(4);
            if( (evonet->getInput(3) + evonet->getInput(4) )/2 > 0.8){
                v1 *= 0.8;
            }
            PRINT_DEV << " r " << v1 << ", " << v2 << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(10*v1, 10*v2 );
        }else if ( leftSensor() ){
            float v1 = evonet->getInput(7);
            float v2 = evonet->getInput(0);
            if( (evonet->getInput(7) + evonet->getInput(0) )/2 > 0.8){
                v2 *= 0.8;
            }
            PRINT_DEV << " r " << v1 << ", " << v2 << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(10*v1, 10*v2 );
        }else{
            m_robot->wheelsController()->setSpeeds(10, 10);
            return False;
        }
    }else
    {
        // std::cout<< "sensors" << frontSensor() << " - " << rightSensor() << " - " << backSensor() << " - " << leftSensor() << std::endl;
        m_robot->wheelsController()->setSpeeds(0.1, 0.1);
        return True;
    }
}

int MarxBotCleaningExperiment::TurnRobot3(int front, int right, int back, int left){ //turn the robot until just the asked sensors stay activated
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    if( front == frontSensor() &&  right == rightSensor() && back == backSensor() && left == leftSensor() ){
        // std::cout<< "sensors" << frontSensor() << " - " << rightSensor() << " - " << backSensor() << " - " << leftSensor() << std::endl;
        m_robot->wheelsController()->setSpeeds(1, 1);
        return True;
    }else
    {
        float v1 = 1, v2 = 1;
        float s1, s2;
        if(front == 1){
            s1 = evonet->getInput(6);
            s2 = evonet->getInput(5);
        }else if(right == 1){
            s1 = evonet->getInput(4);
            s2 = evonet->getInput(3);
        }else if(back == 1){
            s1 = evonet->getInput(2);
            s2 = evonet->getInput(1);
        }else if(left == 1){
            s1 = evonet->getInput(0);
            s2 = evonet->getInput(7);
        }

        if( s1 + s2 < 1 ){
            v1 = 5; v2 = 5;
            PRINT_DEV << "(turnJ3 1) s1/2: (" << s1 <<", "<< s2 << ") vel: (" << v1 <<", "<< v2 << ")angle: " << getAngle() <<PRINTEND_DEV;
        }else if( s1 > 0 && s2 > 0 ){
            v1 = 5*(s2-s1);
            v2 = 5*(s2-s1);
            PRINT_DEV << "(turnJ3 2) s1/2: (" << s1 <<", "<< s2 << ") vel: (" << v1 <<", "<< v2 << ")angle: " << getAngle() <<PRINTEND_DEV;
        }else {
            v1 = 1; v2 = 1;
            PRINT_DEV << "(turnJ3 3) s1/2: (" << s1 <<", "<< s2 << ") vel: (" << v1 <<", "<< v2 << ") angle: " << getAngle() <<PRINTEND_DEV;
        }
        m_robot->wheelsController()->setSpeeds(v1, -v2);
        return False;
    }
}

int MarxBotCleaningExperiment::TurnRobotJ3(char side){ //turn the robot until just the asked sensors stay activated
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    if( (side == 'f' && frontSensor()) ||  (side == 'r' && rightSensor()) || (side == 'b' && backSensor()) || (side == 'l' && leftSensor()) ){
        // std::cout<< "sensors" << frontSensor() << " - " << rightSensor() << " - " << backSensor() << " - " << leftSensor() << std::endl;
        PRINT_DEV << "TurnJ3 out || angle: " << getAngle() <<PRINTEND_DEV;
        m_robot->wheelsController()->setSpeeds(1, 1);
        return True;
    }else
    {
        float v1 = 1, v2 = 1;
        float s1, s2;
        if(side == 'l'){
            s1 = evonet->getInput(0);
            s2 = evonet->getInput(7);
        }else if(side == 'r'){
            s1 = evonet->getInput(3);
            s2 = evonet->getInput(4);
        }else if(side == 'b'){
            s1 = evonet->getInput(1);
            s2 = evonet->getInput(2);
        }else if(side == 'f'){
            s1 = evonet->getInput(5);
            s2 = evonet->getInput(6);
        }

        if( s1 + s2 < 1 ){
            v1 = 5; v2 = 5;
            PRINT_DEV << "(turnJ3 1) s1/2: (" << s1 <<", "<< s2 << ") vel: (" << v1 <<", "<< v2 << ")angle: " << getAngle() <<PRINTEND_DEV;
        }else if( s1 > 0 && s2 > 0 ){
            v1 = 5*(s2-s1);
            v2 = 5*(s2-s1);
            PRINT_DEV << "(turnJ3 2) s1/2: (" << s1 <<", "<< s2 << ") vel: (" << v1 <<", "<< v2 << ")angle: " << getAngle() <<PRINTEND_DEV;
        }else {
            v1 = 1; v2 = 1;
            PRINT_DEV << "(turnJ3 3) s1/2: (" << s1 <<", "<< s2 << ") vel: (" << v1 <<", "<< v2 << ") angle: " << getAngle() <<PRINTEND_DEV;
        }
        m_robot->wheelsController()->setSpeeds(v1, -v2);
        return False;
    }
}

// 7-0 lado esquerdo
// 1-2 atras
// 3-4 lado direito
// 5-6 frente
int  MarxBotCleaningExperiment::frontSensor(){
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    if( ( (evonet->getInput(5) + evonet->getInput(6) )/2 > NEAR_SENSOR) && ( abs(evonet->getInput(5) - evonet->getInput(6)) < FAKE_ZERO_2 ) ){
        return True;
    }else{
        return False;
    }
}
int  MarxBotCleaningExperiment::rightSensor(){
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    if( ( (evonet->getInput(3) + evonet->getInput(4) )/2 > NEAR_SENSOR) && ( abs(evonet->getInput(3) - evonet->getInput(4)) < FAKE_ZERO_2 ) ){
        return True;
    }else{
        return False;
    }
}
int  MarxBotCleaningExperiment::backSensor(){
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
   if( ( (evonet->getInput(1) + evonet->getInput(2) )/2 > NEAR_SENSOR) && ( abs(evonet->getInput(1) - evonet->getInput(2)) < FAKE_ZERO_2 ) ){
        return True;
    }else{
        return False;
    }
}
int  MarxBotCleaningExperiment::leftSensor(){
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    if( ( (evonet->getInput(7) + evonet->getInput(0) )/2 > NEAR_SENSOR) && ( abs(evonet->getInput(7) - evonet->getInput(0)) < FAKE_ZERO_2 ) ){
        return True;
    }else{
        return False;
    }
}

// } Functions by Lucas T. G.
