/********************************************************************************
 *  FARSA - Total99                                                             *
 *  Copyright (C) 2012-2013 Gianluca Massera <emmegian@yahoo.it>                *
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

#ifndef MARXBOTCLEANINGEXPERIMENT_H
#define MARXBOTCLEANINGEXPERIMENT_H

#include <QVector>
#include <QMap>
#include <chrono>

#include "farsaplugin.h"
#include "evorobotexperiment.h"
#include "wvector.h"
#include "randomgenerator.h"
#include "environment/room.h"

/**
 * \brief An experiment in which a khepera robot has to discriminate between an
 *        object and the arena walls
 *
 * The resources used by this experiment are the same as the ones of the parent
 * class (EvoRobotExperiment)
 *
 * The parameters for this experiment are:
 * 	- distanceThreshold: the distance from the object below which the robot
 * 	                     is rewarded. This is the distance of the nearest
 * 	                     points of the robot and the object
 * 	- playgroundWidth: the width of the part of the arena surrounded by
 * 	                   walls (the playground is the area where the robot can
 * 	                   move)
 * 	- playgroundHeight: the height of the part of the arena surrounded by
 * 	                    walls (the playground is the area where the robot
 * 	                    can move)
 * 	- minObjectDistanceFromWall: the minimum distance from the walls at
 * 	                             which the object is placed. The default is
 * 	                             0.05
 * 	- minInitialRobotDistanceFromObject: the minimum distance from the
 * 	                                     object at which the robot is
 * 	                                     initialized. The default is 0.1
 * 	- minInitialRobotDistanceFromWall: the minimum distance from the walls
 * 	                                   at which the robot is initialized.
 * 	                                   The default is 0.1
 */
class FARSA_PLUGIN_API MarxBotCleaningExperiment :
    public farsa::EvoRobotExperiment {
    Q_OBJECT
    FARSA_REGISTER_CLASS(EvoRobotExperiment)

 public:
	/**
	 * \brief Constructor
	 */
    MarxBotCleaningExperiment();
	/**
	 * \brief Destructor
	 */
    ~MarxBotCleaningExperiment();
	/**
	 * \brief Configures the object using a ConfigurationParameters object
	 *
	 * \param params the configuration parameters object with parameters to
	 *               use
	 * \param prefix the prefix to use to access the object configuration
	 *               parameters. This is guaranteed to end with the
	 *               separator character when called by the factory, so you
	 *               don't need to add one
	 */
    virtual void configure(farsa::ConfigurationParameters& params,
                                                        QString prefix);
	/**
	 * \brief Saves the actual status of parameters into the
	 *        ConfigurationParameters object passed
	 *
	 * \param params the configuration parameters object on which to save
	 *               the actual parameters
	 * \param prefix the prefix to use to access the object configuration
	 *               parameters
	 */
    virtual void save(farsa::ConfigurationParameters& params,
                                                    QString prefix);
	/**
	 * \brief Add to Factory::typeDescriptions() the descriptions of all
	 *        parameters and subgroups
	 *
	 * It's mandatory in all subclasses where configure and save methods
	 * have been re-implemented for dealing with new parameters and
	 * subgroups to also implement the describe method
	 * \param type is the name of the type regarding the description. The
	 *             type is used when a subclass reuse the description of its
	 *             parent calling the parent describe method passing the
	 *             type of the subclass. In this way, the result of the
	 *             method describe of the parent will be the addition of the
	 *             description of the parameters of the parent class into
	 *             the type of the subclass
	 */
    static void describe(QString type);
	/**
	 * \brief This function is called after all linked objects have been
	 *        configured
	 *
	 * See the description of the ConfigurationParameters class for more
	 * information. This method creates the arena and fills it with objects
	 */
    virtual void postConfigureInitialization();
	/**
	 * \brief Called at the begin of each generation
	 *
	 * \param generation the generation about to start
	 */
    virtual void initGeneration(int generation);
	/**
	 * \brief Called before the evaluation of a new individual
	 *
	 * \param individual the id of the individual about to be tested
	 */
    virtual void initIndividual(int individual);
	/**
	 * \brief Called at the beginning of each trial
	 *
	 * \param trial the trial about to start
	 */
    virtual void initTrial(int trial);
	/**
	 * \brief Called at the beginning of each step (before world advance)
	 *
	 * \param step the step about to start
	 */
    virtual void initStep(int step);

// Functions by Lucas T. G.{

	#define True 1
	#define False 0
	#define NEAR_SENSOR 0.60
	#define FAR_SENSOR 0.15
	#define FAKE_ZERO 5.10352e-04
	#define FAKE_ZERO_2 0.001
	#define PI_3_2_RAD 4.71239 //is 3/2pi in rad or 270 degrees
	#define DEV_MOD 1
	#define PRINT_DEV if(DEV_MOD){std::cout
	#define PRINTEND_DEV std::endl;}
	#define SENSORS evonet->getInput

	virtual void Cleaning();

	virtual int RunRobotF3(char side,  int SwitchingRooms, int PosInCorner);

	virtual int TurnRobotJ3(char side);

	virtual int frontSensor();
	virtual int rightSensor();
	virtual int backSensor();
	virtual int leftSensor();

	virtual float getAngle();

	virtual void PrintSensors();

	virtual int TurnHalfMoon();

	virtual int RunUntilHit();

	int Turning = False;
	int Running = False;
	int InCorridor = False;
	int effect = 0; //what the robot need to do on current step
	int PositionInTheCorner = 0; //an assist for the 'effect' when 'effect' is equal zero
	int last_track = -1;//the value of 'last_track' is '-1' for right, and '1' for left
	float diameter_robot = 0.15;

// } Functions by Lucas T. G.

	/**
	 * \brief Celled after all sensors have been updated but before network
	 *        spreading
	 *
	 * This is useful, for example, to overwrite the inputs of the neural
	 * network (i.e.: to silence some neurons during the experiment without
	 * modifing sensors classes)
	 */
    virtual void afterSensorsUpdate();
	/**
	 * \brief Called just before updating motors and after updating the
	 *        neural network
	 *
	 * This is useful, for example, to overwrite the outputs of the neural
	 * network
	 */
    virtual void beforeMotorsUpdate();
	/**
	 * \brief Called just before the world advances, after the update of
	 *        motors
	 *
	 * This is useful, for example, to manually actuate motors overriding
	 * the robot controller commands
	 */
    virtual void beforeWorldAdvance();
	/**
	 * \brief Called at the end of each step
	 *
	 * \param step the step about to end
	 */
    virtual void endStep(int step);
	/**
	 * \brief Called at the end of each trial
	 *
	 * \param trial the trial about to end
	 */
    virtual void endTrial(int trial);
	/**
	 * \brief Called at the end of an individual life
	 *
	 * \param individual the individual that has been just tested
	 */
	virtual void savefile(double total_fitness);
	/**
	 * \brief save information for each trial in a csv file 
	 *  
	 */

    virtual void endIndividual(int individual);
	/**
	 * \brief Called at the end of each generation
	 *
	 * \param generation the generation about to end
	 */
    virtual void endGeneration(int generation);
    /**
     * \brief utilization of alternative modules and time since the current module is operating
     */
    int m_selmod;
    int m_selmodold;
    int m_modstep;
    int m_numStepsActive;
    double ntime;
    double oldselector[10];
    int m_changeBehaviours;
    /**
     * \brief The battery level. The trial stop when it becomes null.
     */
    double m_energy;
    /**
     * \brief The battery level. The trial stop when it becomes null.
     */
    int debug;
	int trial;

 private:
	/**
	 * \brief Returns the fitness (a reward or a punishment depending on whether the robot is located on a green or red target area)
	 *
	 * \param robot the robot (specified as a parameter so that we don't
	 *              use resources inside this function)
	 * \return the distance between the robot and the object
	 */
    farsa::real ffitness(farsa::RobotOnPlane* robot) const;
    /**
     * \brief The current Height of arena walls
     */
    farsa::real halfHeight;
    /**
     * \brief The current Width of arena walls
     */
    farsa::real halfWidth;
	/**
	 * \brief The thickness of arena walls
	 */
    const farsa::real m_wallThickness;
    /**
     * \brief The thickness of arena walls
     */
    const farsa::real m_objectHeights;
	/**
	 * \brief The range of variation of walls
     *
	 */
    farsa::real m_wallsvar;
	/**
	 * \brief The minimum length of short walls
	 *
	 */
    farsa::real m_shortwallmlength;
	/**
	 * \brief The minimum length of long walls
	 *
	 */
    farsa::real m_longwallmlength;

	/**
	 * \brief Objects placed inside the arena, walls and cylinders
     */
    QVector<farsa::Box2DWrapper*> outerWall;
    QVector<farsa::Box2DWrapper*> innerWall;
    QVector<farsa::Cylinder2DWrapper*> smallCylinder;
    farsa::Box2DWrapper* couch;
    farsa::Box2DWrapper* bookcase;
    QVector<farsa::wVector> chairPos;
    QVector<farsa::Cylinder2DWrapper*> bigCylinder;
    farsa::Cylinder2DWrapper* rechargeStation;
    QString m_testFilename, m_outStr;
    QVector<environment::Room*> rooms;
	environment::Room* old_room_;
	QMap<environment::Room*, bool> cleaned_rooms_;
    /**
     * \brief Size of the outer and inner arena
     */
    farsa::real topWallPos, bottomWallPos, rightWallPos, leftWallPos;
    farsa::real topWallPosb, bottomWallPosb, rightWallPosb, leftWallPosb;
    /**
     * \brief Matrix containing visited cells
     */
    int m_wcells[10000];
    /**
     * \brief The size of the cells
     */
    double m_cellsize, m_generalCounter;
    int m_xmaxsize, m_ymaxsize, m_visitedXcell,
                m_visitedYcell, m_stepsSameCell;
    /**
	 * \brief If true re-creates the world at the beginning of the next
	 *        trial
	 */
    bool m_recreateWorld, m_considerEnergy;
    int m_multiRooms;
    /**
     * \brief The size of the cells
     */
    int laststepfit[50], m_minPolicyOperation,  m_maxPolicyOperation;
    bool m_saveTestStats;
    int m_varyNonLinear;
    /**
     * \brief utilization of alternative modules and time since the current module is operating
     */
    int modtime[10];
    int modtimeperiod[10];
    /**
     * \brief The seed for the environmental generation that is reset every trial
     */
    int expseed;
    /**
     * \brief Each individual/thread has its own random generator class
     */
    farsa::RandomGenerator locRNG;
    int nbcyl;
    int m_placedCyl;
    void destroyRooms();
    void placeRobot(int trial);
    void createRooms();
    QVector< QVector< farsa::Box2DWrapper * > > m_grid;
	QVector< QVector<double> >trials_;
    int m_numIndividuals;
    int m_individualSelected;
    int m_testedIndividual;
    int m_numStepsChange;
    int m_energyLevel;
    int m_stepsCounter;
    double m_rechargeRate;
    int m_trial;
    double m_changePenaltyFitness;
    double m_changePenaltyEnergy;
    int m_changeProbability;
	bool enable_grid_;
	bool save_data_;
	bool append_data_;
	bool clock_up_;
	QString fk_raise_function_;
	QString fk_fall_function_;
	int fk_steps_raise_;
	int fk_steps_fall_;
	int fk_a_;
	int fk_b_;
	double threshold_;
	QString data_dir_;
};
#endif
