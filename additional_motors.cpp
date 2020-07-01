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


#include "evo/additional_motors.h"
#include "baseexception.h"
#include "randomgenerator.h"
#include "robots.h"
#include "world.h"
#include "phybox.h"
#include "wheeledexperimenthelper.h"
#include "logger.h"
#include "configurationhelper.h"
#include "utilitiesexceptions.h"
#include "mathutils.h"
#include "evo/marxbotcleaningexperiment.h"
#include <QRegExp>
#include <iostream>

MarXbotWheelVelocityMotorMod::MarXbotWheelVelocityMotorMod(farsa::ConfigurationParameters& params, QString prefix) :
	MarXbotMotor(params, prefix),
	m_robot(NULL),
	m_neuronsIterator(NULL),
    motormode(0),
    m_numOutput(2)
{
        addUsableResource("experiment");
        motormode = farsa::ConfigurationHelper::getInt(params, prefix + "motormode", motormode);
        if(motormode==2)
            m_numOutput = 2;
        else
            m_numOutput = 6;
        

}

MarXbotWheelVelocityMotorMod::~MarXbotWheelVelocityMotorMod()
{
	// Nothing to do here
    
}

void MarXbotWheelVelocityMotorMod::save(farsa::ConfigurationParameters& params, QString prefix)
{
	// Calling parent function
	MarXbotMotor::save(params, prefix);

	// Saving parameters
	params.startObjectParameters(prefix, "MarXbotWheelVelocityMotor", this);
}

void MarXbotWheelVelocityMotorMod::describe(QString type)
{
	// Calling parent function
	MarXbotMotor::describe(type);

	// Describing our parameters
	Descriptor d = addTypeDescription(type, "The motor controlling the velocity of the wheels of the MarXbot robot");    
    d.describeInt("motormode").def(0).limits(0, 2).help("0=selfselected 1=every n steps 2=always the first module");    

}

void MarXbotWheelVelocityMotorMod::update(){

    // printf("runing\n");
    
    // if(0){
    //     neuralMode();
    // }else{
    //     ourMode();
    // }
}

void MarXbotWheelVelocityMotorMod::neuralMode(){

    float ovalues[m_numOutput];
    float maxact;
    int maxi;
    double v1, v2;
    int v, vi;
    MarxBotCleaningExperiment *pexp;
    
    // Checking all resources we need exist
	checkAllNeededResourcesExist();

	// Acquiring the lock to get resources
    farsa::ResourcesLocker locker(this);
    
	// Getting limit velocities for wheels
	double minSpeed1;
	double minSpeed2;
	double maxSpeed1;
	double maxSpeed2;
	m_robot->wheelsController()->getSpeedLimits(minSpeed1, minSpeed2, maxSpeed1, maxSpeed2);

	// Computing desired wheel velocities. When appying noise we don't check boundaries, so robots could go a bit faster than their maximum speed
	m_neuronsIterator->setCurrentBlock(name());
    

    maxact = -1.0;
    maxi = 0;
    for (int i=0; i < m_numOutput; i ++)
    {
        ovalues[i] = m_neuronsIterator->getOutput();
        m_neuronsIterator->nextNeuron();
        if (i >= 6)
        {
            if (ovalues[i] > maxact)
            {
                maxi = i - 6;
                maxact = ovalues[i];
            }
        }
    }


    // we store the current selected mode in a a variable of the experimental plugin
    pexp = getResource<MarxBotCleaningExperiment>("experiment");
    
    int interval = 2500;
    if (motormode == 1)
    {
        for(v=0, vi=0; v < 12000; v+= interval, vi++)
        {
         if (pexp->m_modstep >= v && pexp->m_modstep < (v + interval))
           maxi = vi % 3;
        }
    }
    
    if (motormode == 2)
        maxi = 0;

    if (m_numOutput > 2)
    {
      m_neuronsIterator->setCurrentBlock(name());
      for (int i=0; i < m_numOutput; i ++)
      {
         if (((maxi == 0 && (i < 2)) || (maxi == 1 && (i >= 2 && i < 4))  || (maxi == 2 && (i >= 4 && i < 6)) || i >= 6))
           m_neuronsIterator->setGraphicProperties("W" + QString::number(i), 0.0, 1.0, Qt::blue);
         else
           m_neuronsIterator->setGraphicProperties("W" + QString::number(i), 0.0, 1.0, Qt::gray);
         m_neuronsIterator->nextNeuron();
      }
    }
    
    switch (maxi)
    {
        case 0:
            v1 = (maxSpeed1 - minSpeed1) * ovalues[0] + minSpeed1;
            v2 = (maxSpeed2 - minSpeed2) * ovalues[1] + minSpeed2;
            break;
        case 1:
            v1 = (maxSpeed1 - minSpeed1) * ovalues[2] + minSpeed1;
            v2 = (maxSpeed2 - minSpeed2) * ovalues[3] + minSpeed2;
            break;
        case 2:
            v1 = (maxSpeed1 - minSpeed1) * ovalues[4] + minSpeed1;
            v2 = (maxSpeed2 - minSpeed2) * ovalues[5] + minSpeed2;
            break;
    }
    
    
    //decrease energy
    pexp->m_selmod = maxi;

    pexp->m_energy -= (fabs(v1) / maxSpeed1) + (fabs(v2) / maxSpeed1);
    
	//const double v1 = (maxSpeed1 - minSpeed1) * applyNoise(m_neuronsIterator->getOutput(), 0.0, -1.0) + minSpeed1;
	//m_neuronsIterator->nextNeuron();
	//const double v2 = (maxSpeed2 - minSpeed2) * applyNoise(m_neuronsIterator->getOutput(), 0.0, -1.0) + minSpeed2;

	m_robot->wheelsController()->setSpeeds(v1, v2);

}


int MarXbotWheelVelocityMotorMod::size()
{
    return m_numOutput;
}

void MarXbotWheelVelocityMotorMod::resourceChanged(QString resourceName, ResourceChangeType changeType)
{
	// Calling parent function
	MarXbotMotor::resourceChanged(resourceName, changeType);

	if (changeType == Deleted) {
		return;
	}

	if (resourceName == m_marxbotResource) {
		m_robot = getResource<farsa::PhyMarXbot>();
	} else if (resourceName == m_neuronsIteratorResource) {
		m_neuronsIterator = getResource<farsa::NeuronsIterator>();
		m_neuronsIterator->setCurrentBlock(name());
		for (int i = 0; i < size(); i++, m_neuronsIterator->nextNeuron())
        {
            if (i < 6)
                m_neuronsIterator->setGraphicProperties("W" + QString::number(i), 0.0, 1.0, Qt::red);
            else
                m_neuronsIterator->setGraphicProperties("S" + QString::number(i-2), 0.0, 1.0, Qt::red);					}
	} else {
		farsa::Logger::info("Unknown resource " + resourceName + " for " + name());
	}
}
