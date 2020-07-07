int MarxBotCleaningExperiment::TurnRobot2( float degrees){ //here degrees is the final orientation of the robot
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane* robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Arena* arena = getResource<farsa::Arena>("arena");
    farsa::ResourceVector<farsa::real>* m_additionalInputs =
        getResource<farsa::ResourceVector<farsa::real> >
                                ("agent[0]:additionalInputs");
        farsa::Evonet* evonet = getResource<farsa::Evonet>("evonet");

    farsa::MarXbot* m_robot = dynamic_cast<farsa::MarXbot*>(robot);
    // Clock = evonet->getInput(8);
    
    std::cout<< "Orientation: " << getAngle() << std::endl;
    
    std::cout<< "Clock: " << Clock << "  ";
    if ( Turning /*&& Clock <= endClock*/ && ( ( abs(getAngle() - degrees) >= 1 ) || ( abs(getAngle() - degrees) <= 359 ) ) ){
        std::cout<< "Girando" << std::endl;
		m_robot->wheelsController()->setSpeeds(VelPerStepForOneDregrees*last_track, -VelPerStepForOneDregrees*last_track);
        return 0;
    }else if( Turning ){
        if( ( abs(getAngle() - degrees) < 1 ) && ( abs(getAngle() - degrees) > FAKE_ZERO ) ){
            double vel = VelPerStepForOneDregrees*( getAngle() - degrees );
            std::cout<< "delta(getAngle - degrees): " << getAngle() << " - " << degrees << " = " << getAngle()-degrees << "  vel: "<< vel <<std::endl;
            m_robot->wheelsController()->setSpeeds(vel, -vel);
            return 0;

        }else if( ( abs(getAngle()-360 - degrees) < FAKE_ZERO ) && ( abs(getAngle()-360 - degrees) > -1 ) ){
            double vel = VelPerStepForOneDregrees*( getAngle()-360 - degrees );
            std::cout<< "delta(getAngle - degrees): " << getAngle() << " - " << degrees << " = " << getAngle()-degrees << "  vel: "<< vel <<std::endl;
            m_robot->wheelsController()->setSpeeds(-vel, vel);
            return 0;
        }
    }else if( Turning /*&& Clock > endClock*/ && ( abs(getAngle() - degrees) <= FAKE_ZERO ) ){
        Turning = False;
        std::cout<< "Finda a girada, Orientation: "<< getAngle() <<std::endl;
        return 1;
    }else{
        std::cout<< "Inicia a girada" << std::endl;
        Turning = True;
        return 0;
	}
    return 0;
}
