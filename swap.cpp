int MarxBotCleaningExperiment::TurnRobotJ3(char side){ //turn the robot until just the asked sensors stay activated
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    if( (side == 'f' && frontSensor()) ||  (side == 'r' && rightSensor()) || (side == 'b' && backSensor()) || (side == 'l' && leftSensor()) ){
        // std::cout<< "sensors" << frontSensor() << " - " << rightSensor() << " - " << backSensor() << " - " << leftSensor() << std::endl;
        m_robot->wheelsController()->setSpeeds(1, 1);
        return True;
    }else
    {
        float v1 = 1, v2 = 1;
        int s1, s2;
        if(side == 'l'){
            s1 = evonet->getInput(0);
            s2 = evonet->getInput(7);
        }else if(side == 'r'){
            s1 = evonet->getInput(3);
            s2 = evonet->getInput(4);
        }else if(side == 'b'){
            s1 = evonet->getInput(1);
            s2 = evonet->getInput(2);
        }elseif(side == 'f'){
            s1 = evonet->getInput(5);
            s2 = evonet->getInput(6);
        }

        if( s1 < 0.5 || s2 < 0.5 ){
            v1 = 10; v2 = 10;
        }else if( abs(s1 - s2) <= 0.1 ){
            v1 = 1*(s1-s2);
            v2 = 1*(s1-s2);
        }else{
            v1 = 1; v2 = 1;
        }
        m_robot->wheelsController()->setSpeeds(v1, -v2);
        return False;
    }
}
