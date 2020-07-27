void MarxBotCleaningExperiment::Cleaning(){
    farsa::ResourcesLocker locker(this);
    farsa::RobotOnPlane *robot =
        getResource<farsa::RobotOnPlane>("agent[0]:robot");
    farsa::Evonet *evonet = getResource<farsa::Evonet>("evonet");
    farsa::MarXbot *m_robot = dynamic_cast<farsa::MarXbot *>(robot);
    //go find some corner, and go to the corner
    if (effect == 0){
        if (PositionInTheCorner == 0){
            if ( RunRobotW3(0, 0, 0, 0) ){
                std::cout << "corner 0 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
                PositionInTheCorner = 1;
            }
        }

        else if (PositionInTheCorner == 1){
            if ( TurnRobot3(0, 1, 0, 0) ){
            std::cout << "corner 1 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
                PositionInTheCorner = 2;
            }
        }

        else if (PositionInTheCorner == 2){
            PRINT_DEV << "corner 2 Clock: " << evonet->getInput(8) << " position:("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << "sensors " << frontSensor() << " - " << rightSensor() << " - " << backSensor() << " - " << leftSensor() << PRINTEND_DEV;
            // PRINT_DEV << "sensors back: " << evonet->getInput(1) << " - " << evonet->getInput(1) << PRINTEND_DEV;
            if ( RunRobotF3(1, 1, 0, 0) ){
                std::cout << "corner dentro 2 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
                PositionInTheCorner = 3;
            }
        }

        else if (PositionInTheCorner == 3){
            // std::cout << "corner 3 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
            if ( TurnRobot3(0, 1, 1, 0) ){
                PositionInTheCorner = 0;
                effect = 1;
                std::cout << "corner 3 Clock: " << evonet->getInput(8) << " position: ("<< robot->position().x << ", " << robot->position().y << ") || Orientation: " << getAngle() << std::endl;
            }
        }
        
    }
    // NOW THE ROBOT IS POSITIONED
    else if ( effect == 1 ){
        if ( rightSensor() == 1){    
            if (RunRobotW3(1, 1, 0, 0)){
                if (last_Walk == 1){
                    effect = 5;
                }
                else{
                    effect = 2;
                    last_Walk += 1;
                }
                
            }
        }
        else if ( leftSensor() == 1 ){
            if (RunRobotW3(1, 0, 0, 1)){
                effect = 2;
            }
        
        }
        else{
            if (RunRobotW3(1, 0, 0, 0)){
                effect = 2;
            }
        
        }
        
        
    }

    else if( effect == 2 ){
        if (last_track == 1){
            if ( rightSensor() == 1 ){//Teoricamente nao deveria entrar aki pois deveria entrar no effect 5 porem pra previnir eu fiz
                if ( TurnRobot3(1,0,0,1)){
                    effect = 4;
                }
                
            }
            else{
                if( TurnRobot3(0, 0, 0, 1) ){
                    effect = 3;
                }
            }
        }
        else{
            if ( rightSensor() == 1 ){
                if ( TurnRobot3(0,1,1,0)){
                    effect = 3;
                }
                
            }
            else if ( leftSensor() == 1 ){
                if ( TurnRobot3(1,1,0,0) ){
                    effect = 4;
                }
                
            }
            else{
                if( TurnRobot3(0, 1, 0, 0) ){
                    effect = 3;
                }
            }
        }
        
    }

    else if( effect == 3 ){
         if (RunRobot(diameter_robot)){
             effect = 4;
         }
         
    }
    
    else if ( effect == 4 ){
        if (last_track == 1){
            if ( frontSensor() == 1 ){//Teoricamente nao deveria entrar aki pois deveria entrar no effect 5 porem pra previnir eu fiz
                if ( TurnRobot3(0,0,1,1)){
                    effect = 1;
                }
                
            }
            else{
                if( TurnRobot3(0, 0, 1, 0) ){
                    effect = 1;
                }
            }
        }
        else{
            if ( frontSensor() == 1 ){
                if ( TurnRobot3(0,1,1,0)){
                    effect = 3;
                }
                
            }
            else{
                if( TurnRobot3(0, 0, 1, 0) ){
                    effect = 1;
                }
            }
        }
    }// /
    else if (effect == 5){
        demandRoom();
    }
    
}//
