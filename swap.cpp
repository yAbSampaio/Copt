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
                    cleanRoom ++;
                    last_Walk = 0;
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
        if ( cleanRoom > 2 ){
            cleanRoom = 2;
        }
        demandRoom();
    }
    
}//

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
