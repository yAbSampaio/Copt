void MarxBotCleaningExperiment::noiseRun(){
    noiseMotorA = 0;
    noiseMotorB = 0;
    float Noiserate;
    if (nvlNoise == 1){
	    int aux =(rand()%10);
        if (aux == 5){
            Noiserate =  ((rand()%100)/100)*0.05;
            noiseMotorA = (10*Noiserate);
            Noiserate =  ((rand()%100)/100)*0.05;
            noiseMotorB = (10*Noiserate);
        }
    }
    else if (nvlNoise == 2){
        int aux =(rand()%4);
        if (aux == 1){
            Noiserate =  ((rand()%100)/100)*0.1;
            noiseMotorA = (10*Noiserate);
            Noiserate =  ((rand()%100)/100)*0.1;
            noiseMotorB = (10*Noiserate);
        }
    }
    else if (nvlNoise == 3){
        int aux =(rand()%3;
        if (aux == 2){
            Noiserate =  ((rand()%100)/100)*0.15;
            noiseMotorA = (10*Noiserate);
            Noiserate =  ((rand()%100)/100)*0.15;
            noiseMotorB = (10*Noiserate);
        }
    }
}

int MarxBotCleaningExperiment::noiseActive(){
    if (nvlNoise == 2){
        int auxPos =(rand()%20);
        if(auxPos == 12){
            noisePos = 0.02;
            return 1;
        }
        return 0;
    }
    else if (nvlNoise == 3){
        int auxPos =(rand()%10);
        int auxAng =(rand()%1000);
        rateAngAct = 0;
        if(auxPos == 7){
            noisePos = 0.05;
            if (auxAng == 20){
                rateAngAct = 0.01;
                return 1;
            }
            else{
                return 1;
            }
        }
        else if (auxAng == 254){
            rateAngAct = 0.01;
            return 1;
        }
        return 0;
    }
}
		  
		  
		  
		  
		  
/-------------------------------------------------------------/
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
    noiseRun();
    if (Running && Clock <= endClock)
    {
        if (evonet->getInput(5) > NEAR_SENSOR && evonet->getInput(6) > NEAR_SENSOR)
        {
            //PRINT_DEV <<<< "Tem parede, parou de andar" << PRINTEND_DEV;
            m_robot->wheelsController()->setSpeeds(((-VelPerStepForOneDistance+noiseMotorA) / 2), ((-VelPerStepForOneDistance+noiseMotorB) / 2));
            Running = False;
            return 1;
        }
        else
        {
            m_robot->wheelsController()->setSpeeds((VelPerStepForOneDistance-noiseMotorA), (VelPerStepForOneDistance-noiseMotorB));
            PRINT_DEV << "(" << robot->position().x << ", " << robot->position().y << ")" << PRINTEND_DEV;
            return 0;
        }
    }
    else if (Running && Clock > endClock)
    {
        PRINT_DEV << "Finda a caminhada (" << robot->position().x << ", " << robot->position().y << ")" << PRINTEND_DEV;
        Running = False;
        m_robot->wheelsController()->setSpeeds(((-VelPerStepForOneDistance+noiseMotorA) / 1.5), ((-VelPerStepForOneDistance+noiseMotorA) / 1.5));
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
