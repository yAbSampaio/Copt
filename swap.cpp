float MarxBotCleaningExperiment::noiseRun(){
    noiseMotor = 0;
    srand (static_cast <unsigned> (time(0)));
    if (nvlNoise == 1){
	    int aux =(rand()%10);
        if (aux == 5){
            nvlNoise =  ((rand()%100)/100)*0.05;
            noiseMotor = (10*nvlNoise);
            return noiseMotor;
        }
    }
    else if (nvlNoise == 2){
        int aux =(rand()%4);
        if (aux == 2){
            nvlNoise =  ((rand()%100)/100)*0.1;
            noiseMotor = (10*nvlNoise);
            return noiseMotor;
        }
    }
    else if (nvlNoise == 3){
        int aux =(rand()%3;
        if (aux == 2){
            nvlNoise =  ((rand()%100)/100)*0.15;
            noiseMotor = (10*nvlNoise);
            return noiseMotor;
        }
    }
    return noiseMotor;
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
        else if (auxAng == 25){
            rateAngAct = 0.01;
            return 1;
        }
        return 0;
    }
}
                  
/////////////////////////////////////////////////////////////////////////////////////////////
                  
                  
                  
 // Variables creates for noise

	int nvlNoise = 1;
	int noiseMotor = 0;
	int ratePosAct = 0;
	int noisePos = 0;
	int rateAngAct = 0;
