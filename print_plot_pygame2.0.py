import logging
import numpy as np
import pygame
import threading
from time import sleep
from datetime import datetime
import os
import shutil

def plot_threading(thread_num, win_th, array, start = 0, end = 0, step = 1, color = (0,200,200), ratio = 50):
    global TRUE_BOOL
    nan = float("nan")
    i = start
    cont = 1
    trial = 1
    addtrial = 0
    #coment
    color1 = 255
    color2 = 0
    color3 = 0
    stepcolor = 1
    stepcc = 1
    #coment
    while(i < end-step):

        try:
            x1 = round(ratio*array[i][0])
            y1 = round(ratio*array[i][1])
            x = int(500 + x1 )
            y = int(500 + -1*y1 )
            #coment
            color = (color1, color2, color3)

            if ( stepcc == 1 ):
                if( color2 > 255-stepcolor ):
                    stepcc = 2
                else:
                    color2 += stepcolor

            if ( stepcc == 2 ):
                if( color1 < stepcolor ):
                    stepcc = 3
                else:
                    color1 -= stepcolor

            if ( stepcc == 3 ):
                if( color3 > 255-stepcolor ):
                    stepcc = 4
                else:
                    color3 += stepcolor

            if ( stepcc == 4 ):
                if( color2 < stepcolor ):
                    stepcc = 5
                else:
                    color2 -= stepcolor

            if ( stepcc == 5 ):
                if( color1 > 255-stepcolor ):
                    stepcc = 6
                else:
                    color1 += stepcolor

            if ( stepcc == 6 ):
                if( color3 <= stepcolor ):
                    stepcc = 1
                else:
                    color3 -= stepcolor
            
            #coment
            # print(color)
            win_th.set_at((x, y), color)
        except ValueError:
            #coment
            color1 = 255
            color2 = 0
            color3 = 0
            stepcc = 1
            print("end trial ", i)
            i += step
            pygame.image.save(win, DIR+"/Plot_{}.jpg".format(cont))
            cont += 1
            # sleep(1)
            win_th.fill((0, 0, 0))
            trial += 1
            Centros = []
            if( np.isnan(trial_infos[trial+addtrial][0]) ):
                print("addtrial: ",addtrial)
                addtrial += 1

            for j in range(int( trial_infos[trial+addtrial][1]) ):
                X_room = int( 500 + ratio*trial_infos[trial+addtrial][2+j*4] )
                Y_room = int( 500 + -ratio*trial_infos[trial+addtrial][3+j*4] )
                W_room = int(ratio*trial_infos[trial+addtrial][4+j*4] )
                H_room = int(ratio*trial_infos[trial+addtrial][5+j*4] )
                # print("dados: {} - {} - {} - {} \n".format(X_room, Y_room, W_room, H_room))
                Centros.append(int(X_room))
                Centros.append(int(Y_room))
                pygame.draw.rect(win,(10,10,80),(int(X_room - W_room/2),int(Y_room - H_room/2),W_room, H_room))
            j = 0
            while( j < len(Centros) -2 ):
                pygame.draw.line(win, (10,10,80), (Centros[0], Centros[1]), (Centros[j+2], Centros[j+3]), int(ratio*0.25))
                j += 2
            continue
        # print
        i += step

        # sleep(0.005)
    
    print("end trial ", i)
    pygame.image.save(win, DIR+"/Plot_{}.jpg".format(cont))

    print("thread ", thread_num, "terminou seu trabalho", i)
    try:
        TRUE_BOOL[thread_num] = False
    except:
        print("algo de errado aconteceu")

if __name__ == "__main__":

    # DEFININDO CORES
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    BLUE = (0, 0, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)

    #PASTA DE PLOT
    DIR = "Plot/"+datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    os.makedirs(DIR)

    # FAZENDO BACKUP DO ARQUIVOS DE CONTROLE
    shutil.copy("src/evo/marxbotcleaningexperiment.cpp", DIR+"/marxbotcleaningexperiment.cpp", follow_symlinks=True)

    # LEITURA DOS DADOS E INCIALIZAÇÃO DA JANELA
    shutil.move("data/position_robot.csv", DIR+"/position_robot.csv")
    shutil.move("data/trial_info.csv", DIR+"/trial_info.csv")
    numpy_array = np.genfromtxt(DIR+"/position_robot.csv", delimiter=";", skip_header=1)
    trial_infos = np.genfromtxt(DIR+"/trial_info.csv", delimiter=";")

    # MAKING A WINDOW
    pygame.init()
    win = pygame.display.set_mode()
    win = pygame.display.set_mode((1000, 1000), 0, 8)
    win.fill(BLACK)
    pygame.display.flip()
    colors = [(0, 200, 0),(0, 200, 200),(200, 0, 0), (0, 0, 200)]

    # DEFINIÇÃO DE VARIAVEIS
    num_thread = 1
    TRUE_BOOL = [True]*num_thread
    gap = int(len(numpy_array)/num_thread)
    ratio = 60
    init = 0
    end = gap
    step = 1

    # DRAWING ROOMS
    Centros = []
    for i in range( int( trial_infos[1][1]) ):
        X_room = int( 500 + ratio*trial_infos[1][2+i*4] )
        Y_room = int( 500 + -ratio*trial_infos[1][3+i*4] )
        W_room = int(ratio*trial_infos[1][4+i*4] )
        H_room = int(ratio*trial_infos[1][5+i*4] )
        # print("dados: {} - {} - {} - {} \n".format(X_room, Y_room, W_room, H_room))
        Centros.append(int(X_room))
        Centros.append(int(Y_room))
        pygame.draw.rect(win,(10,10,80),(int(X_room - W_room/2),int(Y_room - H_room/2),W_room,H_room))
    i = 0
    while( i < len(Centros) -2 ):
        pygame.draw.line(win, (10,10,80), (Centros[0], Centros[1]), (Centros[i+2], Centros[i+3]), int(ratio*0.25))
        i += 2

    pygame.display.flip()
    # sleep(3)
    
    # MENSAGENS DE LOG'S ANTES DA EXECUÇÃO
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
    logging.info("Main    : antes de criar a thread")

    # INICIO DO LOOP
    for i in range(num_thread):
        x = threading.Thread(target=plot_threading, args=(i, win, numpy_array, init, end, step, colors[i], ratio))
        x.start()
        init += gap
        end += gap

    while(True in TRUE_BOOL):
        pygame.display.flip()

        for event in pygame.event.get():
            if ( event.type == pygame.KEYDOWN ):
                if ( event.key == pygame.K_SPACE ):
                    pygame.display.quit()
                    TRUE_BOOL = [False]
                    break

    # pygame.image.save(win,"screenshot.jpg")
    while(True):
        for event in pygame.event.get():
            if ( event.type == pygame.KEYDOWN ):
                if ( event.key == pygame.K_SPACE ):
                    pygame.display.quit()
                    break
