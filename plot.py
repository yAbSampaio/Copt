from matplotlib import pyplot as plt
import itertools as iter
import pandas as pd
import numpy as np
from io import StringIO
import csv
import os
import subprocess

vec_quartis1 = []
vec_quartis2 = []
vec_med = []
stati_Data = []
total_room = []
fit_byRoom = [[],[],[]]
legenda = [3,4,5]

bestRoom3 = []
bestRoom4 = []
bestRoom5 = []

steps = list(range(10000,51000,2000))
Dire = ['10_20ks','30_40ks','50_60ks','70_80ks','90_100ks']
mode = ['Simulação Manual 10-20k de steps','Simulação Manual 30-40k de steps','Simulação Manual 50-60k de steps','Simulação Manual 70-80k de steps','7imulação Manual 90-100k de steps']

os.chdir("../")
for mod,folder in enumerate(Dire):
    os.chdir("{}".format(folder))
    for steps_i in steps:
        if os.path.isfile("infos_{}_k_step.csv".format(steps_i)):
            # opening the data and separating by number of rooms
            fit_byRoom = [[],[],[]]
            info_Trial = pd.read_csv("infos_{}_k_step.csv".format(steps_i), sep=";")

            total_Trial = info_Trial['total_fitness'] 
            total_room = info_Trial['rooms']

            for i,room in enumerate(total_room):
                fit_byRoom[room-3].append(total_Trial[i])
                
            for w in range(len(fit_byRoom)):
                fit_byRoom[w] = pd.DataFrame(fit_byRoom[w],columns=['fit'])
            
            room3 = fit_byRoom[0]['fit']
            room4 = fit_byRoom[1]['fit']
            room5 = fit_byRoom[2]['fit']
            
            for j in range(len(fit_byRoom)):
                if j == 0:
                    stati_Data.append(room3.describe())
                    vec_quartis1.append("{0:.2f}".format(stati_Data[j]['25%']))
                    vec_quartis2.append("{0:.2f}".format(stati_Data[j]['75%']))
                    vec_med.append("{0:.2f}".format(stati_Data[j]['50%']))
                elif j == 1:
                    stati_Data.append(room4.describe())
                    vec_quartis1.append("{0:.2f}".format(stati_Data[j]['25%']))
                    vec_quartis2.append("{0:.2f}".format(stati_Data[j]['75%']))
                    vec_med.append("{0:.2f}".format(stati_Data[j]['50%']))
                elif j == 2:
                    stati_Data.append(room5.describe())
                    vec_quartis1.append("{0:.2f}".format(stati_Data[j]['25%']))
                    vec_quartis2.append("{0:.2f}".format(stati_Data[j]['75%']))
                    vec_med.append("{0:.2f}".format(stati_Data[j]['50%']))

                # vec_quartis1.append("{0:.2f}".format(stati_Data[j]['25%']))
                # vec_quartis2.append("{0:.2f}".format(stati_Data[j]['75%']))
                # vec_med.append("{0:.2f}".format(stati_Data[j]['50%']))

            #Boxplot startup and analysis
            font_1 = {'family': 'serif', 'color': '#5C1BCC', 'size':'14'}
            
            plt.figure(figsize=(25, 15))
            plt.boxplot([room3,room4,room5])
            plt.xticks([1,2,3], [3,4,5])
            plt.title('Boxplot Fitness - Modo {} - {} Steps'.format(mode[mod],steps_i))
            plt.ylabel('% de Limpeza')
            plt.xlabel('Numero de sala')
            bestRoom3.append(float(vec_med[0]))
            bestRoom4.append(float(vec_med[1]))
            bestRoom5.append(float(vec_med[2]))

            for k in range(len(fit_byRoom)):
                
                plt.text(k+1.16, float((vec_quartis1[k])), str(vec_quartis1[k]), fontdict=font_1)
                plt.text(k+1.16, float((vec_quartis2[k])), str(vec_quartis2[k]), fontdict=font_1)
                plt.text(k+1.16, float((vec_med[k])), str(vec_med[k]), fontdict=font_1)
            plt.savefig('../plot/{}_boxplot.png'.format(steps_i))
            print("steps {}".format(steps_i))
            vec_med.clear()
            vec_quartis1.clear()
            stati_Data.clear()
            vec_quartis2.clear()
    os.chdir("../")

plt.figure(figsize=(35, 15))
plt.plot(steps,bestRoom3)
plt.title('plot Fitness - Integral ')
plt.ylabel('% de Limpeza')
plt.xlabel('Steps')
plt.savefig('plot/plot_Room3.png')

plt.figure(figsize=(35, 15))
plt.plot(steps,bestRoom4)
plt.title('plot Fitness - Integral ')
plt.ylabel('% de Limpeza')
plt.xlabel('Steps')
plt.savefig('plot/plot_Room4.png')

plt.figure(figsize=(35, 15))
plt.plot(steps,bestRoom5)
plt.title('plot Fitness - Integral ')
plt.ylabel('% de Limpeza')
plt.xlabel('Steps')
plt.savefig('plot/plot_Room5.png')
