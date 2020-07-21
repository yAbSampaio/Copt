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
best = []

steps = list(range(10000,102000,2000))
#print(steps)
Dire = ['10_20ks','30_40ks','50_60ks','70_80ks','90_100ks']
mode = ['Simulação Manual 10-20k de steps','Simulação Manual 30-40k de steps','Simulação Manual 50-60k de steps','Simulação Manual 70-80k de steps','7imulação Manual 90-100k de steps']
#stepsUp = [1000,1500,2000]
#stepsDown = [5000,10000,15000,20000]
#combi = list(iter.product(stepsUp,stepsDown))

os.chdir("../")
for mod,folder in enumerate(Dire):
    os.chdir("{}".format(folder))
    for steps_i in steps:
        if os.path.isfile("infos_{}_k_step.csv".format(steps_i)):
            # opening the data and separating by number of rooms
            info_Trial = pd.read_csv("infos_{}_k_step.csv".format(steps_i), sep=";")

            values_Fit = info_Trial['total_fitness']
            total_Trial = values_Fit
            stati_Data = values_Fit.describe()
            #print(total_Trial)

            #vec_quartis1.append(stati_Data['25%'])
            #vec_quartis2.append(stati_Data['75%'])
            #vec_med.append(stati_Data['50%'])
            med = stati_Data['50%']
            quar1 = stati_Data['25%']
            quar2 = stati_Data['75%']


            #Boxplot startup and analysis
            s_q1 = []
            s_mediana = []
            s_q2 = []
            legenda = "Steps".format(steps_i)




            s_q1.append("{0:.2f}".format(quar1))
            s_mediana.append("{0:.2f}".format(med))
            s_q2.append("{0:.2f}".format(quar2))
        
            font_1 = {'family': 'serif', 'color': '#5C1BCC', 'size':'14'}

            plt.figure(figsize=(25, 15))
            plt.boxplot(total_Trial)
            plt.xticks([1], legenda)
            plt.title('Boxplot Fitness - Modo {} - {}k Steps'.format(mode[mod],steps_i))
            plt.ylabel('% de Limpeza')
            plt.xlabel('Tempo de Subida e Descida')

            
            plt.text(1, quar1, s_q1[0], fontdict=font_1)
            plt.text(1, med, s_mediana[0], fontdict=font_1)
            plt.text(1, quar2, s_q2[0], fontdict=font_1)
            plt.savefig('../plot/{}K_boxplot.png'.format(steps_i))
        #s_q1.clear()
        #s_q2.clear()
        #s_mediana.clear()
        #vec_med.clear()
        #vec_quartis1.clear()
        #vec_quartis2.clear()
    os.chdir("../")
