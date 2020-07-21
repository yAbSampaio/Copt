from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
from io import StringIO
import csv
import os
import subprocess

dataset = []
steps = 50000
Dire = '50_60ks'
mode = 'Simulação Manual 10-20k de steps'
stepsDown = 51500

while steps < 51000:
    print(steps)
    os.chdir("../{}".format(Dire))
    with open("trage_s{}_k_step.csv".format(steps)) as pointer:
        plain_text = pointer.read()
        payload = plain_text.split("=")
        print(len(payload))
        trial_info = pd.read_csv("infos_{}_k_step.csv".format(steps), sep=";")
        trials = trial_info['total_fitness']
        trial_infos = trials[trials>0.90]
        for i, trial in enumerate(payload):
            if len(trial) > 0:
                dataset.append([pd.read_csv(StringIO(trial), sep=";"), trial_infos])
        print(len(dataset))
        chunks = 5
        size_ms = 1
        colors = ['red', 'blue', 'gray', 'magenta', 'green']
        for trial, data in enumerate(dataset):
            print(trial)
            times = data[0]['x'].size // chunks
            fig, ax = plt.subplots(figsize=(10, 10))
            labels, lines = [], []
            total_fitness = data[1]*100
            aux = total_fitness.tolist()
            
            ax.set_title('Posição do Robô (Trial {}) - {:.2f}% limpo.'.format(trial+1, aux[trial]))
            for it in range(chunks):
                batch = data[0][['x','y']].loc[it*times:(it+1)*times]
                ax.scatter(batch['x'], batch['y'], c=colors[it], s=0.1, alpha=1)
                lines.append(plt.Line2D([0], [0], color=colors[it], lw=4))
                labels.append("step $\exists$ [{},{}]".format(it*times, (it+1)*times))
            ax.legend(lines, labels,  bbox_to_anchor = (0,-0.1,1,1))
            plt.savefig('../plot/trage/trage_Stps_{}_.png'.format(steps), dpi=100)
    dataset.clear()
    steps += 2000
    #stepsDown = int((steps/5)-4500)
    os.chdir("../cod_plots")
