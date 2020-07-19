import os
import sys
import subprocess

# In this section is defined variables necessary for further simulation.
# The variable 'steps' serves to indicate minimum value of 'steps' that the 
# simulation will have, ie how many cycles will she perform.
# The variables 'functUp' and 'functDown' only indicate what kind of functions will be
# used to increment or decrement the clock.

steps = 10000
# functUp = 'linear'
# functDown = 'linear'
simulation = "temporalE2Release"
path_base = os.path.join(os.path.abspath(os.path.dirname(__file__)), '..')
# dirs = ['expo', 'sub_e', 'sub_l', 'linear']
dirs = ['10_20ks','30_40ks','50_60ks','70_80ks','90_100ks']
workdir = ''
threshold = 0.90

# The variables 'simulation' and 'dirs' are directories that will be used later
# to get the configuration file and save the simulation data respectively.
# Variable 'threshold' is used to select which simulations will have your route saved.

# Is made a for in the dirs list and checked if it already exists,
# otherwise one with the same name is created.

for _dir in dirs:
    d = os.path.join(path_base, 'data', _dir)
    if not os.path.isdir(d):
        os.mkdir(d)

# Here is main for loop, where it will be called the Farsa
# until the number of steps reaches 300 thousand,
# with a logic of always keep a proportion 
# in 'stepsDown' and 'stepsUp' of 95% and 5% respectively.

os.chdir("../")
while steps < 100000:
    # stepsDown = int((steps/5)*.95)
    # stepsUp = int((steps/5)*.05) 
    tr = os.path.join(path_base, 'data', 'trial_info.csv')
    prb = os.path.join(path_base, 'data', 'position_robot.csv')
    if os.path.isfile(tr):
        os.remove(tr)
    if os.path.isfile(prb):
        os.remove(prb)

    sys.stdout.writelines("\r Simulando com {0} steps".format(/steps))
    # After checking the files that will be used to save the data
    # the Farsa program call is started, to work this way was included
    # in the configuration.ini file some previous information and because of that we can change
    # via the command line the variables we want made by the command "-PComponent / GA / Experiment / ..."
    # after adjusting the variables the call is made and the program displays the characteristics of the current simulation
    # to be running in the background
    sys.stdout.flush()
    action_cmd = ["-PComponent/GA/Experiment/nsteps={}".format(steps),
                  "-PComponent/GA/Experiment/threshold={}".format(threshold)]
    cmd_args = ["total99", "--batch", "--file=configuration.ini",
                "--action=runTest", *action_cmd]
    os.chdir("simulations/{}".format(simulation))
    proc = subprocess.Popen(
        cmd_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, error = proc.communicate()
    
    # At the end of the simulation the data files are reallocated according to the function used
    # and is renamed for its features
    
    if error.decode("utf-8") == '':
        os.chdir("../../data")
        if steps < 30000 :
            workdir = '10_20ks'
        elif steps < 50000:
            workdir = '30_40ks'
        elif steps < 70000:
            workdir = '50_60ks'
        elif steps < 90000:
            workdir = '70_80ks'
        else:
            workdir = '90_100ks'
        os.rename("trial_info.csv",
                    "{}/infos_{}_k_step.csv".format(workdir, steps))
        os.rename(
            os.path.join("position_robot.csv"),
            os.path.join(workdir, "trage_s{}_k_step.csv".format(
                steps))
        )

    else:
        print("Erro ao executar o experimento. Parametros:\n Steps {}".format(steps))
    steps += 5000
    os.chdir("../")
