import os
from multiprocessing import Process
import cma
import time
import math as m
import matplotlib.pyplot as plt

path = '../../../../../binaries'
os.chdir(path)
#Op agent is caught from setting the decision making in /itandroids-vss/configs/decision_making/3v3/strategy_mode.conf as OPTIMIZATION_EXTREME_ATTACKERS
#Def agent is caught from setting he decision making in /itandroids-vss/configs/decision_making/3v3/strategy_mode.conf as DEFENSIVE
procs = ['./UnivectorExtremeAttackersOptimization_AgentTest -matches 20 -noGUI -silent', './op_agent -1s -noProj -noGUI',
         './def_agent -2s -noProj -noGUI']


def run_process(process):
    os.system(process)


def run_simulator():
    processes = []
    for p in procs:
        proc = Process(target=run_process, args=(p,))
        processes.append(proc)
        proc.start()

    processes[0].join()
    processes[1].terminate()
    processes[2].terminate()


def cost_function(x):
    wr = open("../configs/trajectory_planner/univector/univector_shoot.conf", 'w')
    parameters = "de:" + str(round(x[0], 4)) + "\nKr:" + str(round(x[1], 4)) + "\nsigma:" + str(
        round(x[2], 4)) + "\ndmin:" + str(round(x[3], 4)) + "\nKe:" + str(round(x[4], 4)) + "\nK0:" + str(
        round(x[5], 4)) + "\nsigmaLine:" + str(round(x[6], 4)) + "\ndminLine:" + str(
        round(x[7], 4)) + "\nsigmaOpponent:" + str(round(x[8], 4)) + "\nsigmaBall:" + str(round(x[9], 4)) + "\n"
    wr.write(parameters)
    wr.close()

    run_simulator()

    output = open("../source/tools/optimization/univector/cost_extreme_optimization.conf", "r")
    val = float(output.readline().split(' ')[0])
    return val


it = 0
iterations = []
best_f_values = []


def callback(cmaEvolutionStrategy):
    global totalEvaluations
    global it
    global best_f_values

    current = cmaEvolutionStrategy.result

    it += 1
    iterations.append(it)
    best_f_values.append(current.fbest)


def plot():
    # save plot data
    wr = open("plot_data.txt", 'w')
    data = "Iterations: " + str(iterations) + "\nBest f-values: " + str(best_f_values);
    print(data)
    wr.write(data)
    wr.close()

    plt.plot(iterations, best_f_values, 'b-', label='best f-value')
    plt.legend()
    plt.xlabel('Iterations')
    plt.ylabel('f-value (s)')
    plt.grid()
    plt.savefig('plot_second_optimization.png')


def main():
    options = {
        'maxiter': 50,  # máximo de iterações
        'bounds': [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], None]
        # limites pra solução [array com mínimos, array com máximos]
    }
    sigma_start = 0.1
    x_start = [0.20, 0.32, 0.00, 0.05, 2.87, 0.04, 0.00, 0.00, 0.00, 0.55]

    res = cma.fmin(cost_function, x_start, sigma_start, options, callback=callback)

    print(res[0])
    wr = open("../source/tools/optimization/univector/output.conf", 'w')
    wr.write('================================\n')
    wr.write('OPTIMIZED PARAMETERS:\n')
    x = res[0]
    parameters = "de:" + str(x[0]) + "\nKr:" + str(x[1]) + "\nsigma:" + str(x[2]) + "\ndmin:" + str(
        x[3]) + "\nKe:" + str(x[4]) + "\nK0:" + str(x[5]) + "\nsigmaLine:" + str(x[6]) + "\ndminLine:" + str(
        x[7]) + "\nsigmaOpponent:" + str(x[8]) + "\nsigmaBall:" + str(x[9]) + "\n"
    wr.write(parameters)
    wr.write('================================\n')
    wr.write('Function value of optimized parameters: ' + str(res[1]) + '\n')
    wr.write('================================\n')
    wr.write('Termination conditions:' + str(res[-3]) + '\n')
    wr.close()

    plot()

main()
