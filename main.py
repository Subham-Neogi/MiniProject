from __future__ import absolute_import
from __future__ import print_function

import os
import sys
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import matplotlib.pyplot as plt
import pandas as pd
from TrafficGenerator import TrafficGenerator
from SimRunner import SimRunner
plt.style.use('dark_background')

# sumo things - we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# PLOT AND SAVE THE STATS ABOUT THE SESSION
def save_graphs(sim_runner, plot_path):

    plt.rcParams.update({'font.size': 24})  # set bigger font size

    # cumulative wait
    data = sim_runner.cumulative_wait_store
    plt.plot(list(data.keys()),list(data.values()),label="Algorithm")
    plt.ylabel("Cumulative delay per vehicle (s)")
    plt.xlabel("Steps")
    plt.margins(0)
    min_val = min(data.values())
    max_val = max(data.values())
    plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'delay.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'delay_data.txt', "w") as file:
        for k,v in data.items():
                file.write("%s %s\n" % (k,v))
    
    #No.of cars
    data = sim_runner.no_of_cars
    plt.plot(list(data.keys()),list(data.values()))
    plt.ylabel("No. of vehicle (s)")
    plt.xlabel("Steps")
    plt.margins(0)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'cars_number.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'cars_number.txt', "w") as file:
        for k,v in data.items():
                file.write("%s %s\n" % (k,v))
    # average number of cars in queue
    data = sim_runner.avg_intersection_queue_store
    plt.plot(data)
    plt.ylabel("Queue length (vehicles)")
    plt.xlabel("Steps")
    plt.margins(0)
    min_val = min(data)
    max_val = max(data)
    plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'queue.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'queue_data.txt', "w") as file:
        for item in data:
                file.write("%s\n" % item)
    
    # Total wait till cycle k
    data = sim_runner.total_wait_time_store
    plt.plot(list(data.keys()),list(data.values()),label="algorithm")
    plt.ylabel("Total wait")
    plt.xlabel("cycle")
    plt.margins(0)
    min_val = min(data.values())
    max_val = max(data.values())
    plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'cycles.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'cycle_wait_data.txt', "w") as file:
        for k,v in data.items():
                file.write("%s %s\n" % (k,v))

    
    # green duration
    data=sim_runner.green_duration_store
    plt.plot(range(len(data)),data,label="algorithm")
    plt.ylabel("Total green duration")
    plt.xlabel("cycle")
    plt.margins(0)
    min_val = min(data)
    max_val = max(data)
    plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'green_duration.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'green_duration_data.txt', "w") as file:
        for k in data:
                file.write("%s \n" % str(k))
    
    # mu plot
    data = sim_runner.mu_store
    plt.plot(data,label="algorithm")
    plt.ylabel(r"$\mu$")
    plt.xlabel("cycles")
    plt.margins(0)
    min_val = min(data)
    max_val = max(data)
    plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'mu.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'mu.txt', "w") as file:
        for item in data:
                file.write("%s\n" % item)

    # sigma plot
    data = sim_runner.sigma_store
    plt.plot(data,label="algorithm")
    plt.ylabel(r"$\sigma$")
    plt.xlabel("cycles")
    plt.margins(0)
    min_val = min(data)
    max_val = max(data)
    plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'sigma.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'sigma.txt', "w") as file:
        for item in data:
                file.write("%s\n" % item)
    
    # cycle duration plot
    data = sim_runner.cycle_duration
    plt.plot(data,label="algorithm")
    plt.ylabel("Cycle duration")
    plt.xlabel("cycles")
    plt.margins(0)
    min_val = min(data)
    max_val = max(data)
    plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'cycle_duration.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'cycle_duration.txt', "w") as file:
        for item in data:
                file.write("%s\n" % item)
    
    #vs no. of cars
    data = sim_runner.total_wait_vs_cars
    plt.plot(list(data.keys()),list(data.values()))
    plt.ylabel("Total wait")
    plt.xlabel("No.of cars")
    plt.margins(0)
    #min_val = min(data.values())
    #max_val = max(data.values())
    #plt.ylim(min_val - 0.05 * min_val, max_val + 0.05 * max_val)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'total_wait_vs_cars.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'total_wait_vs_cars.txt', "w") as file:
        for k,v in data.items():
                file.write("%s %s\n" % (k,v))

    data = sim_runner.wait_time_vs_car
    plt.plot(list(data.keys()),list(data.values()))
    plt.ylabel("Current waiting time")
    plt.xlabel("No. of cars")
    plt.margins(0)
    fig = plt.gcf()
    fig.set_size_inches(20, 11.25)
    fig.savefig(plot_path + 'current_wait_vs_car.png', dpi=96)
    plt.close("all")
    with open(plot_path + 'current_wait_vs_cars.txt', "w") as file:
        for k,v in data.items():
                file.write("%s %s\n" % (k,v))
    

    


if __name__ == "__main__":

    # --- SUMO OPTIONS ---
    gui = False

    # attributes of the simulation
    max_steps = 8000 
    green_duration = {0:20,1:20,2:20,3:20}
    yellow_duration = 10
    path = "./results/" 

    # setting the cmd mode or the visual mode
    if gui == False:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # initializations
    traffic_gen = TrafficGenerator(max_steps)
    sumoCmd = [sumoBinary, "-c", "intersection/sim.sumocfg", "--no-step-log", "true", "--waiting-time-memory", str(max_steps)]
    
    sim_runner = SimRunner(traffic_gen, max_steps, green_duration, yellow_duration, sumoCmd)
    
    sim_runner.run()

    save_graphs(sim_runner,path)