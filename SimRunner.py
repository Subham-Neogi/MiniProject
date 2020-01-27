import traci
import numpy as np
import random


# phase codes based on tlcs.net.xml
PHASE_NC_GREEN = 0  # action 0 code 00
PHASE_NC_YELLOW = 1
PHASE_EC_GREEN = 2  # action 1 code 01
PHASE_EC_YELLOW = 3
PHASE_SC_GREEN = 4  # action 2 code 10
PHASE_SC_YELLOW = 5
PHASE_WC_GREEN = 6  # action 3 code 11
PHASE_WC_YELLOW = 7

# HANDLE THE SIMULATION OF THE AGENT
class SimRunner:
    def __init__(self,traffic_gen, max_steps, green_duration, yellow_duration, sumoCmd):
        self._traffic_gen = traffic_gen
        self._steps = 0
        self._waiting_times = {}
        self._sumoCmd = sumoCmd
        self._max_steps = max_steps
        self._green_duration = green_duration
        self._yellow_duration = yellow_duration
        self._sum_intersection_queue = 0
        self._cumulative_wait_store = {}
        self._avg_intersection_queue_store = []
        self._mu_store = [0]
        self._sigma_store = [0]
        self._cycle_duration = []
        self._total_wait_time_store = {}
        self._c = 0.5
        self._action_order = [0,1,2,3]
        self._k = 0
        self._green_duration_store = []

    # THE MAIN FUCNTION WHERE THE SIMULATION HAPPENS
    def run(self):
        # first, generate the route file for this simulation and set up sumo
        self._traffic_gen.generate_routefile(1234)
        traci.start(self._sumoCmd)

        # inits
        self._steps = 0
        old_action = 3
        self._waiting_times = {}
        self._sum_intersection_queue = 0
        action = 0
        current_total_wait=0
        current_total_wait_road=[0,0,0,0]
        cycle_start=0

        while self._steps < self._max_steps:

            # waiting time = seconds waited by a car since the spawn in the environment, cumulated for every car in incoming lanes
            current_total_wait_road[0] += self._get_waiting_times(["NtoC"])
            current_total_wait_road[1] += self._get_waiting_times(["EtoC"])
            current_total_wait_road[2] += self._get_waiting_times(["StoC"])
            current_total_wait_road[3] += self._get_waiting_times(["WtoC"])
            current_total_wait += sum(current_total_wait_road)
            self._cumulative_wait_store[self._steps]=self._get_waiting_times()

            # if the chosen phase is different from the last phase, activate the yellow phase
            if self._steps != 0 and old_action != action:
                self._set_yellow_phase(old_action)
                self._simulate(self._yellow_duration)

            # execute the phase selected before
            self._set_green_phase(self._action_order[action])
            self._simulate(self._green_duration[self._action_order[action]])

            # saving variables for later 
            old_action = self._action_order[action]
            action += 1
            if action == 4:
                action=0
                self._k+=1
                self._total_wait_time_store[self._k]=self._mu_store[self._k-1]*(self._k-1)+current_total_wait
                self._mu_store.append(self._total_wait_time_store[self._k]/self._k)
                vk=current_total_wait-self._mu_store[self._k]
                self._sigma_store.append(np.sqrt((vk*vk)/self._k))
                if self._k!=1 : 
                    x=self._mu_store[self._k-1]+self._c*self._sigma_store[self._k-1]
                    if current_total_wait > x :
                        self._recompute_green_duration(current_total_wait,current_total_wait_road)
                        
                current_total_wait=0
                current_total_wait_road[0]=0
                current_total_wait_road[1]=0
                current_total_wait_road[2]=0
                current_total_wait_road[3]=0

                self._cycle_duration.append(self._steps-cycle_start)
                cycle_start=self._steps

        traci.close()

    # HANDLE THE CORRECT NUMBER OF STEPS TO SIMULATE
    def _simulate(self, steps_todo):
        if (self._steps + steps_todo) >= self._max_steps:  # do not do more steps than the maximum number of steps
            steps_todo = self._max_steps - self._steps
        self._steps = self._steps + steps_todo  # update the step counter
        while steps_todo > 0:
            traci.simulationStep()  # simulate 1 step in sumo
            steps_todo -= 1
            intersection_queue = self._get_stats()
            self._sum_intersection_queue += intersection_queue
            self._avg_intersection_queue_store.append(intersection_queue)


    # RETRIEVE THE WAITING TIME OF EVERY CAR IN THE INCOMING LANES
    def _get_waiting_times(self,road="all"):
        if road=="all":
            incoming_roads = ["EtoC", "NtoC", "WtoC", "StoC"]
        else:
            incoming_roads = road
        for veh_id in traci.vehicle.getIDList():
            wait_time_car = traci.vehicle.getAccumulatedWaitingTime(veh_id)
            road_id = traci.vehicle.getRoadID(veh_id)  # get the road id where the car is located
            if road_id in incoming_roads:  # consider only the waiting times of cars in incoming roads
                self._waiting_times[veh_id] = wait_time_car
            else:
                if veh_id in self._waiting_times:
                    del self._waiting_times[veh_id]  # the car isnt in incoming roads anymore, delete his waiting time
        total_waiting_time = sum(self._waiting_times.values())
        return total_waiting_time

    # SET IN SUMO THE CORRECT YELLOW PHASE
    def _set_yellow_phase(self, old_action):
        yellow_phase = old_action * 2 + 1 # obtain the yellow phase code, based on the old action
        traci.trafficlight.setPhase("C", yellow_phase)

    # SET IN SUMO A GREEN PHASE
    def _set_green_phase(self, action_number):
        if action_number == 0:
            traci.trafficlight.setPhase("C", PHASE_NC_GREEN)
        elif action_number == 1:
            traci.trafficlight.setPhase("C", PHASE_EC_GREEN)
        elif action_number == 2:
            traci.trafficlight.setPhase("C", PHASE_SC_GREEN)
        elif action_number == 3:
            traci.trafficlight.setPhase("C", PHASE_WC_GREEN)

    # RETRIEVE THE STATS OF THE SIMULATION FOR ONE SINGLE STEP
    def _get_stats(self):
        halt_N = traci.edge.getLastStepHaltingNumber("NtoC")
        halt_S = traci.edge.getLastStepHaltingNumber("StoC")
        halt_E = traci.edge.getLastStepHaltingNumber("EtoC")
        halt_W = traci.edge.getLastStepHaltingNumber("WtoC")
        intersection_queue = halt_N + halt_S + halt_E + halt_W
        return intersection_queue

    def _recompute_green_duration(self,total_wait,total_wait_road):
        self._green_duration_store.append(self._green_duration.copy())
        for i in range(4):
            a=(self._total_wait_time_store[self._k]-self._total_wait_time_store[self._k-1])
            b=(total_wait_road[i]/total_wait)
            self._green_duration[i]=self._green_duration[i]+a*b
            #print(i," ",self._k," ",self._green_duration[i]," ",a," ",b," ",total_wait_road[i])
        self._action_order = sorted(self._action_order,key= lambda i : self._green_duration[i])
        #print(self._action_order," ",self._green_duration," ",self._green_duration_store)

    @property
    def cumulative_wait_store(self):
        return self._cumulative_wait_store

    @property
    def avg_intersection_queue_store(self):
        return self._avg_intersection_queue_store

    @property
    def total_wait_time_store(self):
        return self._total_wait_time_store

    @property
    def green_duration_store(self):
        return self._green_duration_store

    @property
    def mu_store(self):
        return self._mu_store
    
    @property
    def sigma_store(self):
        return self._sigma_store
    
    @property
    def cycle_duration(self):
        return self._cycle_duration
        