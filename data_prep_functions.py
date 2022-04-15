import pandas as pd
from datetime import datetime, timedelta
from gurobipy import *
from math import ceil


class RefDate:
    def __init__(self,year,month,day):
        self.year = year
        self.month = month
        self.day = day

    def create_datetime(self):
        return datetime(self.year, self.month, self.day)


def create_input(jobs: pd.DataFrame, duties: pd.DataFrame, locations: pd.DataFrame, distance_matrix: pd.DataFrame,
                 duty_name: str, ref_datetime: RefDate, nm_map, time_horizon: int = 2880, depot_index=0):
    """
    create input for policy comparision analysis. We assume jobs include time windows in datetime format.
    return a dictionary of jobs for a single duty
    """
    dic_jobs = jobs[jobs[nm_map['Duty Name']] == duty_name].to_dict()
    # change the keys of jobs to [0,...,n_jobs-1]
    dic_jobs = {col: {r: dic_jobs[col][j] for r, j in enumerate(dic_jobs[col])} for col in dic_jobs}
    n_jobs = len(dic_jobs[nm_map['Unique_ID']])
    pickup_tasks = [2 * i - 1 for i in range(1, n_jobs + 1)]
    delivery_tasks = [2 * i for i in range(1, n_jobs + 1)]
    pickup_delivery_pairs = [(2 * i - 1, 2 * i) for i in range(1, n_jobs + 1)]
    dic_loc = locations.to_dict()
    excel_locid_to_locid = {row[nm_map['Street']] + row[nm_map['City']] + row[nm_map['State']]: i
                      for i, row in locations.iterrows()}
    print(excel_locid_to_locid)
    pickup_delivery_tasks = pickup_tasks + delivery_tasks
    last_task_index = len(pickup_delivery_tasks) + 1
    task_loc_mapping_0 = {}
    task_loc_mapping_0[0] = depot_index  # 33 is the index of depot
    task_loc_mapping_0[last_task_index] = depot_index  # 33 is the index of depot
    time_windows_without_depot= [(0,time_horizon)]
    # initialize list of planning quantities with depot planning quantity
    planning_quantities = [0]
    # initialize list of service times with depot service time
    service_times = [16]
    # reference time for calculating integral offset time in minute for given times including start time,
    # earliest start time, etc.
    ref_time = datetime(ref_datetime.year, ref_datetime.month, ref_datetime.day, 0, 0)
    for job_idx in range(0, n_jobs):
        # Add pickup location ID
        task_loc_mapping_0[2 * job_idx + 1] = excel_locid_to_locid[
            dic_jobs[nm_map['Street']][job_idx] + dic_jobs[nm_map['City']][job_idx]
            + dic_jobs[nm_map['State']][job_idx]]
        # Add delivery location ID
        task_loc_mapping_0[2 * job_idx + 2] = excel_locid_to_locid[
            dic_jobs[nm_map['Delivery Street']][job_idx] + dic_jobs[nm_map['Delivery City']][job_idx]
            + dic_jobs[nm_map['Delivery State']][job_idx]]
        # Add pickup quantity
        planning_quantities.append(dic_jobs[nm_map['Planning Quantity']][job_idx])
        # Add delivery quantity which is equal to minus pickup quantity
        planning_quantities.append(-dic_jobs['Planning Quantity'][job_idx])
        # Add scheduled start time for pickup by AP planners
        earliest_pickup_start_time = (int(
            (dic_jobs[nm_map['Earliest Pickup Start Time']][job_idx].to_pydatetime() - ref_time).total_seconds() / 60))
        latest_pickup_start_time = (int(
            (dic_jobs[nm_map['Latest Pickup Start Time']][job_idx].to_pydatetime() - ref_time).total_seconds() / 60))
        time_windows_without_depot.append((earliest_pickup_start_time,latest_pickup_start_time))
        # Add scheduled start time for delivery by AP planners
        earliest_delivery_start_time = (int(
            (dic_jobs[nm_map['Earliest Delivery Start Time']][job_idx].to_pydatetime()
             - ref_time).total_seconds() / 60))
        latest_delivery_start_time = (int(
            (dic_jobs[nm_map['Latest Delivery Start Time']][job_idx].to_pydatetime() - ref_time).total_seconds() / 60))
        time_windows_without_depot.append((earliest_delivery_start_time, latest_delivery_start_time))
        # Add pickup task duration
        service_times.append(ceil(int(dic_jobs[nm_map['Pickup Duration']][job_idx])))
        # Add delivery task duration
        service_times.append(ceil(int(dic_jobs[nm_map['Delivery Duration']][job_idx])))
    # Add planning quantity of last node which is depot
    planning_quantities.append(0)
    # add time windows for return to depot
    time_windows_without_depot.append((0, time_horizon))
    # add duration of last task (i.e., return to depot)
    service_times.append(5)
    # get all location IDs and then sort them in ascending order
    locs = sorted(set([task_loc_mapping_0[i] for i in task_loc_mapping_0]))
    # distance matrix in the input is in seconds
    dic_distance_matrix = [[int(distance_matrix.loc[i, j] / 60) for j in locs] for i in locs]
    task_loc_mapping = {i: locs.index(task_loc_mapping_0[i]) for i in task_loc_mapping_0}
    time_windows = time_windows_without_depot
    CAPACITY = max(dic_jobs[nm_map['Truck Capacity']].values())
    consecutive_visits_pairs = [(i, j) for i in pickup_delivery_tasks for j in pickup_delivery_tasks if i != j] + [
        (0, i) for i in pickup_tasks] + [(j, last_task_index) for j in delivery_tasks]
    return (dic_jobs,pickup_delivery_pairs, planning_quantities, pickup_tasks, delivery_tasks,consecutive_visits_pairs,
            task_loc_mapping,task_loc_mapping_0, time_windows,
    planning_quantities, dic_distance_matrix, service_times, CAPACITY)


def output_to_dic(breaks, duties, locations, dic_jobs: dict, sequence, optimized_start_times: dict, service_time_task,
                  distance_matrix, task_loc_mapping, task_loc_mapping_0, ref_datetime: datetime, nm_map, time_windows):
    """

    """
    dic_output = {'Unique ID': [], 'Task ID': [], 'Workorder': [], 'Task Type': [], 'Optimized Start': [],
                  "Earliest Start Time": [], "Latest Finish Time": [], 'Optimized Start(m)': [],
                  "Earliest Start Time(m)": [], "Latest Finish Time(m)": [], 'AP Planned Start Time': [],
                  'Planning Quantity': [], 'Planning Duration': [], 'AP Duration': [],'Distance to Next': [],
                  'Location Name': []}
    no_tasks = 2*len(dic_jobs[nm_map['Unique_ID']]) + 2
    for idx, task_id in enumerate(sequence):
        id = int(task_id)
        job_id, even = taskID_to_jobID(id)
        dic_output['Task ID'].append(id)
        if id in {0, no_tasks-1}:
            # first and last task do not have Unique_ID info in the dic_jobs because they are not the input files
            dic_output['Unique ID'].append(0)
            dic_output['Workorder'].append(0)
            dic_output['AP Duration'].append(0)
            dic_output['Task Type'].append('')
            dic_output['Location Name'].append('STF')
            dic_output['AP Planned Start Time'].append(datetime(2019,9, 24))
            dic_output['Planning Quantity'].append(0)
        else:
            dic_output['Unique ID'].append(dic_jobs[nm_map['Unique_ID']][job_id])
            if even == False:
                # this is a pickup task
                dic_output['Workorder'].append(dic_jobs[nm_map['Work Order: Work Order Number']][job_id])
                dic_output['AP Duration'].append(dic_jobs[nm_map['Pickup AP Duration']][job_id])
                dic_output['Task Type'].append('Pickup')
                dic_output['Location Name'].append(dic_jobs[nm_map['Location: Location Name']][job_id])
                dic_output['AP Planned Start Time'].append(dic_jobs[nm_map['Scheduled Pickup Start Time']][job_id])
                dic_output['Planning Quantity'].append(dic_jobs[nm_map['Planning Quantity']][job_id])
            else:
                # this is a delivery task
                dic_output['Workorder'].append(dic_jobs[nm_map['Delivery Workorder Number']][job_id])
                dic_output['AP Duration'].append(dic_jobs[nm_map['Delivery AP Duration']][job_id])
                dic_output['Task Type'].append('Delivery')
                dic_output['Location Name'].append(dic_jobs[nm_map['Delivery Location: Location Name']][job_id])
                dic_output['AP Planned Start Time'].append(dic_jobs[nm_map['Scheduled Delivery Start Time']][job_id])
                dic_output['Planning Quantity'].append(-dic_jobs[nm_map['Planning Quantity']][job_id])
        dic_output['Optimized Start'].append(ref_datetime + timedelta(minutes=optimized_start_times[task_id]))
        dic_output['Earliest Start Time'].append(ref_datetime + timedelta(minutes=time_windows[id][0]))
        dic_output['Latest Finish Time'].append(ref_datetime + timedelta(minutes=time_windows[id][1]))
        dic_output['Optimized Start(m)'].append(optimized_start_times[task_id])
        dic_output['Earliest Start Time(m)'].append(time_windows[id][0])
        dic_output['Latest Finish Time(m)'].append(time_windows[id][1])
        dic_output['Planning Duration'].append(service_time_task[int(task_id)])
        if int(sequence[idx]) < (no_tasks-1):
            dic_output['Distance to Next'].append(
                distance_matrix[task_loc_mapping[int(task_id)]][task_loc_mapping[int(sequence[idx + 1])]])
        else:
            dic_output['Distance to Next'].append(0)
        if str(task_id) in breaks.keys():
            dic_output['Task ID'].append(0)
            dic_output['Unique ID'].append(0)
            dic_output['Workorder'].append(0)
            dic_output['AP Duration'].append(0)
            dic_output['Task Type'].append('Rest')
            dic_output['Location Name'].append("")
            dic_output['AP Planned Start Time'].append(0)
            dic_output['Planning Quantity'].append(0)
            dic_output['Optimized Start'].append(0)
            dic_output['Planning Duration'].append(breaks[str(task_id)])
            dic_output['Distance to Next'].append(0)
            dic_output['Earliest Start Time'].append(0)
            dic_output['Latest Finish Time'].append(0)
            dic_output['Optimized Start(m)'].append(0)
            dic_output['Earliest Start Time(m)'].append(0)
            dic_output['Latest Finish Time(m)'].append(0)
    return dic_output


def taskID_to_jobID(taskID):
  if 2*int(taskID/2) == taskID:
    job_id = int(taskID/2) - 1
    even = True
  else:
    job_id = int(taskID/2)
    even = False
  return job_id, even


def create_optimized_tour(optimized_start_times,travel_times,service_time_task,time_windows,breaks,idle_times,sequence):
    optimized_tour = {}
    for idx, task in enumerate(sequence[:-1]):
        optimized_tour[task] = {'Start Time': 0, 'Travel Time': 0, 'Service Time': 0, 'Time Windows': 0, 'Break': 0,
                                'Idle Time': 0}
        optimized_tour[task]['Start Time'] = optimized_start_times[task]
        optimized_tour[task]['Travel Time'] = travel_times[int(task)]
        optimized_tour[task]['Service Time'] = service_time_task[idx]
        optimized_tour[task]['Time Windows'] = time_windows[int(task)]
        optimized_tour[task]['Idle Time'] = idle_times[int(task)]
    for task in breaks:
        optimized_tour[task]['Break'] = breaks[task]
    return optimized_tour


def extract_original_duty_statistics(duties: pd.DataFrame, duty_name, nm_map):
    dic_duty = duties[duties[nm_map['Duty: Duty Template Name']] == duty_name].to_dict('list')
    try:
        index_of_first_prepare_vehicle_task = min(
            [i for i, x in enumerate(dic_duty[nm_map['Activity Type']]) if x == "Prepare Vehicle"])
        index_of_last_return_vehicle_task = max([i for i, x in enumerate(dic_duty[nm_map['Activity Type']]) if
                                               (x == "Return & Refuel Vehicle") or (x == "Return Vehicle")])
        duty_start = dic_duty[nm_map['Scheduled Start']][index_of_first_prepare_vehicle_task]
        duty_end = dic_duty[nm_map['Scheduled Start']][index_of_last_return_vehicle_task]
    except ValueError:
        duty_start = datetime(1900, 1, 1)
        duty_end = datetime(1900, 1, 1)
    duration = int((duty_end - duty_start).total_seconds()/60 + 6)
    no_breaks = dic_duty[nm_map['Activity Type']].count('Rest Break')
    no_vehicles = dic_duty[nm_map['Activity Type']].count('Prepare Vehicle')
    return duty_start, duty_end, duration, no_breaks, no_vehicles


def extract_optimized_duty_statistics(optimized_start_times, ref_datetime: RefDate, last_task_duration):
    """
    """
    duty_start_time_in_minutes = optimized_start_times['0']
    duty_end_time_in_minutes = max([value for key, value in optimized_start_times.items()]) + last_task_duration
    duty_start_time = ref_datetime.create_datetime() + timedelta(minutes=duty_start_time_in_minutes)
    duty_end_time = ref_datetime.create_datetime() + timedelta(minutes=duty_end_time_in_minutes)
    duration = duty_end_time_in_minutes - duty_start_time_in_minutes
    return duty_start_time, duty_end_time, duration


def create_sequence_from_consecutive_pairs(consecutive_pairs):
    """
    create a sequence of tasks from selected pairs in the solution
    """
    n = len(consecutive_pairs)
    sequence = []
    current_task_index = '0'
    for i in range(1, n+1):
        sequence.append(current_task_index)
        current_task_index = consecutive_pairs[current_task_index]
    sequence.append(current_task_index)
    return sequence


def report(model:Model):
    start_times = {}
    consecutive_pairs = {}
    breaks  = {}
    total_break = 0
    print("Model Status:", model.getAttr("status"))
    for v in model.getVars():
        name_split = v.Varname.split('[')
        if v.X >= 0.01 and name_split[0] == 'var_consecutive_visits_pairs':
            first_task_idx = name_split[1].split(']')[0].split(',')[0]
            second_task_idx = name_split[1].split(']')[0].split(',')[1]
            consecutive_pairs[first_task_idx] = second_task_idx
        elif name_split[0] == 'var_service_start_time':
            idx = v.Varname.split('[')[1].split(']')[0]
            start_times[idx] = v.X
        elif name_split[0] == 'var_available_capacity':
            c = 0
        elif name_split[0][0:-3] == 'var_break_between_consec_tasks' and v.X >= 0.001:
            first_task_idx = name_split[1].split(']')[0].split(',')[0]
            breaks[first_task_idx] = int(name_split[0][-2:])
    sequence = create_sequence_from_consecutive_pairs(consecutive_pairs)
    return sequence, start_times, breaks, total_break


def calculate_idle_travel_times(sequence:list, start_times:dict, breaks:dict, distance_matrix:dict,
                                service_time_task:list, task_loc_mapping):
    """
    calculate waiting times between each task to its next task
    """
    idle_times = {}
    travel_times = {}
    total_idle_time = 0
    total_travel_time = 0
    total_service_time = 0
    for idx, task in enumerate(sequence[:-1]):
        next_task = sequence[idx+1]
        travel_times[int(task)] = distance_matrix[task_loc_mapping[int(task)]][task_loc_mapping[int(next_task)]]
        idle_times[int(task)] = start_times[str(next_task)] - travel_times[int(task)] - start_times[str(task)] \
                                - service_time_task[int(task)]
        total_idle_time += idle_times[int(task)]
        total_travel_time += travel_times[int(task)]
        total_service_time += service_time_task[int(task)]
    return idle_times, total_idle_time, travel_times, total_travel_time, total_service_time


def verify_planning_quantities(pickup_delivery_pairs, planning_quantities):
    for (i,j) in pickup_delivery_pairs:
        if planning_quantities[j] == - planning_quantities[i]:
            return True
        else:
            return False


def to_datetime(value):
    """
    return datetime. if value is a datetime it returns the value
    """
    try:
        result = value.to_pydatetime()
    except AttributeError:
        result = value
    return result





