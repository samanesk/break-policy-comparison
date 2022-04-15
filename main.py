import traceback
from models import *
from data_prep_functions import *
from datetime import timedelta
import time
import argparse


def cb(model, where):
    """
    Gurobi callback function, which terminates the optimization process if either of two stopping criteria
    1) at the current MIP Node there is no improvement in the objective since the last improvement in the objective
    2) there has been no improvement in the objective since the last improvement
    """
    if where == GRB.Callback.MIPNODE:
        # Get model objective
        obj = model.cbGet(GRB.Callback.MIPNODE_OBJBST)
        # Has objective changed?
        if abs(obj - model._cur_obj) > 1e-8:
            # If so, update incumbent and time
            model._cur_obj = obj
            model._time = time.time()
        if time.time() - model._time > OPTIMIZATION_STOP_TIME:
            # terminate if the best incumbent's objective has not improved in OPTIMIZATION_STOP_TIME seconds at the
            # current MIP node
            model.terminate()
    # terminate if the objective has not improved in OPTIMIZATION_STOP_TIME seconds
    if time.time() - model._time > OPTIMIZATION_STOP_TIME_MAX:
        model.terminate()


def create_duty_from_duty_name(jobs, duties, locations, distance_matrix, duty_name, ref_datetime, nm_map, time_horizon,
                               depot_index):
    duty_start = 0
    duty_end = 0
    duration = 0
    no_breaks = 0
    no_vehicles = 0
    try:
        (dic_jobs, pickup_delivery_pairs, planning_quantities, pickup_tasks, delivery_tasks, consecutive_visits_pairs,
         task_loc_mapping, task_loc_mapping_0, time_windows,
         planning_quantities, dic_distance_matrix, service_time_task, CAPACITY) = create_input(jobs, duties, locations,
                                                                                               distance_matrix,
                                                                                               duty_name,
                                                                                               ref_datetime, nm_map,
                                                                                               time_horizon,
                                                                                               depot_index)
        duty_start, duty_end, duration, no_breaks, no_vehicles = extract_original_duty_statistics(duties, duty_name, nm_map)
        if not verify_planning_quantities(pickup_delivery_pairs, planning_quantities):
            print("planning quantities are not consistent with pickup-delivery pairs")
        message = ''
        error = 'None'
    except Exception as e:
        logging.error(traceback.format_exc())
        message = traceback.format_exc()
        error = "not identified"
    return (
    dic_jobs, pickup_delivery_pairs, planning_quantities, pickup_tasks, delivery_tasks, consecutive_visits_pairs,
    task_loc_mapping, task_loc_mapping_0, time_windows,
    planning_quantities, dic_distance_matrix, service_time_task, CAPACITY, duty_start, duty_end, duration, no_breaks,
    no_vehicles, error, message)


def main_mip(duty_name, instance_name, policy, dic_compact_output, pickup_tasks, delivery_tasks, pickup_delivery_pairs,
             consecutive_visits_pairs, task_loc_mapping, time_windows, planning_quantities, dic_distance_matrix,
             service_time_task, CAPACITY, ref_datetime, error, message, duration, no_vehicles, nm_map, duty_start,
             duty_end):
    try:
        BIG_M = {'Time': TIME_HORIZON, 'Capacity': 90, 'Break': 200}  # [BIG_M time, BIG_M Capacity]
        print(
            f"*******************************************************Duty {duty_name}, Policy {policy}***********************************************************")
        model, var_consecutive_visits_pairs, var_service_start_time, last_task_index = generate_mip_model(policy, BIG_M,
                                                                                                          pickup_tasks,
                                                                                                          delivery_tasks,
                                                                                                          pickup_delivery_pairs,
                                                                                                          consecutive_visits_pairs,
                                                                                                          task_loc_mapping,
                                                                                                          time_windows,
                                                                                                          planning_quantities,
                                                                                                          dic_distance_matrix,
                                                                                                          service_time_task,
                                                                                                          CAPACITY)
        # Last updated objective and time
        model._cur_obj = float('inf')
        model._time = time.time()
        # the model terminates if conditions of callback function cb is satisfied
        model.optimize(callback=cb)
        error = 'None'
        first_model_status = model.getAttr("status")
        first_objective = model.getAttr("ObjVal")
        first_model_runtime = model.getAttr("Runtime")
        first_model_MIPGap = model.getAttr("MIPGap")
        if first_model_status in {2}:
            model.addConstr(var_service_start_time[last_task_index] - var_service_start_time[0] <= first_objective,
                            name="objective_bound")
            model.setObjective(quicksum(
                distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]] * var_consecutive_visits_pairs[i, j] for
                (i, j) in consecutive_visits_pairs), sense=GRB.MINIMIZE)
            model._cur_obj = float('inf')
            model._time = time.time()
            # the model terminates if conditions of callback function cb is satisfied
            model.optimize(callback=cb)
        message = ""
    except Exception as e:
        logging.error(traceback.format_exc())
        message = traceback.format_exc()
        error = "not identified"
    if error == "not identified":
        dic_compact_output["Duty Name"].append(duty_name)
        dic_compact_output["Instance Name"].append(instance_name)
        dic_compact_output["Duty Start"].append(ref_datetime.create_datetime())
        dic_compact_output["Duty End"].append(ref_datetime.create_datetime())
        dic_compact_output["Optimized Duty Start"].append(0)
        dic_compact_output["Optimized Duty End"].append(0)
        dic_compact_output["AP Duration"].append(0)
        dic_compact_output['Optimized Tour'].append(0)
        dic_compact_output["Total Break"].append(0)
        dic_compact_output["Breaks"].append(0)
        dic_compact_output["Total Idle Time"].append(0)
        dic_compact_output["Total Travel Time"].append(0)
        dic_compact_output["Total Service Time"].append(0)
        dic_compact_output["Sequence"].append(0)
        dic_compact_output["Idle Times"].append(0)
        dic_compact_output["Travel Times"].append(0)
        dic_compact_output["Start Times"].append(0)
        dic_compact_output["Time Windows"].append(0)
        dic_compact_output["Service Times"].append(0)
        dic_compact_output["Optimized Duration"].append(0)
        dic_compact_output["Truck Capacity"].append(0)
        dic_compact_output["AP No. Trucks"].append(0)
        dic_compact_output["Error"].append(error + message)
        dic_compact_output["Model Status"].append(0)
        dic_compact_output['Break Policy'].append(policy)
        dic_compact_output['No of Tasks'].append(0)
        dic_compact_output["MIP Gap"].append(0)
        dic_compact_output["Runtime"].append(0)
        dic_compact_output["Number of Variables"].append(0)
        dic_compact_output["Number of constraints"].append(0)
        dic_compact_output["Objective Value"].append(0)
    elif model.getAttr("SolCount") == 0:
        dic_compact_output["Duty Name"].append(duty_name)
        dic_compact_output["Instance Name"].append(instance_name)
        dic_compact_output["Duty Start"].append(ref_datetime.create_datetime())
        dic_compact_output["Duty End"].append(ref_datetime.create_datetime())
        dic_compact_output["Optimized Duty Start"].append(0)
        dic_compact_output["Optimized Duty End"].append(0)
        dic_compact_output["AP Duration"].append(duration)
        dic_compact_output['Optimized Tour'].append(0)
        dic_compact_output["Total Break"].append(0)
        dic_compact_output["Breaks"].append(0)
        dic_compact_output["Total Idle Time"].append(0)
        dic_compact_output["Total Travel Time"].append(0)
        dic_compact_output["Total Service Time"].append(0)
        dic_compact_output["Sequence"].append(0)
        dic_compact_output["Idle Times"].append(0)
        dic_compact_output["Travel Times"].append(0)
        dic_compact_output["Start Times"].append(0)
        dic_compact_output["Time Windows"].append(0)
        dic_compact_output["Service Times"].append(0)
        dic_compact_output["Optimized Duration"].append(0)
        dic_compact_output["Truck Capacity"].append(CAPACITY)
        dic_compact_output["AP No. Trucks"].append(no_vehicles)
        dic_compact_output["Error"].append(error)
        dic_compact_output["Model Status"].append(first_model_status)
        dic_compact_output['Break Policy'].append(policy)
        dic_compact_output['No of Tasks'].append(0)
        dic_compact_output["MIP Gap"].append(0)
        dic_compact_output["Runtime"].append(model.getAttr("Runtime"))
        dic_compact_output["Number of Variables"].append(model.getAttr("NumVars"))
        dic_compact_output["Number of constraints"].append(model.getAttr("NumConstrs"))
        dic_compact_output["Objective Value"].append(0)
    else:
        (sequence, optimized_start_times, breaks, total_break) = report(model)
        idle_times, total_idle_time, travel_times, total_travel_time, total_service_time = calculate_idle_travel_times(
            sequence, optimized_start_times, breaks, dic_distance_matrix, service_time_task, task_loc_mapping)
        optimized_duty_start_time, optimized_duty_end_time, optimized_duration = extract_optimized_duty_statistics(
            optimized_start_times, ref_datetime, service_time_task[len(optimized_start_times) - 1])
        optimized_tour = create_optimized_tour(optimized_start_times, travel_times, service_time_task, time_windows,
                                               breaks, idle_times, sequence)
        dic_compact_output["Duty Name"].append(duty_name)
        dic_compact_output["Instance Name"].append(instance_name)
        dic_compact_output["Duty Start"].append(duty_start)
        dic_compact_output["Duty End"].append(duty_end)
        dic_compact_output["Optimized Duty Start"].append(optimized_duty_start_time)
        dic_compact_output["Optimized Duty End"].append(optimized_duty_end_time)
        dic_compact_output["AP Duration"].append(duration)
        dic_compact_output['Optimized Tour'].append(optimized_tour)
        dic_compact_output["Total Break"].append(total_break)
        dic_compact_output["Breaks"].append(breaks)
        dic_compact_output["Total Idle Time"].append(total_idle_time)
        dic_compact_output["Total Travel Time"].append(total_travel_time)
        dic_compact_output["Total Service Time"].append(total_service_time)
        dic_compact_output["Sequence"].append(sequence)
        dic_compact_output["Idle Times"].append(idle_times)
        dic_compact_output["Travel Times"].append(travel_times)
        dic_compact_output["Start Times"].append(optimized_start_times)
        dic_compact_output["Time Windows"].append(time_windows)
        dic_compact_output["Service Times"].append(service_time_task)
        dic_compact_output["Optimized Duration"].append(optimized_duration)
        dic_compact_output["Truck Capacity"].append(CAPACITY)
        dic_compact_output["AP No. Trucks"].append(no_vehicles)
        dic_compact_output["Error"].append("None")
        dic_compact_output["Model Status"].append(first_model_status)
        dic_compact_output['Break Policy'].append(policy)
        dic_compact_output['No of Tasks'].append(len(sequence))
        dic_compact_output["MIP Gap"].append(first_model_MIPGap)
        dic_compact_output["Runtime"].append(first_model_runtime)
        dic_compact_output["Number of Variables"].append(model.getAttr("NumVars"))
        dic_compact_output["Number of constraints"].append(model.getAttr("NumConstrs"))
        dic_compact_output["Objective Value"].append(max(first_objective, 0))
        return dic_compact_output


if __name__ == '__main__':
    project_dir = os.getcwd()
    print(project_dir)
    # set the working directory to the current directory
    os.chdir(project_dir)
    # read command-line arguments if there are any
    parser = argparse.ArgumentParser(description='Input arguments')
    parser.add_argument(
        '--max_dt',
        type=int,
        default=MAXIMUM_DUTY_TIME,
        help='maximum duty time'
    )
    parser.add_argument(
        '--tl',
        type=int,
        default=TIME_LIMIT,
        help='time limit'
    )
    parser.add_argument(
        '--trds',
        type=int,
        default=THREADS,
        help='threads'
    )
    parser.add_argument(
        '--th',
        type=int,
        default=TIME_HORIZON,
        help='time horizon'
    )
    parser.add_argument(
        '--ost',
        type=int,
        default=OPTIMIZATION_STOP_TIME,
        help='optimization stop time'
    )
    parser.add_argument(
        '--ostm',
        type=int,
        default=OPTIMIZATION_STOP_TIME_MAX,
        help='optimization stop time max'
    )
    parser.add_argument(
        '--otname',
        default='summary',
        help='output name'
    )
    parser.add_argument(
        '--fd',
        type=int,
        default=0,
        help='first duty in the list'
    )
    parser.add_argument(
        '--ld',
        type=int,
        default=1,
        help='last duty in the list. -1 is the index of the second last item in a given list by default'
    )
    args = parser.parse_args()
    # set arguments values
    TIME_LIMIT = args.tl
    MAXIMUM_DUTY_TIME = args.max_dt
    THREADS = args.trds
    TIME_HORIZON = args.th
    OPTIMIZATION_STOP_TIME = args.ost
    OPTIMIZATION_STOP_TIME_MAX = args.ostm

    jobs = pd.read_excel(project_dir + "/Data/jobs.xlsx", sheet_name='Sheet1')
    duties = pd.read_excel(project_dir + "/Data/duties.xlsx",
                           sheet_name='Sheet1')
    locations = pd.read_excel(project_dir + "/Data/locations.xlsx")
    distance_matrix = pd.read_excel(project_dir + "/Data/duration_peak_matrix_full.xlsx")
    # map names used in codes to names used in input files.
    nm_map = {"Scheduled Start": "Scheduled Start", 'Earliest Pickup Start Time': 'Earliest Pickup Start Time',
              'Latest Pickup Start Time': 'Latest Pickup Start Time',
              'Earliest Delivery Start Time': 'Earliest Delivery Start Time',
              'Latest Delivery Start Time': 'Latest Delivery Start Time',
              'Pickup AP Duration': 'Pickup AP Duration', 'Delivery AP Duration': 'Delivery AP Duration',
              'Duty: Duty Template Name': 'Duty: Duty Template Name', 'Activity Type': "Activity Type",
              "Truck Capacity": "Truck Capacity", "Delivery Duration": "Delivery Planning Duration Sc3",
              'ID': "ID", "Street": "Street", "City": "City", "State": "State", "Delivery Street": "Delivery Street",
              "Delivery City": "Delivery City",
              "Delivery State": "Delivery State", "Delivery Name": "Delivery Name", "Unique_ID": "Unique_ID",
              "Planning Quantity": "Planning Quantity",
              "Pickup Duration": "Pickup Planning Duration Sc3",
              "Scheduled Pickup Start Time": "Scheduled Pickup Start Time",
              "Duty Name": "Duty Name", 'Work Order: Work Order Number': 'Work Order: Work Order Number',
              'Location: Location Name': 'Location: Location Name',
              'Delivery Workorder Number': 'Delivery Workorder Number', "Workorder": "Workorder",
              "Scheduled Delivery Start Time": 'Scheduled Delivery Start Time',
              'Delivery Location: Location Name': 'Delivery Location: Location Name'}
    duties['Scheduled Start'] = duties['Scheduled Start'].apply(pd.to_datetime)
    engine = 'xlsxwriter'
    if nm_map['Earliest Pickup Start Time'] not in jobs.columns:
        jobs[nm_map['Earliest Pickup Start Time']] = jobs.apply(
            lambda row: to_datetime(row[nm_map['Scheduled Pickup Start Time']]) - timedelta(minutes=5), axis=1)
        jobs[nm_map['Latest Pickup Start Time']] = jobs.apply(
            lambda row: to_datetime(row[nm_map['Scheduled Pickup Start Time']]) + timedelta(
                minutes=row[nm_map['Pickup AP Duration']] + 5), axis=1)
    if nm_map['Earliest Delivery Start Time'] not in jobs.columns:
        jobs[nm_map['Earliest Delivery Start Time']] = jobs[nm_map['Earliest Pickup Start Time']]
        jobs[nm_map['Latest Delivery Start Time']] = jobs.apply(
            lambda row: to_datetime(row[nm_map['Latest Pickup Start Time']]) + timedelta(minutes=300), axis=1)
    duty_names = ['duty1']
    duty_names = duty_names[args.fd:args.ld]
    print(duty_names)
    print('**********Start Parameters**************')
    print(args)
    solver = 'Gurobi'
    dic_compact_output = {'Truck Capacity': [], "AP No. Trucks": [], 'Duty Name': [], 'Instance Name': [],
                          "Time Window Offset": [], 'Error': [], 'Duty Start': [], 'Duty End': [], 'AP Duration': [],
                          'Optimized Tour': [], 'Total Break': [], "Total Idle Time": [], "Total Travel Time": [],
                          "Total Service Time": [], "Sequence": [], "Idle Times": [], "Travel Times": [],
                          "Start Times": [], "Time Windows": [], "Service Times": [], "Optimized Duty Start": [],
                          "Optimized Duty End": [], "Breaks": [], 'Optimized Duration': [], 'Break Policy': [],
                          'Solver': [], "Model Status": [], "No of Tasks": [], "MIP Gap": [], "Runtime": [],
                          "Number of Variables": [], "Number of constraints": [], "Objective Value": []}
    ref_datetime = RefDate(2019, 9, 24)
    time_horizon = TIME_HORIZON
    depot_index = 0  # index of depot in the locations file (an excel file)
    for duty_name in duty_names:
        (dic_jobs, pickup_delivery_pairs, planning_quantities, pickup_tasks, delivery_tasks,
         consecutive_visits_pairs,
         task_loc_mapping, task_loc_mapping_0, time_windows,
         planning_quantities, dic_distance_matrix, service_time_task, CAPACITY, duty_start, duty_end, duration,
         no_breaks,
         no_vehicles, error, message) = create_duty_from_duty_name(jobs, duties, locations, distance_matrix,
                                                                   duty_name, ref_datetime, nm_map,
                                                                   time_horizon,
                                                                   depot_index)
        # corresponding to each time_offset and duty, one instance is created
        #  the time windows for each pickup task in each instance is created based on a time_offset. The unit of
        #  time_offset is minute
        for time_offset in {0, 15, 30, 45, 60, 90, 120}:
            instance_name = duty_name + "_" + str(time_offset)
            instance_time_windows = time_windows.copy()
            for i in pickup_tasks:
                instance_time_windows[i] = (
                    instance_time_windows[i][0] - time_offset, instance_time_windows[i][1] + time_offset)
            if solver == "Gurobi":
                main_mip(duty_name, instance_name, "no_break", dic_compact_output, pickup_tasks, delivery_tasks,
                         pickup_delivery_pairs, consecutive_visits_pairs, task_loc_mapping, instance_time_windows,
                         planning_quantities, dic_distance_matrix, service_time_task, CAPACITY, ref_datetime, error,
                         message, duration, no_vehicles, nm_map, duty_start, duty_end)
                dic_compact_output['Solver'].append("MIP")
                dic_compact_output['Time Window Offset'].append(time_offset)
                main_mip(duty_name, instance_name, "free_break", dic_compact_output, pickup_tasks, delivery_tasks,
                         pickup_delivery_pairs, consecutive_visits_pairs, task_loc_mapping, instance_time_windows,
                         planning_quantities, dic_distance_matrix, service_time_task, CAPACITY, ref_datetime, error,
                         message, duration, no_vehicles, nm_map, duty_start, duty_end)
                dic_compact_output['Solver'].append("MIP")
                dic_compact_output['Time Window Offset'].append(time_offset)
                main_mip(duty_name, instance_name, "break_in_depot", dic_compact_output, pickup_tasks,
                         delivery_tasks,
                         pickup_delivery_pairs, consecutive_visits_pairs, task_loc_mapping, instance_time_windows,
                         planning_quantities, dic_distance_matrix, service_time_task, CAPACITY, ref_datetime, error,
                         message, duration, no_vehicles, nm_map, duty_start, duty_end)
                dic_compact_output['Solver'].append("MIP")
                dic_compact_output['Time Window Offset'].append(time_offset)
                main_mip(duty_name, instance_name, "30_min_block_break_in_depot", dic_compact_output, pickup_tasks,
                         delivery_tasks, pickup_delivery_pairs,
                         consecutive_visits_pairs, task_loc_mapping, instance_time_windows, planning_quantities,
                         dic_distance_matrix, service_time_task, CAPACITY, ref_datetime, error, message, duration,
                         no_vehicles, nm_map, duty_start, duty_end)
                dic_compact_output['Solver'].append("MIP")
                dic_compact_output['Time Window Offset'].append(time_offset)
                main_mip(duty_name, instance_name, "30_min_block_free_break", dic_compact_output, pickup_tasks,
                         delivery_tasks, pickup_delivery_pairs,
                         consecutive_visits_pairs, task_loc_mapping, instance_time_windows, planning_quantities,
                         dic_distance_matrix, service_time_task, CAPACITY, ref_datetime, error, message, duration,
                         no_vehicles, nm_map, duty_start, duty_end)
                dic_compact_output['Solver'].append("MIP")
                dic_compact_output['Time Window Offset'].append(time_offset)
            else:
                print("solver does not available")
    # write the solution to an excel file
    path_filename = "/Output/" + args.otname + ".xlsx"
    pd.DataFrame(dic_compact_output).to_excel(project_dir + path_filename.format(engine),
                                              engine=engine)

