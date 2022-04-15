from gurobipy import *
from model_params import *


def generate_mip_model(policy, BIG_M, pickup_tasks, delivery_tasks, pickup_delivery_pairs,
                       consecutive_visits_pairs, task_loc_mapping, time_windows, planning_quantities,
                       distance_matrix: dict, service_time_task, CAPACITY, service_time_loc=[]):
    """
    """
    model = Model("Break Policy Comparison")
    model.Params.timeLimit = TIME_LIMIT
    model.Params.MIPGap = MIPGap
    # index sets
    tasks = pickup_tasks + delivery_tasks
    last_task_index = len(tasks) + 1  # last task is visiting depot
    # indices of all tasks including first task, that is, vehicle prep at start depot and last task vehicle return
    # at start depot
    tasks_indices = set(tasks + [0, last_task_index])
    index_pairs = [(i, j) for i in set(tasks + [0]) for j in set(tasks + [last_task_index]) if i != j]
    # Variables
    var_consecutive_visits_pairs = model.addVars(consecutive_visits_pairs, vtype=GRB.BINARY,
                                                 name="var_consecutive_visits_pairs")
    var_service_start_time = model.addVars(tasks_indices, vtype=GRB.INTEGER, name="var_service_start_time")
    var_available_capacity = model.addVars(tasks_indices, vtype=GRB.INTEGER, name="var_available_capacity")
    var_break_between_tasks_15 = model.addVars(index_pairs, vtype=GRB.BINARY, name="var_break_between_tasks_15")
    var_break_between_tasks_30 = model.addVars(index_pairs, vtype=GRB.BINARY,
                                               name="var_break_between_tasks_30")
    var_break_between_tasks_45 = model.addVars(index_pairs, vtype=GRB.BINARY,
                                               name="var_break_between_tasks_45")
    var_break_between_tasks_60 = model.addVars(index_pairs, vtype=GRB.BINARY,
                                               name="var_break_between_tasks_60")
    var_break_between_consec_tasks_15 = model.addVars(consecutive_visits_pairs, vtype=GRB.BINARY,
                                                      name="var_break_between_consec_tasks_15")
    var_break_between_consec_tasks_30 = model.addVars(consecutive_visits_pairs, vtype=GRB.BINARY,
                                                      name="var_break_between_consec_tasks_30")
    var_break_between_consec_tasks_45 = model.addVars(consecutive_visits_pairs, vtype=GRB.BINARY,
                                                      name="var_break_between_consec_tasks_45")
    var_break_between_consec_tasks_60 = model.addVars(consecutive_visits_pairs, vtype=GRB.BINARY,
                                                      name="var_break_between_consec_tasks_60")
    var_accumulated_break_time = model.addVars(tasks_indices, vtype=GRB.INTEGER, name="var_accumulated_break_time")

    # constraints
    # service constraints
    model.addConstrs(var_consecutive_visits_pairs.sum(i, '*') == 1 for i in tasks + [0])
    model.addConstrs(var_consecutive_visits_pairs.sum('*', j) == 1 for j in tasks + [last_task_index])

    # precedence constraints
    if policy in {'free_break', 'no_break'}:
        model = create_precedence_constraints_free_break_policy(model, var_service_start_time,
                                                                var_break_between_consec_tasks_15,
                                                                var_break_between_consec_tasks_30,
                                                                var_break_between_consec_tasks_45,
                                                                var_break_between_consec_tasks_60,
                                                                service_time_task, distance_matrix,
                                                                task_loc_mapping, var_consecutive_visits_pairs,
                                                                consecutive_visits_pairs, pickup_delivery_pairs, BIG_M)
    elif policy == 'break_in_depot':
        model = create_precedence_constraints_break_in_depot_policy(model, var_service_start_time,
                                                                    var_break_between_consec_tasks_15,
                                                                    var_break_between_consec_tasks_30,
                                                                    var_break_between_consec_tasks_45,
                                                                    var_break_between_consec_tasks_60,
                                                                    service_time_task,
                                                                    distance_matrix,
                                                                    task_loc_mapping,
                                                                    var_consecutive_visits_pairs,
                                                                    consecutive_visits_pairs,
                                                                    pickup_delivery_pairs,
                                                                    BIG_M)
    elif policy == '30_min_block_break_in_depot':
        model = create_precedence_constraints_30_min_block_break_in_depot_policy(model, var_service_start_time,
                                                                                 var_break_between_consec_tasks_30,
                                                                                 var_break_between_consec_tasks_60,
                                                                                 service_time_task,
                                                                                 distance_matrix,
                                                                                 task_loc_mapping,
                                                                                 var_consecutive_visits_pairs,
                                                                                 consecutive_visits_pairs,
                                                                                 pickup_delivery_pairs,
                                                                                 BIG_M)
    elif policy == '30_min_block_free_break':
        model = create_precedence_constraints_30_min_block_free_break_policy(model, var_service_start_time,
                                                                             var_break_between_consec_tasks_30,
                                                                             var_break_between_consec_tasks_60,
                                                                             service_time_task,
                                                                             distance_matrix,
                                                                             task_loc_mapping,
                                                                             var_consecutive_visits_pairs,
                                                                             consecutive_visits_pairs,
                                                                             pickup_delivery_pairs,
                                                                             BIG_M)

    model.addConstr(var_service_start_time[last_task_index] + service_time_task[last_task_index]
                    - var_service_start_time[0] <= MAXIMUM_DUTY_TIME)
    model.addConstrs(var_break_between_consec_tasks_15[i, j] <= var_consecutive_visits_pairs[i, j] for (i, j)
                     in set(consecutive_visits_pairs))
    model.addConstrs(var_break_between_consec_tasks_30[i, j] <= var_consecutive_visits_pairs[i, j] for (i, j)
                     in set(consecutive_visits_pairs))
    model.addConstrs(var_break_between_consec_tasks_45[i, j] <= var_consecutive_visits_pairs[i, j] for (i, j)
                     in set(consecutive_visits_pairs))
    model.addConstrs(var_break_between_consec_tasks_60[i, j] <= var_consecutive_visits_pairs[i, j] for (i, j)
                     in set(consecutive_visits_pairs))

    # time windows
    model.addConstrs(var_service_start_time[i] <= time_windows[i][1] for i in range(last_task_index + 1))
    model.addConstrs(time_windows[i][0] <= var_service_start_time[i] for i in range(last_task_index + 1))

    # capacity constraints
    model.addConstr(var_available_capacity[0] == CAPACITY)
    model.addConstrs(var_available_capacity[j]
                     <=
                     var_available_capacity[i] - planning_quantities[i]
                     + BIG_M["Capacity"] * (1 - var_consecutive_visits_pairs[i, j])
                     for (i, j) in consecutive_visits_pairs)

    # break constraints
    if policy in {'free_break', 'break_in_depot', 'no_break'}:
        model.addConstrs(var_accumulated_break_time[j] <= var_accumulated_break_time[i]
                         + 15 * var_break_between_consec_tasks_15[i, j]
                         + 30 * var_break_between_consec_tasks_30[i, j]
                         + 45 * var_break_between_consec_tasks_45[i, j]
                         + 60 * var_break_between_consec_tasks_60[i, j]
                         + BIG_M['Break'] * (1 - var_consecutive_visits_pairs[i, j])
                         for (i, j) in consecutive_visits_pairs if task_loc_mapping[i] != task_loc_mapping[j])
        model.addConstrs(var_accumulated_break_time[j] <= var_accumulated_break_time[i]
                         + BIG_M['Break'] * (1 - var_consecutive_visits_pairs[i, j]) for
                         (i, j) in consecutive_visits_pairs if task_loc_mapping[i]
                         ==
                         task_loc_mapping[j])
    elif policy in {'30_min_block_break_in_depot', '30_min_block_free_break'}:
        model.addConstrs(var_accumulated_break_time[j] <= var_accumulated_break_time[i]
                         + 30 * var_break_between_consec_tasks_30[i, j]
                         + 60 * var_break_between_consec_tasks_60[i, j]
                         + BIG_M['Break'] * (1 - var_consecutive_visits_pairs[i, j]) for (i, j) in
                         consecutive_visits_pairs if task_loc_mapping[i] != task_loc_mapping[j])
        model.addConstrs(var_accumulated_break_time[j] <= var_accumulated_break_time[i]
                         + BIG_M['Break'] * (1 - var_consecutive_visits_pairs[i, j]) for
                         (i, j) in consecutive_visits_pairs if task_loc_mapping[i]
                         ==
                         task_loc_mapping[j])

    if policy in {'free_break'}:
        model.addConstrs(var_service_start_time[j] + service_time_task[j] - var_service_start_time[i]
                         <=
                         300 + 150 * var_break_between_tasks_15[i, j]
                         + 315 * var_break_between_tasks_30[i, j]
                         + 330 * var_break_between_tasks_45[i, j]
                         + (MAXIMUM_DUTY_TIME - 300) * var_break_between_tasks_60[i, j] for (i, j) in index_pairs)
    elif policy in {'30_min_block_free_break'}:
        model.addConstrs(var_service_start_time[j] + service_time_task[j] - var_service_start_time[i]
                         <=
                         270 + 330 * var_break_between_tasks_30[i, j]
                         + (MAXIMUM_DUTY_TIME - 270) * var_break_between_tasks_60[i, j] for (i, j) in index_pairs)
    elif policy == 'break_in_depot':
        model.addConstrs(var_service_start_time[j] + service_time_task[j]
                         + distance_matrix[task_loc_mapping[j]][task_loc_mapping[0]]
                         * (var_break_between_consec_tasks_15.sum(j, '*')
                            + var_break_between_consec_tasks_30.sum(j, '*')
                            + var_break_between_consec_tasks_45.sum(j, '*')
                            + var_break_between_consec_tasks_60.sum(j, '*'))
                         - (var_service_start_time[i] - distance_matrix[task_loc_mapping[0]][task_loc_mapping[i]]
                            * (var_break_between_consec_tasks_15.sum('*', i)
                               + var_break_between_consec_tasks_30.sum('*', i)
                               + var_break_between_consec_tasks_45.sum('*', i)
                               + var_break_between_consec_tasks_60.sum('*', i)))
                         <=
                         300 + 150 * var_break_between_tasks_15[i, j] + 315 * var_break_between_tasks_30[i, j]
                         + 330 * var_break_between_tasks_45[i, j]
                         + (MAXIMUM_DUTY_TIME - 300) * var_break_between_tasks_60[i, j]
                         for (i, j) in index_pairs)
    elif policy == '30_min_block_break_in_depot':
        model.addConstrs(var_service_start_time[j] + service_time_task[j]
                         + distance_matrix[task_loc_mapping[j]][task_loc_mapping[0]]
                         * (var_break_between_consec_tasks_15.sum(j, '*')
                            + var_break_between_consec_tasks_30.sum(j, '*')
                            + var_break_between_consec_tasks_45.sum(j, '*')
                            + var_break_between_consec_tasks_60.sum(j, '*'))
                         - (var_service_start_time[i]
                            - distance_matrix[task_loc_mapping[0]][task_loc_mapping[i]]
                            * (var_break_between_consec_tasks_15.sum('*', i)
                               + var_break_between_consec_tasks_30.sum('*', i)
                               + var_break_between_consec_tasks_45.sum('*', i)
                               + var_break_between_consec_tasks_60.sum('*', i)))
                         <= 270
                         + 330 * var_break_between_tasks_30[i, j]
                         + (MAXIMUM_DUTY_TIME - 270) * var_break_between_tasks_60[i, j]
                         for (i, j) in index_pairs)

    if policy in {"break_in_depot", "free_break"}:
        model.addConstrs(var_accumulated_break_time[j] - var_accumulated_break_time[i]
                         >=
                         15 * var_break_between_tasks_15[i, j]
                         + 30 * var_break_between_tasks_30[i, j]
                         + 45 * var_break_between_tasks_45[i, j]
                         + 60 * var_break_between_tasks_60[i, j]
                         - 60 * (1 - var_break_between_tasks_15[i, j]
                                 - var_break_between_tasks_30[i, j]
                                 - var_break_between_tasks_45[i, j]
                                 - var_break_between_tasks_60[i, j])
                         for (i, j) in index_pairs)
    elif policy in {'30_min_block_break_in_depot', '30_min_block_free_break'}:
        model.addConstrs(var_accumulated_break_time[j] - var_accumulated_break_time[i]
                         >=
                         30 * var_break_between_tasks_30[i, j]
                         + 60 * var_break_between_tasks_60[i, j]
                         - 60 * (1 - var_break_between_tasks_15[i, j]
                                   - var_break_between_tasks_30[i, j]
                                   - var_break_between_tasks_45[i, j]
                                   - var_break_between_tasks_60[i, j])
                         for (i, j) in index_pairs)

    model.addConstrs(var_break_between_tasks_15[i, j]
                     + var_break_between_tasks_30[i, j]
                     + var_break_between_tasks_45[i, j]
                     + var_break_between_tasks_60[i, j]
                     <= 1
                     for (i, j) in index_pairs)

    if policy == 'no_break':
        model.addConstrs(var_break_between_consec_tasks_15[i, j] == 0 for (i, j) in consecutive_visits_pairs)
        model.addConstrs(var_break_between_consec_tasks_30[i, j] == 0 for (i, j) in consecutive_visits_pairs)
        model.addConstrs(var_break_between_consec_tasks_45[i, j] == 0 for (i, j) in consecutive_visits_pairs)
        model.addConstrs(var_break_between_consec_tasks_60[i, j] == 0 for (i, j) in consecutive_visits_pairs)

    # valid inequality
    model.addConstr(quicksum(var_break_between_consec_tasks_15[i, j]
                             + var_break_between_consec_tasks_30[i, j]
                             + var_break_between_consec_tasks_45[i, j]
                             + var_break_between_consec_tasks_60[i, j] for (i, j) in consecutive_visits_pairs)
                    <= 4)
    # Objective
    model.setObjective(var_service_start_time[last_task_index] - var_service_start_time[0], sense=GRB.MINIMIZE)
    return model, var_consecutive_visits_pairs, var_service_start_time, last_task_index


def create_precedence_constraints_free_break_policy(model, var_service_start_time, var_break_between_consec_tasks_15,
                                                    var_break_between_consec_tasks_30, var_break_between_consec_tasks_45,
                                                    var_break_between_consec_tasks_60, service_time_task,
                                                    distance_matrix, task_loc_mapping, var_consecutive_visits_pairs,
                                                    consecutive_visits_pairs, pickup_delivery_pairs, BIG_M):
    model.addConstrs(var_service_start_time[i] + 15 * var_break_between_consec_tasks_15[i, j]
                     + 30 * var_break_between_consec_tasks_30[i, j]
                     + 45 * var_break_between_consec_tasks_45[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     - var_service_start_time[j]
                     - BIG_M["Time"] * (1 - var_consecutive_visits_pairs[i, j])
                     <= 0
                     for (i, j) in set(consecutive_visits_pairs) - set(pickup_delivery_pairs))
    model.addConstrs(var_service_start_time[i] + 15 * var_break_between_consec_tasks_15[i, j]
                     + 30 * var_break_between_consec_tasks_30[i, j]
                     + 45 * var_break_between_consec_tasks_45[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     - var_service_start_time[j]
                     <= 0
                     for (i, j) in set(pickup_delivery_pairs))
    return model


def create_precedence_constraints_30_min_block_free_break_policy(model, var_service_start_time,
                                                                 var_break_between_consec_tasks_30,
                                                                 var_break_between_consec_tasks_60,
                                                                 service_time_task, distance_matrix,
                                                                 task_loc_mapping, var_consecutive_visits_pairs,
                                                                 consecutive_visits_pairs, pickup_delivery_pairs, big_m):
    model.addConstrs(var_service_start_time[i]
                     + 30 * var_break_between_consec_tasks_30[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     - var_service_start_time[j] - big_m["Time"] * (1 - var_consecutive_visits_pairs[i, j]) <= 0
                     for (i, j) in set(consecutive_visits_pairs) - set(pickup_delivery_pairs))
    model.addConstrs(var_service_start_time[i] + 30 * var_break_between_consec_tasks_30[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     - var_service_start_time[j]
                     <= 0
                     for (i, j) in set(pickup_delivery_pairs))
    return model


def create_precedence_constraints_break_in_depot_policy(model, var_service_start_time,
                                                        var_break_between_consec_tasks_15,
                                                        var_break_between_consec_tasks_30,
                                                        var_break_between_consec_tasks_45,
                                                        var_break_between_consec_tasks_60,
                                                        service_time_task, distance_matrix, task_loc_mapping,
                                                        var_consecutive_visits_pairs, consecutive_visits_pairs,
                                                        pickup_delivery_pairs, big_m):
    model.addConstrs(var_service_start_time[i]
                     + 15 * var_break_between_consec_tasks_15[i, j]
                     + 30 * var_break_between_consec_tasks_30[i, j]
                     + 45 * var_break_between_consec_tasks_45[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     + (distance_matrix[task_loc_mapping[i]][task_loc_mapping[0]]
                        + distance_matrix[task_loc_mapping[0]][task_loc_mapping[j]]
                        - distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]])
                     * (var_break_between_consec_tasks_15[i, j] + var_break_between_consec_tasks_30[i, j]
                        + var_break_between_consec_tasks_45[i, j]
                        + var_break_between_consec_tasks_60[i, j])
                     - var_service_start_time[j] - big_m["Time"] * (1 - var_consecutive_visits_pairs[i, j])
                     <= 0
                     for (i, j) in set(consecutive_visits_pairs) - set(pickup_delivery_pairs))
    model.addConstrs(var_service_start_time[i]
                     + 15 * var_break_between_consec_tasks_15[i, j]
                     + 30 * var_break_between_consec_tasks_30[i, j]
                     + 45 * var_break_between_consec_tasks_45[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     + (distance_matrix[task_loc_mapping[i]][task_loc_mapping[0]]
                        + distance_matrix[task_loc_mapping[0]][task_loc_mapping[j]]
                        - distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]])
                     * (var_break_between_consec_tasks_15[i, j] + var_break_between_consec_tasks_30[i, j]
                        + var_break_between_consec_tasks_45[i, j] + var_break_between_consec_tasks_60[i, j])
                     - var_service_start_time[j]
                     <= 0
                     for (i, j) in set(pickup_delivery_pairs))
    return model


def create_precedence_constraints_30_min_block_break_in_depot_policy(model, var_service_start_time,
                                                                     var_break_between_consec_tasks_30,
                                                                     var_break_between_consec_tasks_60,
                                                                     service_time_task, distance_matrix,
                                                                     task_loc_mapping, var_consecutive_visits_pairs,
                                                                     consecutive_visits_pairs, pickup_delivery_pairs,
                                                                     big_m):
    model.addConstrs(var_service_start_time[i]
                     + 30 * var_break_between_consec_tasks_30[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     + (distance_matrix[task_loc_mapping[i]][task_loc_mapping[0]]
                        + distance_matrix[task_loc_mapping[0]][task_loc_mapping[j]]
                        - distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]])
                     * (var_break_between_consec_tasks_30[i, j] + var_break_between_consec_tasks_60[i, j])
                     - var_service_start_time[j] - big_m["Time"] * (1 - var_consecutive_visits_pairs[i, j])
                     <= 0
                     for (i, j) in set(consecutive_visits_pairs) - set(pickup_delivery_pairs))
    model.addConstrs(var_service_start_time[i]
                     + 30 * var_break_between_consec_tasks_30[i, j]
                     + 60 * var_break_between_consec_tasks_60[i, j]
                     + service_time_task[i] + distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]]
                     + (distance_matrix[task_loc_mapping[i]][task_loc_mapping[0]]
                        + distance_matrix[task_loc_mapping[0]][task_loc_mapping[j]]
                        - distance_matrix[task_loc_mapping[i]][task_loc_mapping[j]])
                     * (var_break_between_consec_tasks_30[i, j] + var_break_between_consec_tasks_60[i, j])
                     - var_service_start_time[j]
                     <= 0
                     for (i, j) in set(pickup_delivery_pairs))
    return model
