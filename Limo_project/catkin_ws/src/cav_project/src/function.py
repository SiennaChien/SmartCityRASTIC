from cav_project.msg import limo_state, limo_state_matrix
from cav_class import CAV

def search_ahead(search_info, limo_num):
    for i in range(limo_num-1, -1, -1):
        if search_info[limo_num]["current_line"] == search_info[i]["current_line"]:
            front_limo = i
            break
        else:
            front_limo = -1
    if limo_num == 0:
        return -1
    return front_limo

def search_previous(limo_num):
    previous_limo = limo_num-1
    return previous_limo

def calc_distance(x1, y1, x2, y2):
    distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    return distance

def calc_manhattan_distance(search_info, limo_num, front_num):
    front_cav = search_info[front_num]["cav_object"]
    cav = search_info[limo_num]["cav_object"]
    distance = calc_distance(cav.position_x, cav.position_y, cav.points[cav.next][0], cav.points[cav.next][1])
    for i in range (cav.next, front_cav.current):
        distance += cav.dist[i]

    return distance

def calc_qp_info(search_info, limo_num):
    front_num = search_ahead(search_info, limo_num)
    previous_num = search_previous(limo_num)

    collision_pt = search_info[limo_num]["current_end_pt"]
    starting_pt = search_info[limo_num]["current_start_pt"]

    dk = calc_distance(search_info[previous_num]["current_pos"][0], search_info[previous_num]["current_pos"][1], collision_pt[0], collision_pt[1])
    di = calc_distance(search_info[limo_num]["current_pos"][0], search_info[limo_num]["current_pos"][1], collision_pt[0], collision_pt[1])
    d2 = di - dk
    d1 = calc_distance(search_info[limo_num]["current_pos"][0], search_info[limo_num]["current_pos"][1], search_info[front_num]["current_pos"][0], search_info[front_num]["current_pos"][1])
    d0 = calc_distance(search_info[limo_num]["current_pos"][0], search_info[limo_num]["current_pos"][1], starting_pt[0], starting_pt[1])

    current_cav = search_info[limo_num]["cav_object"]

    if previous_num == -1:
        d2 = -1
    if front_num == -1:
        d1 = -1
    if previous_num == front_num:
        d2 = -1
    if search_info[limo_num]["current_end_pt"] != search_info[previous_num]["current_end_pt"]:
        d2 = -1

    limo_state_msg = limo_state()
    limo_state_msg.limoID = current_cav.ID
    limo_state_msg.vel = current_cav.velocity
    limo_state_msg.d0 = d0
    limo_state_msg.d1 = d1
    if front_num > -1:
        limo_state_msg.v1 = search_info[front_num]["cav_object"].velocity
    else:
        limo_state_msg.v1 = -1

    limo_state_msg.d2 = d2
    if previous_num > -1:
        limo_state_msg.v2 = search_info[previous_num]["cav_object"].velocity
    else:
        limo_state_msg.v2 = -1
    if current_cav.within_critical_range == True:
        limo_state_msg.vd = 0.5
    else:
        limo_state_msg.vd = 0.9

    return limo_state_msg
