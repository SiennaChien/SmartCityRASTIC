from cav_project.msg import limo_state
from cav_class import CAV

def search_ahead(search_info, limo_num):
    for i in range(limo_num-1, -1, -1):
        if search_info[limo_num]["current_line"] == search_info[i]["current_line"]:
            front_limo = i
            break
        else:
            front_limo = -1
    return front_limo

def search_previous(limo_num):
    previous_limo = limo_num-1
    return previous_limo

def calc_distance(x1, y1, x2, y2):
    distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    return distance

def calc_qp_info(search_info, limo_num):
    front_num = search_ahead(search_info, limo_num)
    previous_num = search_previous(limo_num)

    collision_pt = search_info[limo_num]["current_end_pt"]
    starting_pt = search_info[limo_num]["current_starting_pt"]

    dk = calc_distance(search_info[previous_limo]["current_pos"][0], search_info[previous_limo]["current_pos"][1], collision_pt[0], collision_pt[1])
    di = calc_distance(search_info[limo_num]["current_pos"][0], search_info[limo_num]["current_pos"][1], collision_pt[0], collision_pt[1])
    d2 = di - dk
    d1 = calc_distance(search_info[limo_num]["current_pos"][0], search_info[limo_num]["current_pos"][1], search_info[front_limo]["current_pos"][0], search_info[front_limo]["current_pos"][1])
    d0 = calc_distance(search_info[limo_num]["current_pos"][0], search_info[limo_num]["current_pos"][1], starting_pt[0], starting_pt[1])

    if previous_num == -1:
        d2 = -1
    if front_num == -1:
        d1 = -1
    
    current_cav = search_info[limo_num]["cav_name"]
    front_cav = search_info[front_num]["cav_name"]
    previous_cav = search_info[previous_num]["cav_name"]

    limo_state = limo_state()
    limo_state.limoID = current_cav.ID
    limo_state.vel = current_cav.velocity
    limo_state.d0 = d0
    limo_state.d1 = d1
    limo_state.v1 = front_cav.velocity
    limo_state.d2 = d2
    limo_state.v2 = previous_cav.velocity


    return limo_state