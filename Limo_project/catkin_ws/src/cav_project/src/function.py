from cav_project.msg import limo_state, limo_state_matrix
from cav_class import CAV

def calc_qp_info(order_list, limo_num):
    front_num = search_ahead(order_list, limo_num)
    previous_num = search_previous(limo_num)
    limo = order_list[limo_num]
    limo_current = limo.current
    collision_pt = limo.current_end_pt
    starting_pt = limo.points[limo_current]

    d0 = calc_distance(limo.position_x, limo.position_z, starting_pt[0], starting_pt[1])

    #constraints for front cav
    if front_num == -1:
        d1 = -1
        v1 = -1
    else:
        d1 = calc_manhattan_distance(order_list, limo_num, front_num)
        v1 = order_list[front_num].velocity

    #constraints for previous cav
    if previous_num == -1:
        d2 = -1
        v2 = -1
    elif previous_num == front_num:
        d2 = -1
        v2 = -1
    elif limo.current_end_pt != order_list[previous_num].current_end_pt:
        d2 = -1
        v2 = -1
    else:
        previous_limo = order_list[previous_num]
        dk = calc_distance(previous_limo.position_x, previous_limo.position_z, collision_pt[0], collision_pt[1])
        di = calc_distance(limo.position_x, limo.position_z, collision_pt[0], collision_pt[1])
        d2 = di - dk
        v2 = order_list[previous_num].velocity

    #constraints for vd
    if limo.within_critical_range == True:
        vd = 0.4
    else:
        vd = 0.7

    #building the message
    limo_state_msg = limo_state()
    limo_state_msg.limoID = limo.ID
    limo_state_msg.vel = limo.velocity
    limo_state_msg.d0 = d0
    limo_state_msg.d1 = d1
    limo_state_msg.v1 = v1
    limo_state_msg.d2 = d2
    limo_state_msg.v2 = v2
    limo_state_msg.vd = vd
    return limo_state_msg


#below are helper functions for calc_qp_info()
def search_ahead(order_list, limo_num):
    limo_next = order_list[limo_num].next

    for i in range(limo_num-1, -1, -1):
        if order_list[limo_num].current_line == order_list[i].current_line:# or order_list[limo_num].lines[limo_next] == order_list[i].current_line:
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

def calc_manhattan_distance(order_list, limo_num, front_num):
    front_limo = order_list[front_num]
    limo = order_list[limo_num]

    if front_limo.current_line != limo.current_line:
        distance = calc_distance(limo.position_x, limo.position_y, limo.current_end_pt[0], limo.current_end_pt[1])
        for i in range (limo.next, front_limo.current):
            distance += limo.dist[i]

    else:
        distance = calc_distance(limo.position_x, limo.position_z, front_limo.position_x, front_limo.position_z)

    return distance
