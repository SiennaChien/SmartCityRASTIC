from cav_project.msg import limo_state, limo_state_matrix
from cav_class import CAV
import unittest

def calc_qp_info(order_list, limo_num):
    front_num = search_ahead(order_list, limo_num)
    collision_num1, collision_num2 = search_collision(limo_num)
    limo = order_list[limo_num]
    limo_current = limo.current
    collision_pt1 = limo.current_collision_pt1
    collision_pt2 = limo.current_collision_pt2
    starting_pt = limo.points[limo_current]

    d0 = calc_distance((limo.position_x, limo.position_z), starting_pt)

    #constraints for front cav
    if front_num == -1:
        d1 = -1
        v1 = -1
    else:
        d1 = calc_manhattan_d1(order_list, limo_num, front_num)
        v1 = order_list[front_num].velocity

    #constraints for collision cavs
    if collision_num1 == -1:
        d2 = -1
        v2 = -1
    elif collision_num1 == front_num:
        d2 = -1
        v2 = -1
    else:
        dk = calc_manhattan_distance(order_list, limo_num, collision_pt1)
        di = calc_manhattan_distance(order_list, collision_num1, collision_pt1)
        d2 = di - dk
        v2 = order_list[collision_num1].velocity
    
    if collision_num2 == -1:
        d3 = -1
        v3 = -1
    elif collision_num2 == front_num:
        d3 = -1
        v3 = -1
    elif collision_num2 == collision_num1:
        d3 = -1
        v3 = -1
    else:
        dk = calc_manhattan_distance(order_list, limo_num, collision_pt2)
        di = calc_manhattan_distance(order_list, collision_num2, collision_pt2)
        d3 = di - dk
        v3 = order_list[collision_num2].velocity

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
    limo_state_msg.d3 = d3
    limo_state_msg.v3 = v3
    limo_state_msg.vd = vd
    return limo_state_msg


#below are helper functions for calc_qp_info()
def search_ahead(order_list, limo_num):
    for i in range(len(order_list)):
        if order_list[limo_num].current_line == order_list[i].current_line:
            limo_dist = calc_distance(order_list[limo_num].current_position, order_list[limo_num].current_end_pt)
            front_dist = calc_distance(order_list[i].current_position, order_list[limo_num].current_end_pt)
            if front_dist < limo_dist: 
                front_limo = i
            break
        else:
            front_limo = -1
    return front_limo

def search_collision(order_list, limo_num):
    for i in range(len(order_list)):
        if order_list[limo_num].current_collision_pt1 == order_list[i].current_collision_pt1:
            current_collision_pt = order_list[limo_num].current_collision_pt1
            limo_dist = calc_manhattan_distance(order_list, limo_num, current_collision_pt)
            collision_dist = calc_manhattan_distance(order_list, i, current_collision_pt)
            if limo_dist * collision_dist > 0 and collision_dist < limo_dist: 
                collision_limo1 = i
            break
        else:
            collision_limo1 = -1

    if order_list[limo_num].current_collision_pt1 == order_list[limo_num].current_collision_pt2:
        return collision_limo1, -1


    for i in range(len(order_list)):
        if order_list[limo_num].current_collision_pt2 == order_list[i].current_collision_pt1:
            current_collision_pt = order_list[limo_num].current_collision_pt2
            limo_dist = calc_manhattan_distance(order_list, limo_num, current_collision_pt)
            collision_dist = calc_manhattan_distance(order_list, i, current_collision_pt)
            if limo_dist * collision_dist > 0 and collision_dist < limo_dist: 
                collision_limo2 = i
            break
        else:
            collision_limo2 = -1

    return collision_limo1, collision_limo2

def calc_distance(pt_1, pt_2):
    distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
    return distance

def calc_manhattan_d1(order_list, limonum1, limonum2):
    limo2 = order_list[limonum2]
    limo1 = order_list[limonum1]

    if limo2.current_line != limo1.current_line:
        distance = calc_distance((limo1.position_x, limo1.position_y), limo1.current_end_pt)
        for i in range (limo1.next, limo2.current):
            distance += limo1.dist[i]

    else:
        distance = calc_distance((limo1.position_x, limo1.position_z), (limo2.position_x, limo2.position_z))

    return distance

def calc_manhattan_distance(order_list, limo_num, point):
    #get the all_pts array, see if the point comes first, or next turning point comes first
    #if point comes first, calc distance to that point
    #if tuning point comes first, see how many turning points until point, then calc distance to the first turning point
    #calc distance from turning point to next turning point or point depending
    limo = order_list[limo_num]
    for i in range(len(limo.all_pts)):
        if limo.all_pts[i] == limo.current_end_pt or limo.all_pts[i] == point:
            dist = calc_distance(limo.current_position, limo.all_pts[i])
            if limo.all_pts[i] == point:
                return dist
            else:
                for j in range(len(limo.all_pts), i):
                    if limo.all_pts[j] =! point:
                        dist += limo.dist[i]
                        return dist
            break
    return dist


class TestCalcQpInfo(unittest.TestCase):

    def setUp(self):
        self.limo1 = CAV(ID=1, velocity=10, position_x=0, position_z=0, current=0,
                         current_collision_pt1=(5, 5), current_collision_pt2=(10, 10),
                         points=[(0, 0)], within_critical_range=False, current_line=1,
                         current_position=(0, 0), current_end_pt=(20, 20), next=1, dist=[10, 10], all_pts=[(0, 0), (10, 10), (20, 20)])
        self.limo2 = CAV(ID=2, velocity=15, position_x=5, position_z=5, current=1,
                         current_collision_pt1=(5, 5), current_collision_pt2=(15, 15),
                         points=[(5, 5)], within_critical_range=True, current_line=1,
                         current_position=(5, 5), current_end_pt=(20, 20), next=2, dist=[5, 15], all_pts=[(5, 5), (15, 15), (20, 20)])
        self.limo3 = CAV(ID=3, velocity=20, position_x=10, position_z=10, current=2,
                         current_collision_pt1=(10, 10), current_collision_pt2=(25, 25),
                         points=[(10, 10)], within_critical_range=False, current_line=2,
                         current_position=(10, 10), current_end_pt=(30, 30), next=3, dist=[10, 10, 10], all_pts=[(10, 10), (20, 20), (30, 30)])

    def test_no_front_cav(self):
        order_list = [self.limo1, self.limo2, self.limo3]
        limo_num = 0
        result = calc_qp_info(order_list, limo_num)
        self.assertEqual(result.limoID, 1)
        self.assertEqual(result.vel, 10)
        self.assertEqual(result.d1, -1)
        self.assertEqual(result.v1, -1)

    def test_within_critical_range(self):
        order_list = [self.limo1, self.limo2, self.limo3]
        limo_num = 1
        result = calc_qp_info(order_list, limo_num)
        self.assertEqual(result.limoID, 2)
        self.assertEqual(result.vd, 0.4)

    def test_collision_cavs(self):
        order_list = [self.limo1, self.limo2, self.limo3]
        limo_num = 0
        result = calc_qp_info(order_list, limo_num)
        self.assertEqual(result.limoID, 1)
        self.assertEqual(result.d2, -1)
        self.assertEqual(result.v2, -1)
        self.assertEqual(result.d3, -1)
        self.assertEqual(result.v3, -1)

    def test_multiple_collisions(self):
        order_list = [self.limo1, self.limo2, self.limo3]
        limo_num = 1
        result = calc_qp_info(order_list, limo_num)
        self.assertEqual(result.limoID, 2)
        self.assertNotEqual(result.d2, -1)
        self.assertNotEqual(result.v2, -1)

    def test_different_lines(self):
        order_list = [self.limo1, self.limo3]
        limo_num = 0
        result = calc_qp_info(order_list, limo_num)
        self.assertEqual(result.limoID, 1)
        self.assertEqual(result.d1, -1)
        self.assertEqual(result.v1, -1)

if __name__ == '__main__':
    unittest.main()