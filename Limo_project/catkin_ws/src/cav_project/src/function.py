#holds the functions for the limos to search for which other limos to look out for

def search_ahead(path_matrix, limo_num):
    for i in range (limo_num-1, -1, -1):
        #path_matrix stores the limoID, current line, current end point, and current position of all limos 
        #and is constantly updated
        if path_matrix[limo_num][1] == path_matrix[i][1]:
            front_limo = i
            break
        else:
            front_limo = 0
    return front_limo

def search_previous(limo_num):
    previous_limo = limo_num - 1
    if limo_num -1 < 0:
        return 0
    return previous_limo

def calc_distance(x1, y1, x2, y2):
    distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    return distance

def calc_qp_distance(path_matrix, limo_num):
    front_limo = search_ahead(path_matrix, limo_num)
    previous_limo = search_previous(limo_num)

    collision_pt = path_matrix[limo_num][2]

    dk = calc_distance(path_matrix[previous_limo][3][0], path_matrix[previous_limo][3][1], collision_pt[0], collision_pt[1])
    di = calc_distance(path_matrix[limo_num][3][0], path_matrix[limo_num][3][1], collision_pt[0], collision_pt[1])
    print("dk is ", dk, "di is ", di)
    d2 = di - dk
    d1 = calc_distance(path_matrix[limo_num][3][0], path_matrix[limo_num][3][1], path_matrix[front_limo][3][0], path_matrix[front_limo][3][1])
    
    if previous_limo == 0:
        d2 = -1
    if front_limo == 0:
        d1 = -1
    
    return d1, d2

def main():
    path_matrix = [
        [0, (0, 0, 0), (0, 0), (0, 0)],
        [1, (0, 0, 0), (10, 10), (9, 9)],
        [2, (1, 1, 1), (10, 10), (0, 8)],
        [3, (0, 0, 0), (10, 10), (1, 1)]
    ]

    # Test search_ahead function
    limo_num = 2
    front_limo = search_ahead(path_matrix, limo_num)
    print(f"The limo ahead of limo {limo_num} on the same line is: {front_limo}")

    # Test search_previous function
    previous_limo = search_previous(limo_num)
    print(f"The previous limo for limo {limo_num} is: {previous_limo}")

    # Test calc_distance function
    x1, y1 = path_matrix[limo_num][3]
    x2, y2 = path_matrix[front_limo][3]
    distance = calc_distance(x1, y1, x2, y2)
    print(f"The distance between points ({x1}, {y1}) and ({x2}, {y2}) is: {distance}")

    # Test calc_qp_distance function
    d1, d2 = calc_qp_distance(path_matrix, limo_num)
    print(f"The distances d1 and d2 for limo {limo_num} are: d1 = {d1}, d2 = {d2}")

if __name__ == "__main__":
    main()
