#holds the functions for the limos to search for which other limos to look out for

def search_ahead(cav, limo_num)
    #need to loop through previous limos, go from limo_num and decrease
    #check what path that limo is on, (just check current_path[] for that cav?)
    #find the largest number that is on the same path as the limo
    return front_limo

def search_previous(limo_num)
    previous_limo = limo_num - 1
    if limo_num -1 < 0:
        return previous_limo = 0
    return previous_limo
