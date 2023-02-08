import rospy


def print_partition():
    """Prints a partion line, for pretty print
    """
    rospy.loginfo("=" * 65)


def get_part_location_in_bins(part):
    """Accessing the location of parts in the bin
    """
    error = 0.2
    bin_width = (0.819 / 2) + error

    bin1_y = 3.38
    bin2_y = 2.56
    loc = ""
    if part.pose.y >= bin2_y - bin_width and part.pose.y <= bin2_y + bin_width:
        loc = "bin2"

    elif part.pose.y >= bin1_y - bin_width and part.pose.y <= bin1_y + bin_width:
        loc = "bin1"

    return loc


def sort_to_bin(part):
    """Sorting parts to drop in the bins
    """
    y = part.pose.position.y 
    x = part.pose.position.x 
    if y > 0:
        if x > -2.28:
            if y > 2.98:
                return "bin1"
            else:
                return "bin2"
        else:
            if y > 2.98:
                return "bin4"
            else:
                return "bin3"
    else:
        if x > -2.28:
            if y < -2.98:
                return "bin5"
            else:
                return "bin6"
        else:
            if y < -2.98:
                return "bin8"
            else:
                return "bin7"
