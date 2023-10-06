from math import atan2, degrees, sqrt, acos, radians, degrees

def calc_inv_kin(x, y, z, r):
    try:
        angles = [0.0, 0.0, 0.0, 0.0]
        rear_arm = 135
        forearm = 147 

        angles[3] = radians(r)

        angles[0] = atan2(y,x)

        hypotenuse = pow(x,2) + pow(y,2) + pow(z,2)
        beta = atan2(z, sqrt(pow(x,2) + pow(y,2)))
        psi = abs(acos(( hypotenuse + pow(rear_arm,2) - pow(forearm,2) ) / ( 2 * rear_arm * sqrt(hypotenuse) )))
        if beta >= 0:
            angles[1] = radians(90) - (beta + psi)
        elif beta < 0:
            angles[1] = radians(90) - (psi - abs(beta))

        theta_3 = abs(acos(( hypotenuse - pow(rear_arm,2) - pow(forearm,2) ) / ( 2 * rear_arm*  forearm)))
        angles[2] = theta_3 - radians(90) 

        dobot_angles = [ degrees(angles[0]), degrees(angles[1]), degrees(angles[2]) + degrees(angles[1]), degrees(angles[3])] # in Dobot convention
    except:
        return False

    return dobot_angles


