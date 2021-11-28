#ifndef SS_ALGORITHM_ROBOT_SIM_CPP_GENERAL_H
#define SS_ALGORITHM_ROBOT_SIM_CPP_GENERAL_H

const double GRAVITY_ACC = 9.806;

inline double wrapAngle(double rawAngle)
{
    double ret;
    ret = rawAngle;
    if (ret >= M_PI) {
        while (true) {
            ret = ret - 2 * M_PI;
            if (ret < M_PI) {
                break;
            }
        }
    }
    if (ret < -M_PI) {
        while (true) {
            ret = ret + 2 * M_PI;
            if (ret > -M_PI) {
                break;
            }
        }
    }
    return ret;
}

#endif
