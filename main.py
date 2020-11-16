import Table
import numpy as np
from  math import pi, sin ,cos, atan2
from pybotics.robot import Robot
from pybotics.predefined_models import puma560



def question1():
    puma560 = Table.getPUMA560()
    res = np.eye(4, dtype='float32')
    for joint in puma560:
        a = Table.getTable(joint[0], joint[1], joint[2], joint[3])
        a = ((a*10000).astype('int32')/10000).astype('float32')
        res = np.matmul(res, a)


    return  res

def question2(Tn):
    ik = Table.inverseKinematics(Tn)
    t1 = ik.findTheta_1()
    t3 = ik.findTheta_3()
    t2 = ik.findTheta_2()
    t4 = ik.findTheta_4()
    t6 = ik.findTheta_6()
    t5 = ik.findTheta_5()
    
    sol1 = np.array([t1[0], t2[0], t3[0], t4[0], t5[0], t6[1]] ) / pi * 180
    sol2 = np.array([t1[0], t2[1], t3[1], t4[3], t5[3], t6[2]] ) / pi * 180
    sol3 = np.array([t1[0], t2[0], t3[0], t4[1], t5[1], t6[0]] ) / pi * 180
    sol4 = np.array([t1[0], t2[1], t3[1], t4[2], t5[2], t6[3]] ) / pi * 180
    sol5 = np.array([t1[1], t2[2], t3[0], t4[4], t5[5], t6[4]] ) / pi * 180
    sol6 = np.array([t1[1], t2[3], t3[1], t4[6], t5[7], t6[6]]) / pi * 180
    sol7 = np.array([t1[1], t2[2], t3[0], t4[5], t5[4], t6[5]]) / pi * 180
    sol8 = np.array([t1[1], t2[3], t3[1], t4[7], t5[6], t6[7]]) / pi * 180
    return [sol1, sol2, sol3, sol4, sol5, sol6, sol7, sol8]
    '''
    d3 = 0.149; d4= 0.433;
    a2 = 0.432; a3= -0.02;

    px = Tn[0][3]; ax = Tn[0][2]; ox = Tn[0][1]; nx = Tn[0][0];
    py = Tn[1][3]; ay = Tn[1][2]; oy = Tn[1][1]; ny = Tn[1][0];
    pz = Tn[2][3]; az = Tn[2][2]; oz = Tn[2][1]; nz = Tn[2][0];
    p1 = (px**2 + py**2)**0.5
    p3 = (d4**2 + a3**2)**0.5
    M = (px**2 + py**2 + pz**2 - a2**2 - a3**2 - d4**2 - d3**2) / (2 * a2)

    t1[0] = (atan2(py, px) - atan2(d3 / p1, (1 - (d3 / p1)**2)**0.5))
    t1[1] = (atan2(py, px) - atan2(d3 / p1, -(1 - (d3 / p1)**2)**0.5))
    t3[0] = (-atan2(a3, d4) + atan2(M / p3, (1 - (M / p3)**2)**0.5))
    t3[1] = (-atan2(a3, d4) + atan2(M / p3, -(1 - (M / p3)**2)**0.5))
    t2[0] = atan2(
        -(a3 + a2 * cos(t3[0])) * pz + (cos(t1[0]) * px + sin(t1[0]) * py) * (a2 * sin(t3[0]) + d4)
        , (d4 + a2 * sin(t3[0])) * pz + (cos(t1[0]) * px + sin(t1[0]) * py) * (a2 * cos(t3[0]) + a3)) - t3[0];
    t2[1] = atan2(
        -(a3 + a2 * cos(t3[1])) * pz + (cos(t1[0]) * px + sin(t1[0]) * py) * (a2 * sin(t3[1]) + d4)
        , (d4 + a2 * sin(t3[1])) * pz + (cos(t1[0]) * px + sin(t1[0]) * py) * (a2 * cos(t3[1]) + a3)) - t3[1];
    t2[2] = atan2(
        -(a3 + a2 * cos(t3[0])) * pz + (cos(t1[1]) * px + sin(t1[1]) * py) * (a2 * sin(t3[0]) + d4)
        , (d4 + a2 * sin(t3[0])) * pz + (cos(t1[1]) * px + sin(t1[1]) * py) * (a2 * cos(t3[0]) + a3)) - t3[0];
    t2[3] = atan2(
        -(a3 + a2 * cos(t3[1])) * pz + (cos(t1[1]) * px + sin(t1[1]) * py) * (a2 * sin(t3[1]) + d4)
        , (d4 + a2 * sin(t3[1])) * pz + (cos(t1[1]) * px + sin(t1[1]) * py) * (a2 * cos(t3[1]) + a3)) - t3[1];

    t4[0] = atan2(-ax * sin(t1[0]) + cos(t1[0]) * ay,
                      ax * cos(t1[0]) * cos(t2[0] + t3[0]) + sin(t1[0]) * cos(t2[0] + t3[0]) * ay - sin(
                          t2[0] + t3[0]) * az)
    t4[1] = atan2(ax * sin(t1[0]) - cos(t1[0]) * ay,
                      -ax * cos(t1[0]) * cos(t2[0] + t3[0]) - sin(t1[0]) * cos(t2[0] + t3[0]) * ay + sin(
                          t2[0] + t3[0]) * az)
    t4[2] = atan2(-ax * sin(t1[0]) + cos(t1[0]) * ay,
                      ax * cos(t1[0]) * cos(t2[1] + t3[1]) + sin(t1[0]) * cos(t2[1] + t3[1]) * ay - sin(
                          t2[1] + t3[1]) * az)
    t4[3] = atan2(ax * sin(t1[0]) - cos(t1[0]) * ay,
                      -ax * cos(t1[0]) * cos(t2[1] + t3[1]) - sin(t1[0]) * cos(t2[1] + t3[1]) * ay + sin(
                          t2[1] + t3[1]) * az)
    t4[4] = atan2(-ax * sin(t1[1]) + cos(t1[1]) * ay,
                      ax * cos(t1[1]) * cos(t2[2] + t3[0]) + sin(t1[1]) * cos(t2[2] + t3[0]) * ay - sin(
                          t2[2] + t3[0]) * az)
    t4[5] = atan2(ax * sin(t1[1]) - cos(t1[1]) * ay,
                      -ax * cos(t1[1]) * cos(t2[2] + t3[0]) - sin(t1[1]) * cos(t2[2] + t3[0]) * ay + sin(
                          t2[2] + t3[0]) * az)
    t4[6] = atan2(-ax * sin(t1[1]) + cos(t1[1]) * ay,
                      ax * cos(t1[1]) * cos(t2[3] + t3[1]) + sin(t1[1]) * cos(t2[3] + t3[1]) * ay - sin(
                          t2[3] + t3[1]) * az)
    t4[7] = atan2(ax * sin(t1[1]) - cos(t1[1]) * ay,
                      -ax * cos(t1[1]) * cos(t2[3] + t3[1]) - sin(t1[1]) * cos(t2[3] + t3[1]) * ay + sin(
                          t2[3] + t3[1]) * az)

    t5[0] = atan2(-cos(t1[0]) * sin(t2[0] + t3[0]) * ox - sin(t1[0]) * sin(t2[0] + t3[0]) * oy - cos(
        t2[0] + t3[0]) * oz
                      , cos(t1[0]) * sin(t2[0] + t3[0]) * nx + sin(t1[0]) * sin(t2[0] + t3[0]) * ny + cos(
            t2[0] + t3[0]) * nz)
    t5[1] = atan2(cos(t1[0]) * sin(t2[0] + t3[0]) * ox + sin(t1[0]) * sin(t2[0] + t3[0]) * oy + cos(
        t2[0] + t3[0]) * oz
                      ,
                      -cos(t1[0]) * sin(t2[0] + t3[0]) * nx - sin(t1[0]) * sin(t2[0] + t3[0]) * ny - cos(
                          t2[0] + t3[0]) * nz)
    t5[2] = atan2(-cos(t1[0]) * sin(t2[1] + t3[1]) * ox - sin(t1[0]) * sin(t2[1] + t3[1]) * oy - cos(
        t2[1] + t3[1]) * oz
                      , cos(t1[0]) * sin(t2[1] + t3[1]) * nx + sin(t1[0]) * sin(t2[1] + t3[1]) * ny + cos(
            t2[1] + t3[1]) * nz)
    t5[3] = atan2(cos(t1[0]) * sin(t2[1] + t3[1]) * ox + sin(t1[0]) * sin(t2[1] + t3[1]) * oy + cos(
        t2[1] + t3[1]) * oz
                      ,
                      -cos(t1[0]) * sin(t2[1] + t3[1]) * nx - sin(t1[0]) * sin(t2[1] + t3[1]) * ny - cos(
                          t2[1] + t3[1]) * nz)
    t5[4] = atan2(-cos(t1[1]) * sin(t2[2] + t3[0]) * ox - sin(t1[1]) * sin(t2[2] + t3[0]) * oy - cos(
        t2[2] + t3[0]) * oz
                      , cos(t1[1]) * sin(t2[2] + t3[0]) * nx + sin(t1[1]) * sin(t2[2] + t3[0]) * ny + cos(
            t2[2] + t3[0]) * nz)
    t5[5] = atan2(cos(t1[1]) * sin(t2[2] + t3[0]) * ox + sin(t1[1]) * sin(t2[2] + t3[0]) * oy + cos(
        t2[2] + t3[0]) * oz
                      ,
                      -cos(t1[1]) * sin(t2[2] + t3[0]) * nx - sin(t1[1]) * sin(t2[2] + t3[0]) * ny - cos(
                          t2[2] + t3[0]) * nz)
    t5[6] = atan2(-cos(t1[1]) * sin(t2[3] + t3[1]) * ox - sin(t1[1]) * sin(t2[3] + t3[1]) * oy - cos(
        t2[3] + t3[1]) * oz
                      , cos(t1[1]) * sin(t2[3] + t3[1]) * nx + sin(t1[1]) * sin(t2[3] + t3[1]) * ny + cos(
            t2[3] + t3[1]) * nz)
    t5[7] = atan2(cos(t1[1]) * sin(t2[3] + t3[1]) * ox + sin(t1[1]) * sin(t2[3] + t3[1]) * oy + cos(
        t2[3] + t3[1]) * oz
                      ,-cos(t1[1]) * sin(t2[3] + t3[1]) * nx - sin(t1[1]) * sin(t2[3] + t3[1]) * ny - cos(
        t2[3] + t3[1]) * nz)

    t6[0] = atan2((cos(t1[0]) * cos(t4[0]) * cos(t2[0] + t3[0]) - sin(t1[0]) * sin(t4[0])) * ax + (
                cos(t1[0]) * sin(t4[0]) + cos(t4[0]) * cos(t2[0] + t3[0]) * sin(t1[0])) * ay - cos(t4[0]) * sin(
t2[0] + t3[0]) * az, cos(t1[0]) * sin(t2[0] + t3[0]) * ax + sin(t1[0]) * sin(
        t2[0] + t3[0]) * ay + cos(t2[0] + t3[0]) * az)

    t6[1] = atan2((cos(t1[0]) * cos(t4[1]) * cos(t2[0] + t3[0]) - sin(t1[0]) * sin(t4[1])) * ax + (
                cos(t1[0]) *
                sin(t4[1]) + cos(t4[1]) * cos(t2[0] + t3[0]) * sin(t1[0])) * ay - cos(t4[1]) * sin(
        t2[0] + t3[0]) * az, cos(t1[0]) * sin(t2[0] + t3[0]) * ax + sin(t1[0]) * sin(
        t2[0] + t3[0]) * ay + cos(t2[0] + t3[0]) * az)
    t6[2] = atan2((cos(t1[0]) * cos(t4[2]) * cos(t2[1] + t3[1]) - sin(t1[0]) * sin(t4[2])) * ax + (
                cos(t1[0]) *
                sin(t4[2]) + cos(t4[2]) * cos(t2[1] + t3[1]) * sin(t1[0])) * ay - cos(t4[2]) * sin(
        t2[1] + t3[1]) * az, cos(t1[0]) * sin(t2[1] + t3[1]) * ax + sin(t1[0]) * sin(
        t2[1] + t3[1]) * ay + cos(t2[1] + t3[1]) * az)
    t6[3] = atan2((cos(t1[0]) * cos(t4[3]) * cos(t2[1] + t3[1]) - sin(t1[0]) * sin(t4[3])) * ax + (
                cos(t1[0]) *
                sin(t4[3]) + cos(t4[3]) * cos(t2[1] + t3[1]) * sin(t1[0])) * ay - cos(t4[3]) * sin(
        t2[1] + t3[1]) * az, cos(t1[0]) * sin(t2[1] + t3[1]) * ax + sin(t1[0]) * sin(
        t2[1] + t3[1]) * ay + cos(t2[1] + t3[1]) * az)
    t6[4] = atan2((cos(t1[1]) * cos(t4[4]) * cos(t2[2] + t3[0]) - sin(t1[1]) * sin(t4[4])) * ax + (
                cos(t1[1]) *
                sin(t4[4]) + cos(t4[4]) * cos(t2[2] + t3[0]) * sin(t1[1])) * ay - cos(t4[4]) * sin(
        t2[2] + t3[0]) * az, cos(t1[1]) * sin(t2[2] + t3[0]) * ax + sin(t1[1]) * sin(
        t2[2] + t3[0]) * ay + cos(t2[2] + t3[0]) * az)
    t6[5] = atan2((cos(t1[1]) * cos(t4[5]) * cos(t2[2] + t3[0]) - sin(t1[1]) * sin(t4[5])) * ax + (
                cos(t1[1]) *
                sin(t4[5]) + cos(t4[5]) * cos(t2[2] + t3[0]) * sin(t1[1])) * ay - cos(t4[5]) * sin(
        t2[2] + t3[0]) * az, cos(t1[1]) * sin(t2[2] + t3[0]) * ax + sin(t1[1]) * sin(
        t2[2] + t3[0]) * ay + cos(t2[2] + t3[0]) * az)
    t6[6] = atan2((cos(t1[1]) * cos(t4[6]) * cos(t2[3] + t3[1]) - sin(t1[1]) * sin(t4[6])) * ax + (
                cos(t1[1]) *
                sin(t4[6]) + cos(t4[6]) * cos(t2[3] + t3[1]) * sin(t1[1])) * ay - cos(t4[6]) * sin(
        t2[3] + t3[1]) * az, cos(t1[1]) * sin(t2[3] + t3[1]) * ax + sin(t1[1]) * sin(
        t2[3] + t3[1]) * ay + cos(t2[3] + t3[1]) * az)
    t6[7] = atan2((cos(t1[1]) * cos(t4[7]) * cos(t2[3] + t3[1]) - sin(t1[1]) * sin(t4[7])) * ax + (
                cos(t1[1]) *
                sin(t4[7]) + cos(t4[7]) * cos(t2[3] + t3[1]) * sin(t1[1])) * ay - cos(t4[7]) * sin(
        t2[3] + t3[1]) * az, cos(t1[1]) * sin(t2[3] + t3[1]) * ax + sin(t1[1]) * sin(
        t2[3] + t3[1]) * ay + cos(t2[3] + t3[1]) * az)

    sol1 = np.array([t1[0], t2[0], t3[0], t4[0], t6[0], t5[1]])/ pi * 180
    sol2 = np.array([t1[0], t2[1], t3[1], t4[2], t6[2], t5[3]]) / pi * 180
    sol3 = np.array([t1[0], t2[0], t3[0], t4[1], t6[1], t5[0]]) / pi * 180
    sol4 = np.array([t1[0], t2[1], t3[1], t4[3], t6[3], t5[2]]) / pi * 180
    sol5 = np.array([t1[1], t2[2], t3[0], t4[4], t6[4], t5[5]]) / pi * 180
    sol6 = np.array([t1[1], t2[3], t3[1], t4[6], t6[6], t5[7]]) / pi * 180
    sol7 = np.array([t1[1], t2[2], t3[0], t4[5], t6[5], t5[4]]) / pi * 180
    sol8 = np.array([t1[1], t2[3], t3[1], t4[7], t6[7], t5[6]]) / pi * 180
    return [sol1, sol2, sol3, sol4, sol5, sol6, sol7, sol8]'''


if __name__ == "__main__":
    T = question1()
    soln = question2(T)
    print(soln[1])
    


