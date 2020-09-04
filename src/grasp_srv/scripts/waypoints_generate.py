# generate waypoints between start and target
# waypoints on translation
import numpy as np
from scipy.spatial.transform import Rotation as R


def t_waypoints(t_start, t_end, waypoints_num):
    t_step = (t_end - t_start)/(float(waypoints_num))
    print(t_step)
    waypoints = list()
    for i in range(waypoints_num):
        waypoints.append(t_start + i*t_step)
    waypoints.append(t_end)
    return waypoints


# rotation in matrix form
def r_waypoints(r_start, r_end, waypoints_num):
    q_start = R.from_dcm(r_start).as_quat()
    q_end   = R.from_dcm(r_end).as_quat()
    q_step  = (q_end - q_start)/(float(waypoints_num))
    waypoints=  list()
    for i in range(waypoints_num):
        q_mid = q_start + i*q_step
        q_mid /= np.linalg.norm(q_mid)
        r_mid = R.from_quat(q_mid).as_dcm()
        waypoints.append(r_mid)
    waypoints.append(r_end)
    return waypoints


def m_waypoints(m_start, m_end, waypoints_num):
    t_start = m_start[:3, 3]
    t_end   = m_end[:3, 3]
    r_start = m_start[:3, :3]
    r_end   = m_end[:3, :3]
    waypoints_t = t_waypoints(t_start, t_end, waypoints_num)
    waypoints_r = r_waypoints(r_start, r_end, waypoints_num)
    waypoints = list()
    for t_mid, r_mid in zip(waypoints_t, waypoints_r):
        m_mid = np.zeros([4, 4])
        m_mid[:3, :3] = r_mid
        m_mid[:3, 3]  = t_mid
        m_mid[3, 3]   = 1.
        waypoints.append(m_mid)
    waypoints.append(m_end)
    return waypoints


if __name__ == "__main__":
    m_start = np.array([[1.,  0., 0., 0.0],
                        [0.,  1., 0., 0.0],
                        [0.,  0., 1., 0.0],
                        [0.,  0., 0., 1.]])

    m_end = np.array([[0.,  1., 0., 0.2],
                      [-1., 0., 0., 0.3],
                      [0.,  0., 1., 0.1],
                      [0.,  0., 0., 1.]])
    waypoints = m_waypoints(m_start, m_end, 10)
    for waypoint in waypoints:
        print(waypoint)
