"""
Generate a geodesic of SO(3) equipped
with its left-invariant canonical METRIC
for trajectory generation for a robotic manipulator
and sends the trajectory through a redis server at a selected rate
"""

from geomstats.special_orthogonal_group import SpecialOrthogonalGroup

import numpy as np
import redis
import time

SO3_GROUP = SpecialOrthogonalGroup(n=3)
METRIC = SO3_GROUP.bi_invariant_metric

redis_server = redis.StrictRedis(host='localhost', port=6379, db=0)
DESIRED_ORIENTATION_KEY = "geomstats_examples::desired_orientation"
DESIRED_POSITION_KEY = "geomstats_examples::desired_position"

# [s]
TRAJECTORY_TIME = 5.0
# [Hz]
LOOP_FREQUENCY = 100.0


def deserialize_matrix_redis(redis_key):
    """
    reads a the value corresponding to 'redis_key'
    from the redis server and returns it as a 2D array
    """
    lines = str(redis_server.get(redis_key)).split('\'')[1].split('; ')
    matrix = np.array([x.split(' ') for x in lines])

    return matrix.astype(np.float)


def serialize_matrix_to_redis(redis_key, mat, float_fmt='%08f'):
    """
    writes a np 2D array 'mat' to the redis server
    using the key 'redis_key'
    """
    line_separator = ';'
    col_separator = ' '
    each_float_as_str = np.char.mod(float_fmt, mat)
    each_line_as_str = [col_separator.join(f) for f in each_float_as_str]
    redis_entry = line_separator.join(each_line_as_str)
    redis_server.set(redis_key, redis_entry)


def main():

    initial_orientation = deserialize_matrix_redis(DESIRED_ORIENTATION_KEY)
    initial_point = SO3_GROUP.rotation_vector_from_matrix(initial_orientation)

    final_orientation = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    final_point = SO3_GROUP.rotation_vector_from_matrix(final_orientation)

    geodesic = METRIC.geodesic(initial_point=initial_point,
                               end_point=final_point)

    n_steps = int(TRAJECTORY_TIME * LOOP_FREQUENCY)
    t = np.linspace(0, 1, n_steps)

    points = geodesic(t)

    period = 1.0 / LOOP_FREQUENCY
    t_init = time.time()
    t = t_init
    for point in points:
        current_point = point
        rot_desired = SO3_GROUP.matrix_from_rotation_vector(current_point)[0]
        serialize_matrix_to_redis(DESIRED_ORIENTATION_KEY, rot_desired)

        t += period
        time.sleep(max(0.0, t - time.time()))

    elapsed_time = time.time() - t_init
    print("Elapsed time : ", elapsed_time, " seconds")
    print("Loop cycles  : ", len(points))
    print("Frequency    : ", len(points) / elapsed_time, " hz")


if __name__ == "__main__":
    main()
