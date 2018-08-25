import numpy
import numpy.linalg
import math
import random

def toRotation(w):
    """
    This function converts vector w(lie group) to rotation matrix.
    Args:
      w: A 3D vector.
    Returns:
      matrix: A 3x3 Matrix.
    """
#TODO CHECK IF IT IS 0,0,0
    w_length = numpy.linalg.norm(w)
    w_hat = numpy.array([
        [    0,-w[2], w[1]],
        [ w[2],    0,-w[0]],
        [-w[1], w[0],    0]
    ])
    eye = numpy.eye(3)
    e_w = eye + \
          w_hat/w_length * math.sin(w_length) + \
          numpy.matmul(w_hat, w_hat) / w_length / w_length * (1 - math.cos(w_length))
    return e_w

def toLieGroup(R):
    """
    This function converts rotation matrix to vector w(lie group).
    Args:
      R: A 3x3 rotation matrix.
    Returns:
      w: A 3D vector.
    """
    w_length = math.acos((numpy.trace(R) - 1)/2.0)
    w = w_length / (2 * math.sin(w_length)) * numpy.array(
        [R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]]
        )
    return w

if __name__ == "__main__":

    mat1 = toRotation(numpy.array([random.random(),random.random(),random.random()]))
    w = toLieGroup(mat1)
    mat2 = toRotation(numpy.array(w))
    print numpy.allclose(mat1,mat2)




