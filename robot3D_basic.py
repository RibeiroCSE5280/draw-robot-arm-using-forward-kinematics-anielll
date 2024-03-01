#!/usr/bin/env python
# coding: utf-8


from vedo import *
from time import sleep
def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
  
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij
  

def calc(angles, lengths):
  colors = ["yellow","red","blue","green"]
  transforms = []
  frames = []
  origin = np.array([[3],[2], [0.0]]) 
  for i in range(len(angles)):
    arm = Cylinder(r=0.4, height=lengths[i], pos = (lengths[i]/2+0.4,0,0), c=colors[i], alpha=.8, axis=(1,0,0))
    lengths[i] = lengths[i]+0.8
    rot = RotationMatrix(angles[i], axis_name = 'z')
    trans = origin if i==0 else np.array([[lengths[i-1]],[0.0],[0.0]])
    transforms.append(getLocalFrameMatrix(rot, trans))         # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)
    if(i>0):
      transforms[i] = transforms[i-1]@transforms[i]
    arrows = createCoordinateFrameMesh()
    sphere = Sphere(r=0.4).pos(0,0).color("gray").alpha(.8) 
    Frame = arrows + arm + sphere
    Frame.apply_transform(transforms[i])  
    frames.append(Frame)
    end_effector_pos = transforms[-1]@np.array([[-0.4],[0],[0],[1]])
    end_effector_pos = [el[0] for el in end_effector_pos]
    end_effector_pos = end_effector_pos[:-1]
  return (frames, end_effector_pos, transforms)

def forward_kinematics(Phi, L1, L2, L3, L4):
  lengths = [L1,L2,L3,L4]
  output = calc(Phi,lengths)
  T1, T2, T3, T4 = output[2]
  return(T1,T2,T3,T4, output[1])
def lerp_arrays(list1, list2, percentage):
    list3 = []
    for i in range(len(list1)):
        interpolated_value = list1[i] * (1 - percentage) + list2[i] * percentage
        list3.append(interpolated_value)

    return list3
def move(lengths, start_angles,end_angels,plotter):
  animation_frames = 60
  fps = 60
  camera = {
    'position': (10, 5, 50), 
    'focal_point': (10, 5, 0),
  }
  for i in range(animation_frames):
    percent= i / animation_frames
    angles = lerp_arrays(start_angles, end_angels, percent)
    lcopy = [i for i in lengths]
    arms =calc(angles,lcopy)[0]
    plotter.clear()
    for arm in arms:
      plotter += arm
    axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))
    plotter.show(interactive=False,axes=axes, viewup="z",camera=camera)
    sleep(1.0/fps)
    
def main():
  lengths = [5, 8, 3, 0]
  plotter = Plotter()
  angles = [-30, 50, 30, 0]
  end_angles= [30, -50, -30, 0]
  move(lengths,angles,end_angles,plotter)
  angles = end_angles
  end_angles = [45, -60, 15, 0]  # New array for the end angles
  move(lengths, angles, end_angles, plotter)
  angles = end_angles
  end_angles = [-15, 70, -45, 0]  # New array for the end angles
  move(lengths, angles, end_angles, plotter)
  angles = end_angles
  end_angles = [60, -30, 0, 0]  # New array for the end angles
  move(lengths, angles, end_angles, plotter)
  angles = end_angles
  end_angles = [-45, 15, 30, 0]  # New array for the end angles
  move(lengths, angles, end_angles, plotter)
  plotter.close()


if __name__ == '__main__':
    main()



