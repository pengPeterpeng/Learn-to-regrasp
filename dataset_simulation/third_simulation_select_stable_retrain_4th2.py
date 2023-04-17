"""
use the unstable pose in 2nd_simulation
start pose 01
stable pose p1/p2
write into two files and label the same initial and ending pair

2023/3/14
use 02 data from the new simulation results, not use 01 to avoid always going down
record the stable pose

this is another file to generate test list pairs poses

"""
import pybullet as p
import trimesh
import time
import sys
import os
import math
import numpy as np
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

logretrain = os.path.join(BASE_DIR, '4th_eval.txt')  # results from 02 to stable
if os.path.exists(logretrain):
    os.remove(logretrain)
    print("the log file is removed")

loginfo_0 = os.path.join(BASE_DIR, '4th_simulation_full.txt')  # content with 02
lines_0 = open(loginfo_0, 'r').readlines()
# print(lines_0[0])
# print(lines_0[1])

loginfo = os.path.join(BASE_DIR, 'test_pairs.txt')  # change the name of pairs; test_pairs.txt
lines = open(loginfo, 'r').readlines()
# print(lines[0:3])
# j = 0

pairList = []
for lineq in lines:
    # j += 1
    lineq = lineq.strip().split()
    pair = (lineq[0], lineq[1])
    # print(j)
    pairList.append(
        pair
    )
print(len(pairList))

for pairq in pairList:
    print(pairq)  # each pair
    p_num = 0  # record the stable poses of each pair

    j = 0
    for line in lines_0:  #
        j += 1
        line = line.split()
        if j % 2 == 1:
            name = line[0]  # support
            afterline = lines_0[j].split()  # j is the line number for the start pose of the object
            name2 = afterline[0]  # object
            pair = (name, name2)

            if pair == pairq:  # every pair in the training set
                # print(pairList.index(pair))
                if line[-1] == '02':  # start with the unstable pose
                    # load plane
                    physicsClient = p.connect(p.DIRECT)  # DIRECT;GUI
                    p.setGravity(0, 0, -9.8)
                    planeId = p.loadURDF("/home/xu/Documents/mesh_plane/plane.urdf")  # the friction is changed to 0.5

                    # load fixture
                    fixtureStartPos = [float(line[1]), float(line[2]), float(line[3])]
                    fixtureStartOri = [float(line[4]), float(line[5]), float(line[6])]
                    fixtureStartOrientation = p.getQuaternionFromEuler(fixtureStartOri)
                    p.setAdditionalSearchPath("/home/xu/Documents/mesh_fixture_urdf")
                    fixture = p.loadURDF(name + '.urdf', fixtureStartPos, fixtureStartOrientation)
                    # the fiction is changed to
                    p.changeDynamics(fixture, -1, lateralFriction=0.8, contactStiffness=1000000, contactDamping=1)
                    aabb1 = p.getAABB(fixture)
                    aabbMax1 = aabb1[1]

                    # load object unstable pose here
                    objStartPos = [float(afterline[1]), float(afterline[2]), float(afterline[3])]
                    objStartOrientation = p.getQuaternionFromEuler([float(afterline[4]), float(afterline[5]),
                                                                    float(afterline[6])])
                    p.setAdditionalSearchPath("/home/xu/Documents/mesh_object_urdf_2nd_simulation")
                    object_class = p.loadURDF(name2+'.urdf', objStartPos, objStartOrientation)
                    p.changeDynamics(object_class, -1, lateralFriction=0.8, contactStiffness=1000000, contactDamping=1)

                    # p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=50, cameraPitch=-35,
                    #                              cameraTargetPosition=[-0.6, 0.5, -0.5])

                    file = sys.stdout
                    # mode=w overlap the file before; mode=a not overlap
                    sys.stdout = open(logretrain, mode='a')

                    for i in range(9*240):  # after some time the pose is stable
                        p.stepSimulation()
                        time.sleep(1. / 3840.)  # 3840. to speed up

                    #
                    contactInfo = p.getContactPoints(fixture, object_class)

                    objectvels = p.getBaseVelocity(object_class)
                    a = np.array(objectvels[0])
                    vel = math.sqrt(np.sum(a ** 2))

                    aabb2 = p.getAABB(object_class)
                    aabbMax2 = aabb2[1]
                    # aabbMin2 = aabb2[0]

                    objectPos, objectOrn = p.getBasePositionAndOrientation(object_class)  # stable pose
                    objectOrn = p.getEulerFromQuaternion(objectOrn)

                    # record stable pose
                    velthred = 1e-4
                    heightthred = aabbMax1[2] * 0.2

                    if len(contactInfo) > 0 and all(i[8] for i in contactInfo) > 0 and vel < velthred and objectPos[2] > heightthred:

                        print(name, fixtureStartPos[0], fixtureStartPos[1], fixtureStartPos[2],
                              fixtureStartOri[0],
                              fixtureStartOri[1], fixtureStartOri[2], j, sep=" ", end="\n")
                        print(name2, objectPos[0], objectPos[1], objectPos[2], objectOrn[0], objectOrn[1],
                              objectOrn[2], j, sep=" ", end="\n")
                        p_num += 1

                    sys.stdout.close()
                    # restore print commands to interactive prompt
                    sys.stdout = file
                    p.disconnect()

    print(pairq, p_num)
