"""
the support poses are given, object in random poses around the AABB of the support
this is the 4th simulation process to obtain new stable pose dataset

env: regrasp_placement
"""
import sys
import os
import pybullet as p
import numpy as np
import random
import math
import time
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


logretrain = os.path.join(BASE_DIR, '4th_sim_record_makeup.txt')  # save simulation results
if os.path.exists(logretrain):
    os.remove(logretrain)
    print("the log file is removed")

loginfo_0 = os.path.join(BASE_DIR, 'support_pose.txt')  # support poses
lines_0 = open(loginfo_0, 'r').readlines()
# print(lines_0)
sup_dict = {}
for line in lines_0:
    line = line.strip().split()
    nm = line[0]
    ps = [line[1], line[2], line[3], line[4], line[5], line[6]]
    # print(ps)
    ele = {nm: ps}
    # print(ele)
    sup_dict.update(ele)
# print(sup_dict)

timesteps1 = 5 * 240  # contact in 5s
timesteps2 = 6 * 240  # simulation 8s to stable pose

loginfo = os.path.join(BASE_DIR, 'makeup_pairs.txt')  # pairs fit in sizes; 786pairs3
lines = open(loginfo, 'r').readlines()

j = 0
for lineq in lines:
    j += 1

    lineq = lineq.strip().split()
    sup = lineq[0]
    obj = lineq[1]

    # print(sup_dict[sup])
    line = sup_dict[sup]

    index02 = 0
    indexp1 = 0
    indexp2 = 0
    indexp3 = 0

    for m in range(100):  # each pair run 50 simulation runs

        # load plane
        physicsClient = p.connect(p.DIRECT)  # DIRECT;GUI
        p.setGravity(0, 0, -9.8)
        planeId = p.loadURDF("/home/xu/Documents/mesh_plane/plane.urdf")  # the friction is changed to 0.8

        # load fixture
        fixtureStartPos = [float(line[0]), float(line[1]), float(line[2])]
        fixtureStartOri = [float(line[3]), float(line[4]), float(line[5])]
        fixtureStartOrientation = p.getQuaternionFromEuler(fixtureStartOri)
        p.setAdditionalSearchPath("/home/xu/Documents/mesh_fixture_urdf")
        fixture = p.loadURDF(sup + '.urdf', fixtureStartPos, fixtureStartOrientation)
        # the fiction is changed
        p.changeDynamics(fixture, -1, lateralFriction=0.8, contactStiffness=100000000, contactDamping=1)  # stiff 1000000

        aabb1 = p.getAABB(fixture)
        aabbMax1 = aabb1[1]
        side_length = max(abs(aabbMax1[0]), abs(aabbMax1[1]))  # restrict random poses of the object
        # the object starts at random poses within bounding box of the support
        a = (random.random() - 0.5) * 2.0
        b = (random.random() - 0.5) * 2.0
        redu1, redu2, redu3 = a * side_length, b * side_length, aabbMax1[2] + 0.5  # the height should be large
        objStartPos = [redu1, redu2, redu3]
        # print(a, b)
        c = random.random()  # [0, 1)
        d = random.random()
        e = random.random()
        redu4, redu5, redu6 = c * math.pi * 2,  d * math.pi * 2,  e*math.pi * 2
        objStratOri = [redu4, redu5, redu6]
        objStartOrientation = p.getQuaternionFromEuler(objStratOri)

        # the parameters name should be the same for the judgement
        p.setAdditionalSearchPath("/home/xu/Documents/mesh_object_urdf")
        object_class = p.loadURDF(obj + '.urdf', objStartPos, objStartOrientation)
        p.changeDynamics(object_class, -1, lateralFriction=0.8, contactStiffness=100000000, contactDamping=1)

        p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=0, cameraPitch=-10,
                                     cameraTargetPosition=[-0., 0., -0.])

        # start simulation and record different placements
        file = sys.stdout
        # mode=w overlap the file before; mode=a not overlap
        sys.stdout = open(logretrain, mode='a')

        # judge if two meshes overlap
        w = p.getClosestPoints(fixture, object_class, -0.001)  # the distance can be changed
        w2 = p.getClosestPoints(planeId, object_class, -0.001)

        # if not overlap!!!!!!!!
        if len(w) == 0 and len(w2) == 0:  # this line relates to p.getClosestPoints
            for i in range(timesteps1):
                # first judge if contact
                contactInfo = p.getContactPoints(fixture, object_class)
                if len(contactInfo) > 0:  # when falling down and touch the support
                    objectPos0, objectOrn0 = p.getBasePositionAndOrientation(object_class)
                    objectOrn0 = p.getEulerFromQuaternion(objectOrn0)
                    # simu time after contact, run full 6s
                    for k2 in range(timesteps2):
                        p.stepSimulation()
                        time.sleep(1. / 3840.)  # 3840.
                    objectPos, objectOrn = p.getBasePositionAndOrientation(object_class)
                    objectOrn = p.getEulerFromQuaternion(objectOrn)

                    thred = 0.01
                    thred2 = math.pi * 2 / 360 * 10
                    if abs(objectPos[0] - objectPos0[0]) > thred or abs(objectPos[1] - objectPos0[1]) > thred or \
                            abs(objectPos[2] - objectPos0[2]) > thred or abs(objectOrn[0] - objectOrn0[0]) > thred2 or \
                            abs(objectOrn[1] - objectOrn0[1]) > thred2 or abs(objectOrn[2] - objectOrn0[2]) > thred2:
                        print(sup, fixtureStartPos[0], fixtureStartPos[1], fixtureStartPos[2],
                              fixtureStartOri[0], fixtureStartOri[1], fixtureStartOri[2],
                              '02', sep=" ", end="\n")
                        print(obj, objectPos0[0], objectPos0[1], objectPos0[2], objectOrn0[0], objectOrn0[1], objectOrn0[2],
                              '02', sep=" ", end="\n")
                        index02 += 1
                    break

                p.stepSimulation()
                time.sleep(1. / 3840.)  # 3840.

            contactInfo = p.getContactPoints(fixture, object_class)
            contactInfo2 = p.getContactPoints(planeId, object_class)

            objectvels = p.getBaseVelocity(object_class)
            a = np.array(objectvels[0])
            vel = math.sqrt(np.sum(a ** 2))

            aabb2 = p.getAABB(object_class)
            aabbMax2 = aabb2[1]
            # aabbMin2 = aabb2[0]

            objectPos, objectOrn = p.getBasePositionAndOrientation(object_class)
            objectOrn = p.getEulerFromQuaternion(objectOrn)

            # record stable pose
            velthred = 1e-4
            heightthred = aabbMax1[2]*0.2  # the height of the support

            if len(contactInfo) > 0 and all(i[8] for i in contactInfo) > 0 and len(contactInfo2) == 0 and vel < velthred and objectPos[2] > aabbMax1[2]:  # p1

                print(sup, fixtureStartPos[0], fixtureStartPos[1], fixtureStartPos[2],
                      fixtureStartOri[0], fixtureStartOri[1], fixtureStartOri[2],
                      "p1", sep=" ", end="\n")
                print(obj, objectPos[0], objectPos[1], objectPos[2], objectOrn[0], objectOrn[1], objectOrn[2],
                      "p1", sep=" ", end="\n")
                indexp1 += 1

            elif len(contactInfo) > 0 and all(i[8] for i in contactInfo) > 0 and len(contactInfo2) > 0 and vel < velthred and objectPos[2] > heightthred \
                    and abs(objectPos[0]) > abs(aabbMax1[0]) and abs(objectPos[1]) > abs(aabbMax1[1]):  # p3

                print(sup, fixtureStartPos[0], fixtureStartPos[1], fixtureStartPos[2],
                      fixtureStartOri[0], fixtureStartOri[1], fixtureStartOri[2],
                      "p3", sep=" ", end="\n")
                print(obj, objectPos[0], objectPos[1], objectPos[2], objectOrn[0], objectOrn[1], objectOrn[2],
                      "p3", sep=" ", end="\n")
                indexp3 += 1

            elif len(contactInfo) > 0 and all(i[8] for i in contactInfo) > 0 and len(contactInfo2) == 0 and vel < velthred and heightthred < objectPos[2] < aabbMax1[2] \
                    and abs(objectPos[0]) < abs(aabbMax1[0]) and abs(objectPos[1]) < abs(aabbMax1[1]):  # p2

                print(sup, fixtureStartPos[0], fixtureStartPos[1], fixtureStartPos[2],
                      fixtureStartOri[0], fixtureStartOri[1], fixtureStartOri[2],
                      "p2", sep=" ", end="\n")
                print(obj, objectPos[0], objectPos[1], objectPos[2], objectOrn[0], objectOrn[1], objectOrn[2],
                      "p2", sep=" ", end="\n")
                indexp2 += 1

        sys.stdout.close()
        # restore print commands to interactive prompt
        sys.stdout = file
        p.disconnect()

    print(j, sup, obj, index02, indexp1, indexp2, indexp3)
