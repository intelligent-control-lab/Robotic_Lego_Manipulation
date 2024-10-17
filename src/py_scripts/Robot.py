import numpy as np
import open3d as o3d
import copy

class Robot():
    def __init__(self):
        self.DH = np.matrix([[0, 0, 0, -np.pi/2],
                             [-np.pi/2, 0, 0.26, np.pi],
                             [0, 0, 0.015, -np.pi/2],
                             [0, -0.29, 0, np.pi/2],
                             [0, 0, 0, -np.pi/2],
                             [0, 0, 0, 0]]) # FTS and mounting plate thickness
        self.DH_tool = np.matrix([[0, 0, 0, -np.pi/2],
                             [-np.pi/2, 0, 0.26, np.pi],
                             [0, 0, 0.015, -np.pi/2],
                             [0, -0.29, 0, np.pi/2],
                             [0, 0, 0, -np.pi/2],
                             [0, -0.183, 0, np.pi]])
        self.DH_tool_assemble = np.matrix([[0, 0, 0, -np.pi/2],
                             [-np.pi/2, 0, 0.26, np.pi],
                             [0, 0, 0.015, -np.pi/2],
                             [0, -0.29, 0, np.pi/2],
                             [0, 0, 0, -np.pi/2],
                             [0, -0.183, 0.0078, np.pi]])
        self.DH_tool1 = np.matrix([[0, 0, 0, -np.pi/2],
                             [-np.pi/2, 0, 0.26, np.pi],
                             [0, 0, 0.015, -np.pi/2],
                             [0, -0.29, 0, np.pi/2],
                             [np.pi/2, 0, 0, -np.pi/2],
                             [np.pi, 0.0078 + 0.0096, 0.183 + 0.0032 - 0.0078,  0]])
        
        self.DH_tool1_assemble = np.matrix([[0, 0, 0, -np.pi/2],
                             [-np.pi/2, 0, 0.26, np.pi],
                             [0, 0, 0.015, -np.pi/2],
                             [0, -0.29, 0, np.pi/2],
                             [np.pi/2, 0, 0, -np.pi/2],
                             [np.pi, 0.0078, 0.183 + 0.0032 - 0.0078,  0]])
        self.base_trans = np.matrix([[1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 0.33],
                                     [0, 0, 0, 1]])

    def FK_deg(self, q, DH):
        q_rad = np.asarray(q, dtype=np.float32)
        for i in range(q_rad.shape[0]):
            q_rad[i] = q_rad[i] / 180 * np.pi
        return self.FK(q_rad, DH)
    
    def FK(self, q, DH): # q in radian
        q_rad = np.asarray(q, dtype=np.float32)
        trans_mtx = np.copy(self.base_trans)

        DH[:, 0] = DH[:, 0] + q_rad
        for i in range(DH.shape[0]):
            tmp = np.matrix([[np.cos(DH[i, 0]), -np.sin(DH[i, 0]) * np.cos(DH[i, 3]),  np.sin(DH[i, 0]) * np.sin(DH[i, 3]), DH[i, 2] * np.cos(DH[i, 0])],
                             [np.sin(DH[i, 0]),  np.cos(DH[i, 0]) * np.cos(DH[i, 3]), -np.cos(DH[i, 0]) * np.sin(DH[i, 3]), DH[i, 2] * np.sin(DH[i, 0])],
                             [0,                 np.sin(DH[i, 3]),                     np.cos(DH[i, 3]),                    DH[i, 1]],
                             [0,                 0,                                    0,                                   1]])
            trans_mtx = np.matmul(trans_mtx, tmp)
        return trans_mtx
    
if __name__ == "__main__":
    robot = Robot()
    q = np.array([0, 0, 0, 0, 0, 0]).reshape((6, 1))

    base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    ee_trans = robot.FK_deg(q, robot.DH)
    ee_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05).transform(ee_trans)

    tool_trans = robot.FK_deg(q, robot.DH_tool)
    tool_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05).transform(tool_trans)

    tool_assemble_trans = robot.FK_deg(q, robot.DH_tool_assemble)
    tool_assemble_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05).transform(tool_assemble_trans)

    tool1_assemble_trans = robot.FK_deg(q, robot.DH_tool1_assemble)
    tool1_assemble_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05).transform(tool1_assemble_trans)

    tool_1_trans = robot.FK_deg(q, robot.DH_tool1)
    tool_1_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05).transform(tool_1_trans)
    o3d.visualization.draw_geometries([base_frame, ee_frame, tool_frame, tool_assemble_frame, tool_1_frame, tool1_assemble_frame])