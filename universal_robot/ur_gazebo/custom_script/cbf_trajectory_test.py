#!/usr/bin/env python3
import sys
import rospy
import actionlib
import cvxpy as cp
import numpy as np
import PyKDL as kdl

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromParam

# trajectory_data.py 파일에서 미리 계산된 경로 데이터를 가져옵니다.
from cbf_trajectory_data import POSITION_LIST, VELOCITY_LIST, JOINT_NAMES


class TrajectoryClient:
    """
    Control Barrier Function (CBF)을 사용하여 장애물을 회피하는
    안전한 로봇 경로를 생성하고 실행하는 클래스.
    """
    def __init__(self):
        """클래스 초기화 및 ROS 파라미터 로딩"""
        rospy.init_node("cbf_trajectory_node")

        # --- 가. 파라미터 로딩 ---
        # ROS 파라미터 서버에서 설정값을 불러옵니다. (하드코딩 제거)
        self.controller_name = rospy.get_param("cbf_controller/controller_name", "pos_joint_traj_controller")
        self.p_obs = np.array(rospy.get_param("cbf_controller/obstacle_pos"))
        self.gamma = rospy.get_param("cbf_controller/gamma", 1.0)
        self.robot_radius = rospy.get_param("cbf_controller/robot_radius", 0.05)
        self.obs_radius = rospy.get_param("cbf_controller/obstacle_radius", 0.15)
        rospy.loginfo("CBF parameters loaded.")
        
        self.current_joint_angles = [0.0] * len(JOINT_NAMES)
        self.joint_state_received = False

        # URDF에서 로봇의 Kinematic Chain 정보 로딩
        success, tree = treeFromParam("/robot_description")
        if not success:
            rospy.logerr("Failed to load KDL tree from URDF. Is robot_description on param server?")
            sys.exit(1)

        self.chain = tree.getChain("base_link", "tool0")
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # JointState 구독자
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        # 액션 클라이언트 설정
        self.trajectory_client = actionlib.SimpleActionClient(
            f"{self.controller_name}/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for controller action server...")
        if not self.trajectory_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(1)
        rospy.loginfo("Connected to controller action server.")

    def joint_state_callback(self, msg):
        """/joint_states 토픽을 구독하여 현재 관절 각도를 업데이트하는 콜백 함수"""
        try:
            # 수신된 메시지의 관절 순서를 JOINT_NAMES 순서에 맞게 재정렬
            ordered_positions = [0.0] * len(JOINT_NAMES)
            for i, name in enumerate(JOINT_NAMES):
                idx = msg.name.index(name)
                ordered_positions[i] = msg.position[idx]
            self.current_joint_angles = ordered_positions
            self.joint_state_received = True
        except ValueError as e:
            # 간혹 joint_states 메시지에 모든 관절이 포함되지 않는 경우가 있어 예외 처리
            pass

    def manipulator_cbf(self, q, p, J, u_des):
        """
        Control Barrier Function (CBF) 필터.
        안전 제약조건을 만족하면서 원래 속도와 가장 유사한 '안전한' 속도를 계산합니다.

        Args:
            q (np.array): 현재 관절 각도
            p (np.array): 현재 엔드이펙터 위치 (Cartesian)
            J (np.array): 현재 자세의 자코비안 행렬
            u_des (np.array): 원래 목표하던 관절 속도

        Returns:
            np.array: 안전이 보장된 새로운 관절 속도. 실패 시 0 벡터 반환.
        """
        # --- 나. 안전 장치 강화 ---
        # h(x): 안전 상태를 나타내는 함수. (로봇-장애물 거리)^2 - (안전반경 합)^2
        # h(x) >= 0 이면 안전한 상태.
        hx = np.linalg.norm(p - self.p_obs)**2 - (self.robot_radius + self.obs_radius)**2

        # dh/dx: h(x)의 그라디언트. 로봇이 장애물로부터 멀어지는 방향을 나타냄.
        dhdx = 2 * (p - self.p_obs).transpose() @ J[:3, :]

        # 최적화 문제 설정 (cvxpy)
        u = cp.Variable(len(JOINT_NAMES))
        # 비용 함수: || u - u_des ||^2, 즉, 수정된 속도 u가 원래 속도 u_des와 최대한 비슷해지도록 함.
        cost = cp.sum_squares(u - u_des)
        # 제약 조건: dh/dt >= -gamma * h(x). 이 조건을 만족하면 h(x)가 0 이하로 떨어지지 않음(충돌 방지).
        constraints = [dhdx @ u >= -self.gamma * hx]
        
        problem = cp.Problem(cp.Minimize(cost), constraints)

        print(f"hx={hx:.4f}, dhdx_norm={np.linalg.norm(dhdx):.4f}, u_des_norm={np.linalg.norm(u_des):.4f}")
        problem.solve()

        print(f"u_act_norm={np.linalg.norm(u.value) if problem.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) else 0.0:.4f}, status={problem.status}")

        if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
            return u.value
        else:
            # 최적화 해를 찾지 못한 경우, 잠재적으로 위험한 상황.
            # 로봇을 정지시키는 것이 가장 안전함.
            rospy.logwarn(f"CBF optimization failed with status: {problem.status}. Stopping robot.")
            return np.zeros(len(JOINT_NAMES))

    def calculate_jacobian(self, joint_angles):
        """주어진 관절 각도에 대한 자코비안을 계산합니다."""
        kdl_angles = kdl.JntArray(len(joint_angles))
        for i, angle in enumerate(joint_angles):
            kdl_angles[i] = angle
        jacobian = kdl.Jacobian(len(joint_angles))
        self.jac_solver.JntToJac(kdl_angles, jacobian)
        return np.array([[jacobian[i, j] for j in range(len(joint_angles))] for i in range(6)])

    def get_cartesian_position(self, joint_angles):
        """주어진 관절 각도에 대한 엔드이펙터의 Cartesian 위치를 계산합니다."""
        kdl_angles = kdl.JntArray(len(joint_angles))
        for i, angle in enumerate(joint_angles):
            kdl_angles[i] = angle
        end_effector_frame = kdl.Frame()
        self.fk_solver.JntToCart(kdl_angles, end_effector_frame)
        pos = end_effector_frame.p
        return np.array([pos[0], pos[1], pos[2]])

    def generate_safe_trajectory(self):
        """
        미리 정의된 경로를 CBF로 필터링하여 안전한 경로를 생성합니다.
        """
        num_points = len(POSITION_LIST)
        time_duration = 4.0
        dt = time_duration / (num_points - 1)
        duration_list = np.linspace(dt, time_duration, num_points).tolist()

        safe_positions = []
        safe_velocities = []

        # 경로의 시작점은 미리 정의된 첫 번째 위치
        current_q = np.array(POSITION_LIST[0])
        safe_positions.append(current_q)

        for i in range(num_points - 1):
            p = self.get_cartesian_position(current_q)
            J = self.calculate_jacobian(current_q)
            u_des = np.array(VELOCITY_LIST[i])
            
            u_act = self.manipulator_cbf(current_q, p, J, u_des)
            # u_act = u_des # CBF 비활성화 (원래 속도 사용)
            safe_velocities.append(u_act)

            # 다음 위치 계산 (오일러 적분)
            current_q = current_q + u_act * dt
            safe_positions.append(current_q)
        
        # 마지막 속도는 0으로 추가
        safe_velocities.append(np.zeros(len(JOINT_NAMES)))

        return safe_positions, safe_velocities, duration_list

    def execute(self):
        """전체 실행 흐름을 관리합니다."""
        rospy.loginfo("Waiting for initial joint state...")
        while not self.joint_state_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # --- 다. 경로 시작점으로 안전 이동 기능 추가 ---
        rospy.loginfo("Moving to the trajectory start point...")
        start_point = POSITION_LIST[0]
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = start_point
        point.time_from_start = rospy.Duration(3.0) # 시작점까지 3초 동안 이동
        goal.trajectory.points.append(point)
        
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
        rospy.loginfo("Reached start point.")

        # --- CBF 필터링된 메인 경로 생성 및 실행 ---
        rospy.loginfo("Generating and executing CBF-filtered trajectory...")
        safe_positions, safe_velocities, duration_list = self.generate_safe_trajectory()
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        for i, pos in enumerate(safe_positions):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.velocities = safe_velocities[i]
            point.time_from_start = rospy.Duration(duration_list[i] if i < len(duration_list) else 4.0)
            goal.trajectory.points.append(point)
            
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()
        rospy.loginfo(f"Trajectory execution finished in state {result.error_code}")

if __name__ == "__main__":
    try:
        client = TrajectoryClient()
        client.execute()
    except rospy.ROSInterruptException:
        pass
