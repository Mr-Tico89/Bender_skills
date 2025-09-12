import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import math

class RobotSkills(Node):
    def __init__(self):
        super().__init__('robot_skills')

        # API de navegación de Nav2
        self.navigator = BasicNavigator()

        # Publisher directo a cmd_vel (para rotar manualmente)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.goal_handle = None
        self.is_executing_goal = False

    # 1. Obtener posición actual
    def where_am_i(self):
        pose = self.navigator.getCurrentPose()
        if pose:
            self.get_logger().info(f"Pose actual: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        else:
            self.get_logger().warn("No se pudo obtener la pose actual")
        return pose

    # 2. Enviar al robot a una pose
    def go_to_pose(self, x, y, yaw=0.0):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.navigator.goToPose(goal_pose)
        self.is_executing_goal = True
        self.get_logger().info(f"Navegando hacia: ({x:.2f}, {y:.2f}) con yaw={yaw:.2f}")

    # 3. Cancelar navegación
    def cancel(self):
        if self.is_executing_goal:
            self.navigator.cancelTask()
            self.is_executing_goal = False
            self.get_logger().info("Navegación cancelada")

    # 4. ¿Se está moviendo?
    def is_moving(self):
        if not self.is_executing_goal:
            return False
        return self.navigator.isTaskComplete() is False

    # 5. ¿Llegó a la meta?
    def reached(self):
        if not self.is_executing_goal:
            return False

        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Meta alcanzada")
                self.is_executing_goal = False
                return True
            else:
                self.get_logger().warn("No se alcanzó la meta")
                self.is_executing_goal = False
        return False

    # 6. Rotar en el lugar
    def rotate(self, angular_speed=0.5, duration=3.0):
        msg = Twist()
        msg.angular.z = angular_speed
        self.cmd_pub.publish(msg)

        # Esperar durante la duración especificada
        import time
        time.sleep(duration)
        self.stop()
        self.get_logger().info(f"Rotación completada durante {duration} s")

    # Función auxiliar para detener el robot
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = RobotSkills()

    # Ejemplo de uso
    node.where_am_i()
    node.go_to_pose(2.0, 1.0, yaw=math.pi/2)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)

        if node.reached():
            node.rotate(angular_speed=0.5, duration=2.0)
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()