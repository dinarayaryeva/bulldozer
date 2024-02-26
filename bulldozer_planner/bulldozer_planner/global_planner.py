import rclpy
from rclpy.node import Node
from bulldozer_interfaces.srv import GlobalPlan
from nav_msgs.msg import Path

class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('service_topic', rclpy.Parameter.Type.STRING),
                ('working_width', rclpy.Parameter.Type.DOUBLE),
                ('angle2axis', rclpy.Parameter.Type.DOUBLE),
            ])
        # topic names
        service_topic = self.get_parameter(
            'service_topic').get_parameter_value().string_value

        # service
        self.srv = self.create_service(
            GlobalPlan, service_topic, self.service_callback)

    def service_callback(self, request, response):
        operation_id = request.operation_id
        zone = request.zone
        grid_map = request.grid_map

        if operation_id == 1:  # align mounded heaps
            trajectory = self.align_heaps(zone, grid_map)
        else:
            trajectory = self.clean_snow(zone, grid_map)

        response.trajectory = trajectory

        return response
    
    def align_heaps(self, zone, grid_map):
        pass

    def clean_snow(self, zone, grid_map):
        path = Path()
        pass


def main(args=None):
    rclpy.init(args=args)

    global_planner = GlobalPlanner()

    rclpy.spin(global_planner)

    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
