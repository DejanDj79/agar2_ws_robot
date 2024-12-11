import rclpy
import math 

from rclpy.node import Node

from nav2_msgs.srv import GetCostmap

from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.line_iterator import LineIterator

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32, Polygon, PolygonStamped

NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
MAX_NON_OBSTACLE = 252
FREE_SPACE = 0

class CollisionChecker(Node):
    #region Initialization
    def __init__(self):
        super().__init__('collision_checker')
        self.get_logger().info('Initialization completed')

        # self.timer                  = self.create_timer(0.05, self.run)
        self.nav                    = BasicNavigator()
        self.costmap                = None
        self.footprint              = None
        self.costmap_subscriber     = self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap',
            self.costmap_subscriber_callback, 10)
        self.footprint_subscriber   = self.create_subscription(
            PolygonStamped, 'global_costmap/published_footprint',
            self.footprint_subscriber_callback, 10)
    
        self.front_poly_pub = self.create_publisher(PolygonStamped, 'front_footprint', 10)
        self.rear_poly_pub = self.create_publisher(PolygonStamped, 'rear_footprint', 10)
    #endregion

    #region Process Data
    def costmap_subscriber_callback(self, msg):
        self.costmap = PyCostmap2D(msg)
        # self.nav.clearGlobalCostmap()

    def footprint_subscriber_callback(self, msg):
        polygon = msg.polygon
        self.footprint = polygon

        point_A = (polygon.points[0].x, polygon.points[0].y)
        point_B = (polygon.points[1].x, polygon.points[1].y)
        point_C = (polygon.points[2].x, polygon.points[2].y)
        point_D = (polygon.points[3].x, polygon.points[3].y)

        
        front_poly = self.generate_polygon(point_D, point_A, point_C, point_B, 1.0)
        self.front_poly_pub.publish(front_poly)

        rear_poly = self.generate_polygon(point_A, point_D, point_B, point_C, 1.0)
        self.rear_poly_pub.publish(rear_poly)
        # self.get_logger().info(f'{slow_polygon}')

        
        
        # slowdown_polygon = polygon

    def run(self):
        if self.costmap is None or self.footprint is None:
            self.get_logger().warn('OccupancyGrid or Footprint not yet received.')
            return
        try:
            val = self.footprintCost(self.footprint)
            if val != 0.0:
                self.get_logger().info(f'{val}')
        except Exception as ex:
            self.get_logger().error(
                f'Exception: {ex}')
    #endregion
            
    #region Utils
    def move_point(self, A, B, d): 
        vector_AB = (B[0] - A[0], B[1] - A[1]) 
        length_AB = math.sqrt(vector_AB[0]**2 + vector_AB[1]**2) 
        unit_vector = (vector_AB[0] / length_AB, vector_AB[1] / length_AB) 
        return (B[0] + d * unit_vector[0], B[1] + d * unit_vector[1])
    
    def generate_polygon(self, point_1_back, point_1_front, point_2_back, point_2_front, distance):
        point_A1 = self.move_point(point_1_back, point_1_front, distance)
        point_B1 = self.move_point(point_2_back, point_2_front, distance)

        polygon = PolygonStamped()
        polygon.header.stamp = self.get_clock().now().to_msg()
        polygon.header.frame_id = 'map'
        footprint = Polygon()
        point = Point32()
        point.x = point_A1[0]
        point.y = point_A1[1]
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = point_B1[0]
        point.y = point_B1[1]
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = point_2_front[0]
        point.y = point_2_front[1]
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = point_1_front[0]
        point.y = point_1_front[1]
        point.z = 0.0
        footprint.points.append(point)
        polygon.polygon = footprint
        return polygon
    #endregion


    #region Collision Checker
    def worldToMapValidated(self, wx: float, wy: float):
        """
        Get the map coordinate XY using world coordinate XY.

        Args
        ----
            wx (float): world coordinate X
            wy (float): world coordinate Y

        Returns
        -------
            None: if coordinates are invalid
            tuple of int: mx, my (if coordinates are valid)
            mx (int): map coordinate X
            my (int): map coordinate Y

        """
        if self.costmap is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        # return self.costmap.worldToMapValidated(wx, wy)
        return self.costmap.worldToMap(wx, wy)


    def footprintCost(self, footprint: Polygon):
        """
        Iterate over all the points in a footprint and check for collision.

        Args
        ----
            footprint (Polygon): The footprint to calculate the collision cost for

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            footprint_cost (float): The maximum cost found in the footprint points

        """
        footprint_cost = 0.0
        x1 = 0.0
        y1 = 0.0

        x0, y0 = self.worldToMapValidated(footprint.points[0].x, footprint.points[0].y)

        if x0 is None or y0 is None:
            return LETHAL_OBSTACLE

        xstart = x0
        ystart = y0

        for i in range(len(footprint.points) - 1):
            x1, y1 = self.worldToMapValidated(
                footprint.points[i + 1].x, footprint.points[i + 1].y
            )

            if x1 is None or y1 is None:
                return LETHAL_OBSTACLE

            footprint_cost = max(float(self.lineCost(x0, x1, y0, y1)), footprint_cost)
            x0 = x1
            y0 = y1

            if footprint_cost == LETHAL_OBSTACLE:
                return footprint_cost

        return max(float(self.lineCost(xstart, x1, ystart, y1)), footprint_cost)
    
    def lineCost(self, x0, x1, y0, y1, step_size=0.5):
        """
        Iterate over all the points along a line and check for collision.

        Args
        ----
            x0 (float): Abscissa of the initial point in map coordinates
            y0 (float): Ordinate of the initial point in map coordinates
            x1 (float): Abscissa of the final point in map coordinates
            y1 (float): Ordinate of the final point in map coordinates
            step_size (float): Optional, Increments' resolution, defaults to 0.5

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            line_cost (float): The maximum cost found in the line points

        """
        line_cost = 0.0
        point_cost = -1.0
        line_iterator = LineIterator(x0, y0, x1, y1, step_size)

        while line_iterator.isValid():
            point_cost = self.pointCost(
                int(line_iterator.getX()), int(line_iterator.getY())
            )

            if point_cost == LETHAL_OBSTACLE:
                return point_cost

            if line_cost < point_cost:
                line_cost = point_cost

            line_iterator.advance()

        return line_cost

    def pointCost(self, x: int, y: int):
        """
        Get the cost of a point in the costmap using map coordinates XY.

        Args
        ----
            mx (int): map coordinate X
            my (int): map coordinate Y

        Returns
        -------
            np.uint8: cost of a point

        """
        if self.costmap is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        return self.costmap.getCostXY(x, y)
    #endregion

def main(args=None):
    rclpy.init(args=args)
    collision_checker = CollisionChecker()
    rclpy.spin(collision_checker)
    collision_checker.destroy_node()
    rclpy.shutdown()
    collision_checker.get_logger().info('Closed')

if __name__ == '__main__':
    main()