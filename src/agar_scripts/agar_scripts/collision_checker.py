import rclpy
from rclpy.time import Time
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import GetCostmap
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from agar_msgs.msg import CollisionZones

from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.line_iterator import LineIterator
from nav2_simple_commander.robot_navigator import BasicNavigator

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion

from math import cos, sin, sqrt

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

        self.timer                  = self.create_timer(0.05, self.run)
        self.nav                    = BasicNavigator()
        self.costmap                = None
        self.footprint              = None
        self.costmap_subscriber     = self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap',
            self.costmap_subscriber_callback, 10)
        # self.footprint_subscriber   = self.create_subscription(
        #     PolygonStamped, 'global_costmap/published_footprint',
        #     self.footprint_subscriber_callback, 10)
    
        self.robot_height           = 2.0
        self.robot_width            = 1.8
        self.source_frame           = 'odom'
        self.target_frame           = 'base_footprint'

        self.distance = sqrt(pow(self.robot_height / 2, 2) + pow(self.robot_width / 2, 2))

        self.footprint_poly_pub     = self.create_publisher(PolygonStamped, 'footprint', 10)
        self.front_stop_poly_pub    = self.create_publisher(PolygonStamped, 'front_stop_footprint', 10)
        self.front_slow_poly_pub    = self.create_publisher(PolygonStamped, 'front_slow_footprint', 10)
        self.rear_stop_poly_pub     = self.create_publisher(PolygonStamped, 'rear_stop_footprint', 10)
        self.rear_slow_poly_pub     = self.create_publisher(PolygonStamped, 'rear_slow_footprint', 10)
        # self.left_stop_poly_pub     = self.create_publisher(PolygonStamped, 'left_stop_footprint', 10)
        # self.left_slow_poly_pub     = self.create_publisher(PolygonStamped, 'left_slow_footprint', 10)
        # self.right_stop_poly_pub    = self.create_publisher(PolygonStamped, 'right_stop_footprint', 10)
        # self.right_slow_poly_pub    = self.create_publisher(PolygonStamped, 'right_slow_footprint', 10)

        self.collision_zones_pub    = self.create_publisher(CollisionZones, 'collision_zones', 10)

        self.front_stop_poly        = None
        self.front_slow_poly        = None
        self.rear_stop_poly         = None
        self.rear_slow_poly         = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    #endregion

    #region Process Data
    def costmap_subscriber_callback(self, msg):
        self.costmap = PyCostmap2D(msg)
        # self.nav.clearGlobalCostmap()

    # def footprint_subscriber_callback(self, msg):
    #     polygon = msg.polygon
    #     self.footprint = polygon

    #     point_A = (polygon.points[0].x, polygon.points[0].y)
    #     point_B = (polygon.points[1].x, polygon.points[1].y)
    #     point_C = (polygon.points[2].x, polygon.points[2].y)
    #     point_D = (polygon.points[3].x, polygon.points[3].y)

    #     (self.front_stop_poly, self.front_slow_poly) = self.generate_zones(point_D, point_A, point_C, point_B, 0.25, 0.75)
    #     self.front_stop_poly_pub.publish(self.front_stop_poly)
    #     self.front_slow_poly_pub.publish(self.front_slow_poly)

    #     (self.rear_stop_poly, self.rear_slow_poly) = self.generate_zones(point_A, point_D, point_B, point_C, 0.25, 0.75)
    #     self.rear_stop_poly_pub.publish(self.rear_stop_poly)
    #     self.rear_slow_poly_pub.publish(self.rear_slow_poly)

    def run(self):
        self.generate_footprint()
        if self.costmap is None or self.footprint is None:
            self.get_logger().warn('OccupancyGrid or Footprint not yet received.')
            return
        try:
            val_f_stop = self.footprintCost(self.front_stop_poly.polygon)
            val_f_slow = self.footprintCost(self.front_slow_poly.polygon)
            val_r_stop = self.footprintCost(self.rear_stop_poly.polygon)
            val_r_slow = self.footprintCost(self.rear_slow_poly.polygon)
            
            # if val_f_stop != 0.0 or val_f_slow != 0.0 or val_r_stop != 0.0 or val_r_slow != 0.0:
            #     self.get_logger().info(f'Front stop: {val_f_stop}, slow: {val_f_slow}')
            #     self.get_logger().info(f'Rear stop: {val_r_stop}, slow: {val_r_slow}')

            collision_zones = CollisionZones()
            collision_zones.front_stop = val_f_stop != 0
            collision_zones.front_slow = val_f_slow != 0
            collision_zones.rear_stop = val_r_stop != 0
            collision_zones.rear_slow = val_r_slow != 0
            self.collision_zones_pub.publish(collision_zones)

        except Exception as ex:
            self.get_logger().error(
                f'Exception: {ex}')
    #endregion
            
    #region Utils
    def generate_footprint(self):
        """
        Generate and publish robot footprint with stop and slow zones.
        """

        try:
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, Time())
                
            pos_x = trans.transform.translation.x
            pos_y = trans.transform.translation.y

            orientation_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            (roll, pitch, pos_z) = euler_from_quaternion(orientation_list)

            ax = pos_x + self.distance * cos(pos_z + 0.67474429)
            ay = pos_y + self.distance * sin(pos_z + 0.67474429)

            bx = pos_x + self.distance * cos(pos_z + 5.60844102)
            by = pos_y + self.distance * sin(pos_z + 5.60844102)

            cx = pos_x + self.distance * cos(pos_z + 3.81633694)
            cy = pos_y + self.distance * sin(pos_z + 3.81633694)

            dx = pos_x + self.distance * cos(pos_z + 2.46684836)
            dy = pos_y + self.distance * sin(pos_z + 2.46684836)

            point_A = (ax, ay)
            point_B = (bx, by)
            point_C = (cx, cy)
            point_D = (dx, dy)

            footprint = self.generate_polygon(point_D, point_A, point_C, point_B)
            self.footprint = footprint.polygon
            self.footprint_poly_pub.publish(footprint)

            (self.front_stop_poly, self.front_slow_poly) = self.generate_zones(point_D, point_A, point_C, point_B, 0.25, 0.75)
            self.front_stop_poly_pub.publish(self.front_stop_poly)
            self.front_slow_poly_pub.publish(self.front_slow_poly)

            (self.rear_stop_poly, self.rear_slow_poly) = self.generate_zones(point_A, point_D, point_B, point_C, 0.25, 0.75)
            self.rear_stop_poly_pub.publish(self.rear_stop_poly)
            self.rear_slow_poly_pub.publish(self.rear_slow_poly)
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')


    def move_point(self, A, B, d):
        """
        Move point on line (A, B) by distance

        Args:
            A (float): Back point.
            B (float): Front point.
            d (float): Distance for to move point from point B.

        Returns:
            tuple of points: x, y (Moved point).
            x (float): point x position.
            y (float): point t position.
        """
        vector_AB = (B[0] - A[0], B[1] - A[1]) 
        length_AB = sqrt(vector_AB[0]**2 + vector_AB[1]**2) 
        unit_vector = (vector_AB[0] / length_AB, vector_AB[1] / length_AB) 
        return (B[0] + d * unit_vector[0], B[1] + d * unit_vector[1])

    """
            D---------------A
            |               |
            |   FOOTPRINT   |
            |               |
            C---------------B
    """
    def generate_zones(self, point_1_back, point_1_front, point_2_back, point_2_front, dis_stop, dis_slow):
        """
        Generate stop and slow robot footprint

        Args:
            point_1_back (float): The rear left point.
            point_1_front (float): The front left point.
            point_2_back (float): The rear right point.
            point_2_front (float): The rear right point.
            dis_stop (float): Distance for stop polygon.
            dis_slow (float): Distance for slow polygon.

        Returns:
            tuple of polygons: polygon_stop, polygon_slow (Generated stop and slow footprint)
        """
        point_A1 = self.move_point(point_1_back, point_1_front, dis_stop)
        point_B1 = self.move_point(point_2_back, point_2_front, dis_stop)

        point_A2 = self.move_point(point_1_back, point_1_front, dis_slow)
        point_B2 = self.move_point(point_2_back, point_2_front, dis_slow)

        stop_poly = self.generate_polygon(point_1_front, point_A1, point_2_front, point_B1)
        slow_poly = self.generate_polygon(point_A1, point_A2, point_B1, point_B2)

        return (stop_poly, slow_poly)

        
    def generate_polygon(self, point_1_back, point_1_front, point_2_back, point_2_front):
        """
        Generate Polygon Stamped

        Args:
            point_1_back (float): The rear left point.
            point_1_front (float): The front left point.
            point_2_back (float): The rear right point.
            point_2_front (float): The rear right point.

        Returns:
            polygon (Generated polygon stamped)
        """
        polygon = PolygonStamped()
        polygon.header.stamp = self.get_clock().now().to_msg()
        polygon.header.frame_id = 'map'
        footprint = Polygon()
        point = Point32()
        point.x = point_1_front[0]
        point.y = point_1_front[1]
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = point_2_front[0]
        point.y = point_2_front[1]
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = point_2_back[0]
        point.y = point_2_back[1]
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = point_1_back[0]
        point.y = point_1_back[1]
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