import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
import pcl
import numpy as np
from pointcloud_to_grid.pointcloud_to_grid_core import GridMap


class PointcloudToGridNode(Node):
    def __init__(self):
        super().__init__("pointcloud_to_grid_node")
        self.grid_map = GridMap()
        self.declare_parameter("cell_size", self.grid_map.cell_size)
        self.declare_parameter("position_x", self.grid_map.position_x)
        self.declare_parameter("position_y", self.grid_map.position_y)
        self.declare_parameter("length_x", self.grid_map.length_x)
        self.declare_parameter("length_y", self.grid_map.length_y)
        self.declare_parameter("cloud_in_topic", self.grid_map.cloud_in_topic)
        self.declare_parameter("intensity_factor", self.grid_map.intensity_factor)
        self.declare_parameter("height_factor", self.grid_map.height_factor)
        self.declare_parameter("mapi_topic_name", self.grid_map.mapi_topic_name)
        self.declare_parameter("maph_topic_name", self.grid_map.maph_topic_name)
        self.update_params()
        
        self.publisher_igrid = self.create_publisher(
            OccupancyGrid, self.grid_map.mapi_topic_name, 1
        )
        self.publisher_hgrid = self.create_publisher(
            OccupancyGrid, self.grid_map.maph_topic_name, 1
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.grid_map.cloud_in_topic, self.pointcloud_callback, 1
        )
        self.srv = self.create_service(
            Empty,
            "set_parameters",
            self.params_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def params_callback(self, request, response):
        self.update_params()
        return response

    def update_params(self):
        params = self.get_parameters(
            [
                "cell_size",
                "position_x",
                "position_y",
                "length_x",
                "length_y",
                "cloud_in_topic",
                "intensity_factor",
                "height_factor",
                "mapi_topic_name",
                "maph_topic_name",
            ]
        )
        self.grid_map.update_params(params)

    def pointcloud_callback(self, msg):
        out_cloud = pcl.PointCloud()
        pcl.fromROSMsg(msg, out_cloud)
        self.grid_map.initGrids()
        ipoints = np.full(
            (self.grid_map.cell_num_x, self.grid_map.cell_num_y), -128, dtype=np.int8
        )
        hpoints = np.full(
            (self.grid_map.cell_num_x, self.grid_map.cell_num_y), -128, dtype=np.int8
        )
        for out_point in out_cloud:
            cell = self.grid_map.getIndex(out_point.x, out_point.y)
            if (
                cell[0] < self.grid_map.cell_num_x
                and cell[1] < self.grid_map.cell_num_y
            ):
                ipoints[cell[1], cell[0]] = (
                    out_point.intensity * self.grid_map.intensity_factor
                )
                hpoints[cell[1], cell[0]] = out_point.z * self.grid_map.height_factor
        self.grid_map.updateGridData(ipoints, hpoints, msg.header.frame_id)
        self.publisher_igrid.publish(self.grid_map.intensity_grid)
        self.publisher_hgrid.publish(self.grid_map.height_grid)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudToGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
