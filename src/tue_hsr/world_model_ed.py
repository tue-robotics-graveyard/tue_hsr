#!/usr/bin/env python
import ed.srv
import ed_sensor_integration.srv
import rospy
import visualization_msgs.msg
from geometry_msgs.msg import Point
from ed.srv import SimpleQuery, SimpleQueryRequest, UpdateSrv, Configure
from ed_gui_server.srv import GetEntityInfo

from .robot_part import RobotPart


class ED(RobotPart):
    def __init__(self, robot_name, tf_listener):
        super(ED, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._ed_simple_query_srv = self.create_service_client('/%s/ed/simple_query' % robot_name, SimpleQuery)
        self._ed_entity_info_query_srv = self.create_service_client('/%s/ed/gui/get_entity_info' % robot_name,
                                                                    GetEntityInfo)
        self._ed_update_srv = self.create_service_client('/%s/ed/update' % robot_name, UpdateSrv)
        self._ed_kinect_update_srv = self.create_service_client('/%s/ed/kinect/update' % robot_name,
                                                                ed_sensor_integration.srv.Update)
        self._ed_configure_srv = self.create_service_client('/%s/ed/configure' % robot_name, Configure)
        self._ed_reset_srv = self.create_service_client('/%s/ed/reset' % robot_name, ed.srv.Reset)
        self._ed_get_image_srv = self.create_service_client('/%s/ed/kinect/get_image' % robot_name,
                                                            ed_sensor_integration.srv.GetImage)
        self._ed_ray_trace_srv = self.create_service_client('/%s/ed/ray_trace' % robot_name,
                                                            ed_sensor_integration.srv.RayTrace)

        self._tf_listener = tf_listener

        self._marker_publisher = rospy.Publisher("/" + robot_name + "/ed/simple_query", visualization_msgs.msg.Marker,
                                                 queue_size=10)

        self.robot_name = robot_name

    def wait_for_connections(self, timeout):
        """ Waits for the connections until they are connected
        :param timeout: timeout in seconds
        :return: bool indicating whether all connections are connected
        """
        return super(ED, self).wait_for_connections(timeout) and self.navigation.wait_for_connections(timeout)

    # ----------------------------------------------------------------------------------------------------
    #                                             QUERYING
    # ----------------------------------------------------------------------------------------------------

    def get_entities(self, type="", center_point=Point(), radius=0, id=""):
        self._publish_marker(center_point, radius)

        center_point_in_map = center_point.projectToFrame("/map", self._tf_listener)
        query = SimpleQueryRequest(id=id, type=type, center_point=center_point_in_map,
                                   radius=radius)

        try:
            entity_infos = self._ed_simple_query_srv(query).entities
            entities = map(from_entity_info, entity_infos)
        except Exception, e:
            rospy.logerr("ERROR: robot.ed.get_entities(id=%s, type=%s, center_point=%s, radius=%s)" % (
            id, type, str(center_point), str(radius)))
            rospy.logerr("L____> [%s]" % e)
            return []

        return entities

    def get_entity(self, id):
        entities = self.get_entities(id=id)
        if len(entities) == 0:
            return None

        assert len(entities) == 1
        return entities[0]

    def get_entity_info(self, id):
        return self._ed_entity_info_query_srv(id=id, measurement_image_border=20)

    # ----------------------------------------------------------------------------------------------------
    #                                             UPDATING
    # ----------------------------------------------------------------------------------------------------

    def reset(self, keep_all_shapes=True):
        try:
            self._ed_reset_srv(keep_all_shapes=keep_all_shapes)
        except rospy.ServiceException, e:
            rospy.logerr("Could not reset ED: {0}".format(e))

        rospy.sleep(.2)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _publish_marker(self, center_point, radius):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = center_point.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = 2

        marker.pose.position.x = center_point.vector.x()
        marker.pose.position.y = center_point.vector.y()
        marker.pose.position.z = center_point.vector.z()
        marker.lifetime = rospy.Duration(20.0)
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius

        marker.color.a = 0.5
        marker.color.r = 1

        self._marker_publisher.publish(marker)
