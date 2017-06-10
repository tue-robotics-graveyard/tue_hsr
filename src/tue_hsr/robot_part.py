class RobotPart(object):
    """ Base class for robot parts """
    def __init__(self, robot_name, tf_listener):
        """ Constructor
        :param robot_name: string with robot name
        :param tf_listener: tf listener object
        """
        self.robot_name = robot_name
        self.tf_listener = tf_listener