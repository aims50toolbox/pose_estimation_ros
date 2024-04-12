import abc

class Estimator(abc.ABC):
    def __init__(self,logger,config):
        self.logger = logger
        self.config = config

    def get_logger(self):
        return self.logger

    def get_config(self):
        return self.config

    @abc.abstractmethod
    def estimate(self, color_msg, depth_msg, camera_msg):
        pass