import abc
from timeit import default_timer as timer

class MeasureIt:
    def __init__(self,logger,text):
        self.start = timer()
        self.logger = logger
        self.text = text

    def end(self):
        self.logger.info(f'{self.text}: {timer() - self.start} s')


class Estimator(abc.ABC):
    def __init__(self,logger,config):
        self.logger = logger
        self.config = config

    def get_logger(self):
        return self.logger

    def get_config(self):
        return self.config

    def measureit(self,text):
        return MeasureIt(self.logger,text)

    @abc.abstractmethod
    def estimate(self, color_msg, depth_msg, camera_msg):
        pass