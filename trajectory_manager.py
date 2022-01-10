class TrajectoryManager:
    """
    This class aim to assist the following task:
    - Input: Vehicle A trip and current pos/velocity, vehicle B position and current pos/velocity
    - Output: Whether or not vehicle B will intersect vehicle's A trajectory
    """

    def __init__(self, map_file):
        self.map_file = map_file

    def intersect(self, vehicle_a, vehicle_b, threshold=5):
        pass