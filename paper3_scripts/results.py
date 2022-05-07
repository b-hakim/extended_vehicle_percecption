
"""
# Game Theory - duplicates: 2, total sent (unique): 11, total sent value (unique): 10.198405364824104, total duplicate Value: 2.000518252254722
# Maximum - duplicates: 5, total sent (unique): 11, total sent value (unique): 9.967952582598397, total duplicate Value: 5.736302852082445
# Total not sent messages: 2, total missing value: 1.2943220395748838

"""
import pickle


class Stats:
    def __init__(self):
        self.gt_number_sent_unique = 0
        self.gt_total_sent_value = 0
        self.gt_number_duplicate = 0
        self.gt_total_duplicate_value = 0
        self.gt_num_not_sent_msgs = 0
        self.gt_total_not_sent_value = 0

        self.max_number_sent_unique = 0
        self.max_total_sent_value = 0
        self.max_number_duplicate = 0
        self.max_total_duplicate_value = 0

        self.approaches_counts = [0, 0, 0]

    def print(self):
        print(f"#Game Theory - total sent (unique): {self.gt_number_sent_unique},"            
              f" total sent value (unique): {self.gt_total_sent_value},"
              f" duplicates: {self.gt_number_duplicate}"
              f" total duplicate Value: {self.gt_total_duplicate_value}")

        print(f"#Maximum - total sent (unique): {self.max_number_sent_unique},"
              f" total sent value (unique): {self.max_total_sent_value},"
              f" duplicates: {self.max_number_duplicate},"
              f" total duplicate Value: {self.max_total_duplicate_value}")

        print(f"#Total not sent messages: {self.gt_num_not_sent_msgs},"
              f" total missing value: {self.gt_total_not_sent_value}")

        print(f"Dominant approaches: {self.approaches_counts[0]},"
              f"Dominant - other approaches: {self.approaches_counts[1]}, "
              f"Mixed Strategy approaches: {self.approaches_counts[2]}")

    def __add__(self, other):
        ret = Stats()
        ret.gt_number_sent_unique = self.gt_number_sent_unique + other.gt_number_sent_unique
        ret.gt_total_sent_value = self.gt_total_sent_value + other.gt_total_sent_value
        ret.gt_number_duplicate = self.gt_number_duplicate + other.gt_number_duplicate
        ret.gt_total_duplicate_value = self.gt_total_duplicate_value + other.gt_total_duplicate_value
        ret.gt_num_not_sent_msgs = self.gt_num_not_sent_msgs + other.gt_num_not_sent_msgs
        ret.gt_total_not_sent_value = self.gt_total_not_sent_value + other.gt_total_not_sent_value
        ret.max_number_sent_unique = self.max_number_sent_unique + other.max_number_sent_unique
        ret.max_total_sent_value = self.max_total_sent_value + other.max_total_sent_value
        ret.max_number_duplicate = self.max_number_duplicate + other.max_number_duplicate
        ret.max_total_duplicate_value = self.max_total_duplicate_value + other.max_total_duplicate_value
        ret.approaches_counts = [self.approaches_counts[0] + other.approaches_counts[0],
                                 self.approaches_counts[1] + other.approaches_counts[1],
                                 self.approaches_counts[2] + other.approaches_counts[2],
                                 ]
        return ret

    def __truediv__(self, other):
        if not isinstance(other, int):
            raise TypeError("Only possible division by int")

        ret = Stats()
        ret.gt_number_sent_unique = self.gt_number_sent_unique / other
        ret.gt_total_sent_value = self.gt_total_sent_value / other
        ret.gt_number_duplicate = self.gt_number_duplicate / other
        ret.gt_total_duplicate_value = self.gt_total_duplicate_value / other
        ret.gt_num_not_sent_msgs = self.gt_num_not_sent_msgs / other
        ret.gt_total_not_sent_value = self.gt_total_not_sent_value / other
        ret.max_number_sent_unique = self.max_number_sent_unique / other
        ret.max_total_sent_value = self.max_total_sent_value / other
        ret.max_number_duplicate = self.max_number_duplicate / other
        ret.max_total_duplicate_value = self.max_total_duplicate_value / other
        ret.approaches_counts = [self.approaches_counts[0] / other,
                                 self.approaches_counts[1] / other,
                                 self.approaches_counts[2] / other]
        return ret