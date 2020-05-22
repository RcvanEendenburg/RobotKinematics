import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt


class Joint:
    def __init__(self, length, current_angle, min_angle, max_angle, is_static=False, can_y_rotate=False):
        self.length = length
        self.current_angle = current_angle
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.is_static = is_static
        self.can_y_rotate = can_y_rotate

    def get_current_angle(self):
        if self.can_y_rotate:
            return 0
        return self.current_angle

    def set_current_angle(self, angle):
        if angle < self.min_angle:
            raise "angle < min_angle"
        if angle > self.max_angle:
            self.current_angle = self.max_angle
            return angle - self.max_angle
        self.current_angle = angle
        return 0

    def get_end_point_xz(self, start, x_distance):
        start_x, start_y, start_z = start
        end_x = start_x + x_distance * math.sin(math.radians(self.current_angle))
        end_z = start_z + x_distance * math.cos(math.radians(self.current_angle))
        return end_x, end_z

    def get_end_point_xy(self, start, sum_of_angles):
        start_x, start_y = start
        if not self.is_static:
            end_x = start_x + self.length * math.sin(math.radians(sum_of_angles) + math.radians(self.current_angle))
            end_y = start_y + self.length * math.cos(math.radians(sum_of_angles) + math.radians(self.current_angle))
        else:
            end_x = start_x
            end_y = start_y + self.length
        return end_x, end_y


class AL5D:
    def __init__(self, start, preferred_xy_angle):
        self.joints = []
        self.start = start
        self.preferred_xy_angle = preferred_xy_angle

    def add_joint(self, joint):
        self.joints.append(joint)

    def change_angles(self):
        sum_of_angles = 0
        for i in range(0, len(self.joints)):
            sum_of_angles += self.joints[i].get_current_angle()

        excess_angle_sum = abs(sum_of_angles - self.preferred_xy_angle)
        if excess_angle_sum is 0:
            return
        for j in reversed(self.joints):
            excess_angle = j.set_current_angle(j.get_current_angle() + excess_angle_sum)
            if excess_angle is 0:
                break
            excess_angle_sum = excess_angle

    def calculate_xy_points(self):
        start_x, start_y, start_z = self.start
        x_offset = start_x
        y_offset = start_y
        points = []
        sum_of_angles = 0
        if self.preferred_xy_angle is not 0:
            self.change_angles()
        for joint in self.joints:
            end_x, end_y = joint.get_end_point_xy(start=(x_offset, y_offset), sum_of_angles=sum_of_angles)
            sum_of_angles += joint.get_current_angle()
            points.append((end_x, end_y))
            x_offset = end_x
            y_offset = end_y
        return points

    def plot_side_view(self):
        points = self.calculate_xy_points()
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Side view of AL5D')
        previous_x, previous_y, previous_z = self.start
        for point in points:
            x, y = point
            plt.plot(x, y, 'ro')
            plt.plot([previous_x, x], [previous_y, y])
            previous_x, previous_y = point
            plt.annotate("(%0.2f, %0.2f)" % (x, y), point)
        plt.savefig("side_al5d.png")
        plt.clf()

    def plot_top_view(self):
        points = self.calculate_xy_points()
        plt.xlabel('x')
        plt.ylabel('z')
        plt.title('Top view of AL5D')
        start_x, start_y, start_z = self.start
        end_x, end_y = points[len(points) - 1]
        x_distance = abs(end_x - start_x)
        end_z, end_x = self.joints[0].get_end_point_xz(self.start, x_distance)
        plt.plot(start_x, start_z, 'ro')
        plt.plot(end_x, end_z, 'ro')
        plt.plot([start_x, end_x], [start_z, end_z])
        plt.annotate("(%0.2f, %0.2f)" % (start_x, start_z), (start_x, start_z))
        plt.annotate("(%0.2f, %0.2f)" % (end_x, end_z), (end_x, end_z))
        plt.savefig("top_al5d.png")


al5d = AL5D(start=(0, 0, 0), preferred_xy_angle=180)

al5d.add_joint(Joint(length=50, current_angle=0, min_angle=-90, max_angle=90, is_static=True, can_y_rotate=True))
al5d.add_joint(Joint(length=145, current_angle=0, min_angle=-30, max_angle=90))
al5d.add_joint(Joint(length=188, current_angle=100, min_angle=0, max_angle=135))
al5d.add_joint(Joint(length=120, current_angle=40, min_angle=-90, max_angle=90))

al5d.plot_side_view()
al5d.plot_top_view()
