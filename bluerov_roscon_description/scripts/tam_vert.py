import numpy as np
import matplotlib.pyplot as plt
import copy

plt.figure()
plt.xlim(-0.3, 0.3)
plt.ylim(-0.3, 0.3)
plt.grid(True)


class Thruster:

    x = 0
    y = 0
    angle = 0
    dir = np.array([0, 0])

    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle
        self.dir = np.array([np.cos(angle), (np.sin(angle))])

    def __array__(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def get_perpendicular(self):
        # find angle between vectors
        beta = np.arccos(
            np.dot(np.array(self), self.dir)
            / (np.linalg.norm(self) * np.linalg.norm(self.dir))
        )
        print(np.rad2deg(beta))
        # force component on the perpedicuar to the vector
        fc = np.cos(np.pi / 2 - beta)
        print(fc)
        dist = np.sqrt(self.x**2 + self.y**2)
        f = fc * dist
        return f

    def draw_perpendicular(self):
        # omega = np.arctan2(th.y, th.x)
        omega = np.arctan(self.y / self.x)
        print(np.rad2deg(omega))

        beta = np.arccos(
            np.dot(np.array(self), self.dir)
            / (np.linalg.norm(self) * np.linalg.norm(self.dir))
        )

        vec = np.array([np.cos(np.pi / 2 - beta), 0])
        vec_rotated = np.array(
            [vec[0] * np.cos(np.pi / 2 + omega), vec[0] * np.sin(np.pi / 2 + omega)]
        )

        print(vec_rotated)

        tc = copy.deepcopy(self)
        # to find then the perpendicular component
        tc.angle = (self.angle / (abs(self.angle))) * (np.pi / 2 - beta) + self.angle

        plot_thruster(tc, "tc" + str(3), "red")


def plot_thruster(thruster: Thruster, name, color):

    dir_x = 0.1 * (np.cos(thruster.angle))
    dir_y = 0.1 * (np.sin(thruster.angle))

    plt.arrow(
        thruster.x,
        thruster.y,
        dir_x,
        dir_y,
        head_width=0.03,
        head_length=0.05,
        fc=color,
        ec=color,
    )

    # line = np.array([0, 0, thruster.x, thruster.y])
    # plt.plot(line, "o")
    plt.plot([0, thruster.x], [0, thruster.y], linestyle="--", color="gray")
    plt.text(thruster.x, thruster.y, name, fontsize=16)


# vertical
t5 = Thruster(0.12861, 0.064159, 1.57)
t6 = Thruster(-0.11161, 0.064159, 1.57)
t7 = Thruster(0.12861, 0.064159, 1.57)
t8 = Thruster(-0.11161, 0.064159, 1.57)

th_list = [t5, t6, t7, t8]

for idx, th in enumerate(th_list):
    plot_thruster(th, "th" + str(idx + 5), "blue")

tt = t5.get_perpendicular()
t5.draw_perpendicular()
tt2 = t6.get_perpendicular()
t6.draw_perpendicular()
tt3 = t7.get_perpendicular()
t7.draw_perpendicular()
tt4 = t8.get_perpendicular()
# plot_thruster(tt4, "tc" + str(4), "red")


TAM = np.array(
    [
        # t1, t2, t3, t4, t5, t6, t7, t8
        [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ],  # x
        [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ],  # y
        [0, 0, 0, 0, 1, 1, 1, 1],  # z
        [0, 0, 0, 0, 0, 0, 0, 0],  # roll
        [
            0,
            0,
            0,
            0,
            t5.get_perpendicular(),
            t6.get_perpendicular(),
            t7.get_perpendicular(),
            t8.get_perpendicular(),
        ],  # pitch
        [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ],  # yaw
    ]
)

# print numpy array with "," separator
np.set_printoptions(precision=4)
print(np.array2string(TAM, separator=","))

# print(TAM)
plt.title("Thruster location")
plt.plot(0, 0, "o")
plt.axis("equal")
plt.show()
