#!/usr/bin/env python3
from math import sqrt
from itertools import chain

class Thruster:
    def __init__(self, position, thrust_components, thrust=1):
        '''
        'position' is an x,y,z tuple where x is right, y is forward, z is up.
           Origin is vehicle's center of mass.
        'thrust_components' determines the orientation of the thruster, by the
            ratios between the provided x,y,z values.
            (e.g. a forward thruster could be (0,1,0),
                  a vertically upwards thruster could be (0,0,1),
                  equally in every direction coud be (1,1,1))
        'thrust' is the total thrust capacity of this thruster.
        TODO: add propeller spin direction property (1/-1?)
        '''
        self.lateral, self.forward, self.throttle = \
            self.normalise(thrust_components)
        self.throttle *= -1 # throttle should be down, not up
        # internally use aeronautical axes: x forward, y right, z down
        self._y, self._x, self._z = self.position = position
        self._z *= -1
        self.thrust = thrust
        self.calculate_thrust_factors()

    def calculate_thrust_factors(self):
        self.roll = -self.lateral * self._z - self.throttle * self._y
        self.pitch = self.throttle * self._x + self.forward * self._z
        self.yaw = self.lateral * self._x - self.forward * self._y

    @staticmethod
    def normalise(values):
        norm = sqrt(sum(v**2 for v in values))
        return (v / norm for v in values)

    def mirror(self, normal, offset=(0,0,0)):
        '''

        'normal' is an x,y,z tuple representing the normal of the plane to
            mirror over. e.g. (1,0,0) to mirror over the x-y plane.
        'offset' is the (optional) offset point to shift the mirror plane by.

        '''
        normal = list(self.normalise(normal))
        def dot(vec, normal):
            return sum(v * n for v, n in zip(vec, normal))

        position = [p - o for p, o in zip(self.position, offset)]
        p_dot_n = dot(position, normal)
        position = [p - 2 * p_dot_n * n + o
                    for p, n, o in zip(position, normal, offset)]

        thrust_components = self.thrust_components
        t_dot_n = dot(thrust_components, normal)
        thrust_components = [t - 2 * t_dot_n * n
                             for t, n in zip(thrust_components, normal)]

        return self.__class__(position, thrust_components, self.thrust)

    @property
    def factors(self):
        return (self.roll, self.pitch, self.yaw,
                self.throttle, self.forward, self.lateral)

    @property
    def thrust_components(self):
        return self.lateral, self.forward, -self.throttle

    def __repr__(self):
        position = self.position
        thrust_components = self.thrust_components
        thrust = self.thrust
        return (f'{self.__class__.__name__}({position=}, '
                f'{thrust_components=}, {thrust=})')

    def __str__(self):
        thrust = self.thrust
        position = self.position
        roll, pitch, yaw, throttle, forward, lateral = self.factors
        return '\n'.join(f'Thruster ({thrust=}):',
                         f'  {position = }',
                         '  Factors:',
                         f'  {roll=}, {pitch=}, {yaw=}, ' # continue same line
                         f'  {throttle=}, {forward=}, {lateral=}')

    def __eq__(self, other):
        return (isinstance(other, self.__class__)
                and self.thrust == other.thrust
                and self.position == other.position
                and self.factors == other.factors)

    def __hash__(self):
        return hash((self.thrust, self.position, self.factors))

    def plot(self, ax=None):
        ... # TODO


class ThrusterGroup:
    def __init__(self, *thrusters):
        self.thrusters = list(thrusters)
        self._modified = True

    def mirror(self, normal, offset=(0,0,0)):
        return self.__class__(*(t.mirror(normal, offset)
                                for t in self.thrusters))

    def mirror_complete(self, normal, offset=(0,0,0)):
        return self.__class__(*chain(*((t, t.mirror(normal, offset))
                                       for t in self.thrusters)))

    @classmethod
    def with_symmetry(cls, thruster, mirror_normals, mode='sequential'):
        group = thruster if isinstance(thruster, cls) else cls(thruster)
        if mode == 'sequential':
            for normal in mirror_normals:
                group = group.mirror_complete(normal)
        elif mode == 'separate':
            for normal in mirror_normals:
                group += cls(thruster.mirror(normal))
        return group

    @classmethod
    def with_basic_symmetry(cls, thruster, front_back=True, left_right=True,
                            up_down=False):
        normals = []
        if front_back:
            normals.append((0,1,0))
        if left_right:
            normals.append((1,0,0))
        if up_down:
            normals.append((0,0,1))
        return cls.with_symmetry(thruster, normals)

    def __iadd__(self, other):
        self.thrusters += other.thrusters
        self._modified = True

    def __add__(self, other):
        if isinstance(other, self.__class__):
            return self.__class__(*self.thrusters, *other.thrusters)
        else:
            return self.__class__(*self.thrusters, other)

    def __radd__(self, other):
        return self.__class__(other, *self.thrusters)

    def __repr__(self):
        class_name = self.__class__.__name__
        sep = ',\n' + ' '*(len(class_name) + 1)
        thrusters = sep.join(repr(t) for t in self.thrusters)
        return f'{class_name}({thrusters})'

    def __str__(self):
        ''' return table with rows of (id, thrust, *position, *factors) '''
        factor_table_rows = self.factor_table.split('\n')
        id, factor_table_columns = (factor_table_rows[0]
                                    .lstrip().split(maxsplit=1))
        correction = '{{:>{}}}'.format(len(factor_table_columns) + 4)

        columns = (id, 'thrust', 'pos_x', 'pos_y', 'pos_z')
        header, template = self.table_columns(columns)
        header += correction.format(factor_table_columns)

        output = [header]
        for row, thruster in zip(factor_table_rows[1:], self.thrusters):
            id, factors = row.lstrip().split(maxsplit=1)
            output.append(template.format(id, thruster.thrust,
                                          *thruster.position) +
                          correction.format(factors))

        return '\n'.join(output)
        
    @staticmethod
    def table_columns(columns):
        l = max(len(c) for c in columns) + 1
        pad = '{{:>{}}}'.format(l)
        first = '{{:>{}}}'.format(len(columns[0])+1)
        header = (first + pad * (len(columns) - 1)).format(*columns)
        template = first + '{{:{}.2f}}'.format(l) * (len(columns) - 1)
        return header, template

    @property
    def factor_table(self):
        if not self._modified:
            return self._factor_table

        columns = ('id',
                   'roll', 'pitch', 'yaw',
                   'throttle', 'forward', 'lateral')
        header, template = self.table_columns(columns)

        data = []
        maxes = [0] * 6
        for thruster in self.thrusters:
            factors = []
            for index, f in enumerate(thruster.factors):
                factors.append(f)
                maxes[index] = max(maxes[index], abs(f))
            data.append(factors)

        output = [header]
        for id, row in enumerate(data, start=1):
            for index, element in enumerate(row):
                divisor = maxes[index] or 1 # handle full zero column case
                row[index] /= divisor
            output.append(template.format(id, *row))

        self._factor_data = data
        self._factor_table = '\n'.join(output)

        return self._factor_table

    @property
    def factor_data(self):
        self.factor_table # ensure data available
        return self._factor_data

    def __eq__(self, other):
        return (isinstance(other, self.__class__)
                and set(self.thrusters) == set(other.thrusters))

    def __contains__(self, thruster):
        return thruster in self.thrusters

    def plot(self, ax=None):
        ... # TODO


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        frame = sys.argv[1]
        if frame == 'BlueROV2':
            thrusters = ThrusterGroup.with_basic_symmetry(
                ThrusterGroup.with_basic_symmetry(
                    Thruster((1,2,0), (1,-1,0)), left_right=False
                ) + Thruster((1,0,0), (0,0,1)), front_back=False
            )
        elif frame == 'BlueROV2-Heavy':
            thrusters = ThrusterGroup.with_basic_symmetry(
                ThrusterGroup(
                    Thruster((0.5, 1.5, 0), (1,-1,0)),
                    Thruster((1.5, 0.5, 0), (0,0,1))
                )
            )
        elif frame == 'Fully-Vectored':
            thrusters = ThrusterGroup.with_basic_symmetry(
                Thruster((2, 3, 1), (1, -1, 1)),
                up_down=True
            )
        elif frame == 'Fully-Vectored-all-top-right':
            thrusters = ThrusterGroup(
                *(Thruster((2,3,1), [((x & 2**i) >> i) * 2 - 1 for i in range(3)])
                  for x in range(8))
            )
        else:
            raise Exception(f'Invalid frame {frame=}')
    else:
        thrusters = ThrusterGroup.with_basic_symmetry(
            Thruster((2, 3, 1), (1, -1, 1)),
            up_down=True
        )

    print(thrusters)
