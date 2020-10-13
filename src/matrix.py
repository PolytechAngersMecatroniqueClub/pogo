# todo
# not used so far....


class Matrix:
    """ class to handle Matrix without using numpy
    """

    def __init__(self, nb_line, nb_columns):
        """ constructor of the class
        """
        self.values = []
        self.nb_lines = nb_line
        self.nb_columns = nb_columns
        for line in range(nb_line):
            self.values.append([])
            for column in range(nb_columns):
                self.values[line].append(0)

    def __str__(self):
        string = ""
        for line in self.values:
            for value in line:
                string += f"{value} "
            string += "\n"
        return string

    def __setitem__(self, key, value):
        if not isinstance(key, tuple):
            raise IndexError("Key should be a tuple : matrix[li, col]")

        if not (isinstance(value, float) or isinstance(value, int)):
            raise ValueError("Matrix value should be an number")

        li, col = key

        if not (0 <= li < self.nb_lines and 0 <= col < self.nb_columns):
            raise IndexError("Out of range")

        self.values[li][col] = value

    def __getitem__(self, key):
        if not isinstance(key, tuple):
            raise IndexError("Key should be a tuple : matrix[li, col]")

        li, col = key

        if not (0 <= li < self.nb_lines and 0 <= col < self.nb_columns):
            raise IndexError("Out of range")

        return self.values[li][col]

    def trans(self):
        transpose_matrix = Matrix(self.nb_columns, self.nb_lines)
        for li in range(self.nb_lines):
            for col in range(self.nb_columns):
                transpose_matrix[col, li] = self[li, col]
        return transpose_matrix

    def diagonal(self):
        diagonal_matrix = Matrix(self.nb_columns, self.nb_lines)
        for li in range(self.nb_lines):
            for col in range(self.nb_columns):
                if li == col:
                    diagonal_matrix[col, li] = self[li, col]
                else:
                    diagonal_matrix[col, li] = 0
        return diagonal_matrix

    def __mul__(self, value):
        if isinstance(value, float) or isinstance(value, int):
            # multiply by a number
            result = Matrix(self.nb_lines, self.nb_columns)
            for i, li in enumerate(self.values):
                for j, val in enumerate(li):
                    result[i, j] = val * value
            return result
        elif isinstance(value, Matrix):
            # multiply by a matrix
            if self.nb_columns == value.nb_lines:
                result = Matrix(self.nb_lines, value.nb_columns)
                for li_res in range(result.nb_lines):
                    for col_res in range(result.nb_columns):
                        val_res = 0
                        for i in range(self.nb_columns):
                            val_res += (self[li_res, i] * value[i, col_res])
                        result[li_res, col_res] = val_res
                return result
            else:
                raise ValueError("The size of the matrices are not compatibles for the dot product")
        else:
            raise ValueError("Matrix can only be multiply by numbers or matrices")

    def __add__(self, value):
        if isinstance(value, Matrix):
            if self.nb_columns == value.nb_columns and self.nb_lines == value.nb_lines:
                result = Matrix(self.nb_lines, self.nb_columns)
                for li in range(self.nb_lines):
                    for col in range(self.nb_columns):
                        result[li, col] = self[li, col] + value[li, col]
                return result
            else:
                raise ValueError("The matrices must have the same size")
        else:
            raise ValueError("Matrix can only be added to other matrix so far")

    def inverse(self):
        if self.nb_lines == self.nb_columns == 3:
            result = Matrix(3, 3)
            # intermediate value
            a, b, c = self[0, 0], self[0, 1], self[0, 2]
            d, e, f = self[1, 0], self[1, 1], self[1, 2]
            g, h, i = self[2, 0], self[2, 1], self[2, 2]

            determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

            if determinant == 0:
                raise ValueError("The determinant is null")

            result[0, 0] = (e * i - f * h) / determinant
            result[0, 1] = (c * h - b * i) / determinant
            result[0, 2] = (b * f - c * e) / determinant
            result[1, 0] = (f * g - d * i) / determinant
            result[1, 1] = (a * i - c * g) / determinant
            result[1, 2] = (c * d - a * f) / determinant
            result[2, 0] = (d * h - e * g) / determinant
            result[2, 1] = (b * g - a * h) / determinant
            result[2, 2] = (a * e - d * b) / determinant

            return result

        elif self.nb_lines == self.nb_columns == 2:
            result = Matrix(2, 2)
            determinant = self[0, 0] * self[1, 1] - self[0, 1] * self[1, 0]
            if determinant == 0:
                raise ValueError("The determinant is null")
            result[0, 0] = self[1, 1] / determinant
            result[0, 1] = -self[0, 1] / determinant
            result[1, 0] = -self[1, 0] / determinant
            result[1, 1] = self[0, 0] / determinant
        else:
            raise ValueError("Only 2x2 or 3x3 matrices can be inverted so far")

    def scalar_product(self):
        product = self * self.trans()
        return product[0, 0]


class Identity(Matrix):
    """ class to handle Matrix without using numpy
    """

    def __init__(self, dimension):
        """ constructor of the class
        """
        Matrix.__init__(self, dimension, dimension)
        for line in range(dimension):
            for column in range(dimension):
                if line == column:
                    self.values[line][column] = 1
                else:
                    self.values[line][column] = 0
