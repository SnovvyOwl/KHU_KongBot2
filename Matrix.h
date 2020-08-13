#pragma once
#include<iostream>
#include<iosfwd>
#include<exception>
#include<vector>
template<typename T>
class Matrix {
	private:
		size_t row;
		size_t col;
		std::vector<std::vector<T>> matrix;
	public:
		Matrix(size_t _row, size_t _col, std::vector<std::vector<T>>&_mat) : //Generater
		row(_row), col(_col), matrix(_mat)
		{
			if ((matrix.size()*matrix[0].size()) != (row*col)) {
				std::cerr << "Matrix size error" << std::endl;
			}
		};

		//Call Private Variable;
		size_t rows() {
			return row;
		}
		size_t columns() {
			return col;
		}
		const size_t rows() const {
			return row;
		}
		const size_t columns() const {
			return col;
		}
		size_t size() {
			return row * col;//Size of Matrix
		}

		//Indexing Matrix
		std::vector<T>& operator[](size_t row) {
			return matrix[row];
		}
		const std::vector<T>& operator[](size_t row) const {
			return matrix[row];//index Matrix
		}
		std::vector<std::vector<T>> element() {
			return matrix;
		}
		//Assign Matrix
		Matrix(const Matrix& other) = default;
		Matrix(Matrix&& other) :
		matrix(std::move(other.element()))
		{
			row = other.rows();
			col = other.columns();
		}
		Matrix& operator=(const Matrix& other) = default;
		Matrix& operator=(Matrix&& other) {
			std::swap(matrix, other.element());
			row = other.rows();
			col = other.columns();
			return *this;
		}
		//Multiply Matrix
		Matrix& operator *= (const T& rhs) {
			for (auto& row : matrix) {
				for (auto& cell : row) {
					cell *= rhs;
				}
			}
			return *this;
		}

		Matrix& operator *= (const Matrix& rhs) {
			if (col != rhs.rows()) {
				throw std::logic_error("First Matrix's column count and Second Matrix's row count are not equal\n");
			}
			std::vector<std::vector<T>>data;
			data.resize(row);
			T temp = 0;
			for (size_t i = 0; i < row; i++) {
				for (size_t j = 0; j < rhs.columns(); j++) {
					temp = 0;
					for (size_t k = 0; k < col; k++) {
						temp += matrix[i][k] * rhs[k][j];
						//std::cout<<matrix[i][k]<<"*"<<rhs[k][j]<<std::endl;
					}
					data[i].push_back(temp);
				}
			}
			col = rhs.columns();
			matrix.swap(data);
			return *this;
		}
		//ADD Matrix
		Matrix& operator +=(const Matrix& rhs) {
			if (row != rhs.rows() || col != rhs.columns()) {
				throw std::logic_error("either or both of row count and column count of two matrices are not equal\n");
			}
			for (size_t i = 0; i < row; i++) {
				for (size_t j = 0; j < col; j++) {
					matrix[i][j] += rhs[i][j];
				}
			}
			return *this;
		}
		//SUB Matrix
		Matrix& operator -=(const Matrix& rhs) {
			if (row != rhs.rows() || col != rhs.columns()) {
				throw std::logic_error("either or both of row count and column count of two matrices are not equal\n");
			}
			for (size_t i = 0; i < row; i++) {
				for (size_t j = 0; j < col; j++) {
					matrix[i][j] -= rhs[i][j];
				}
			}
			return *this;
		}
		// Transpose Matrix
		Matrix& transpose() {
			std::vector<std::vector<T>>data;
			data.resize(col);
			for (size_t j = 0; j < col; j++) {
				for (size_t i = 0; i < row; i++) {
					data[j].push_back(matrix[i][j]);
				}
			}
			size_t temp;
			temp = row;
			row = col;
			col = temp;
			std::swap(matrix, data);
			return *this;
		}
		//Gaussian_Elimination
		Matrix Gaussian_Elimination(Matrix& rhs) {
			if (matrix[0][0] == 0) {
				throw std::logic_error("Matrix[0][0]==0");
			}
			else if (row != col) {
				throw std::logic_error("This matrix is NOT SQUARE MATRIX");
			}
			else {
				T tmp;
				size_t R_col = rhs.columns();
				for (size_t i = 0; i < row; i++) {
					for (size_t j = i + 1; j < row; j++) {
						tmp = -1 * matrix[j][i] / matrix[i][i];
						for (size_t k = 0; k < row; k++) {
							matrix[j][k] += tmp * matrix[i][k];
						}
						for (size_t m = 0; m < R_col; m++) {
							rhs[j][m] += tmp * rhs[i][m];
						}
					}
				}
				this->back_subsititution(rhs);
				return *this;
			}
		}
		//Back_subsititution
		Matrix back_subsititution(Matrix& rhs) {
			T tmp = 0;
			if (matrix[row - 1][col - 1] == 0) {
				throw std::logic_error("Matrix[row][col]==0");
			}
			else {
				size_t R_col = rhs.columns();
				for (int i = row - 1; i >= 0; i--) {
					tmp = 1 / matrix[i][i];
					matrix[i][i] = 1;
					for (size_t n = 0; n < R_col; n++) {
						rhs[i][n] *= tmp;
					}
					for (size_t j = 0; j < i; j++) {
						tmp = -1 * matrix[j][i];
						for (size_t k = 0; k < row; k++) {
							matrix[j][k] += tmp * matrix[i][k];
						}
						for (size_t m = 0; m < R_col; m++) {
							rhs[j][m] += tmp * rhs[i][m];
						}
					}
				}
				matrix = rhs.element();
				row = rhs.rows();
				col = rhs.columns();
				return *this;
			}
		}
		//inverse Matrix
		Matrix inv() { 
			std::vector<std::vector<T>>e;
			e.resize(row);
			e[0].resize(col);
			//Unit Matrix
			for (int i = 0; i < row; i++) {
				e[i].resize(col);
				e[i][i] = 1;
			}
			Matrix<T>E(row, col, e);
			this->Gaussian_Elimination(E);
			return *this;
		}
};