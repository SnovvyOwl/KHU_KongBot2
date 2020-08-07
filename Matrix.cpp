#include<iostream>
#include<iosfwd>
#include<exception>
#include<vector>
template<typename T>
class Matrix{
    private:
        size_t row;
        size_t col;
        std::vector<std::vector<T>> matrix;
    public:
        Matrix(size_t _row, size_t _col,  std::vector<std::vector<T>>&_mat): //Generater
            row(_row),col(_col),matrix(_mat)
        {
            if((matrix.size()*matrix[0].size())!=(row*col)){
                std::cerr<<"Matrix size error"<<std::endl;
            }
        };
        size_t rows(){
            return row;
        }
        size_t columns(){
            return col;
        }
        const size_t rows() const{
            return row;
        }
        const size_t columns() const{
            return col;
        }
        size_t size(){
            return row*col;//Size of Matrix
        }
        std::vector<T>& operator[](size_t row){
            return matrix[row];
        }
        const std::vector<T>& operator[](size_t row) const{
            return matrix[row];//index Matrix
        }

        Matrix(const Matrix& other) = default;
        Matrix(Matrix&& other) :
        matrix(std::move(other.matrix))
        {
            row = other.rows();
            col = other.columns();
        }
        Matrix& operator=(const Matrix& other) = default;
        Matrix& operator=(Matrix&& other){
            std::swap(matrix, other.matrix);
            row = other.rows();
            col = other.columns();
            return *this;
        }
        Matrix& operator *= (const T& rhs){
            for (auto& row : matrix){
                for (auto& cell : row){
                cell *= rhs;
                }   
            }
            return *this;
        }
        Matrix& operator *= (const Matrix& rhs){
            if (col != rhs.rows()){
                throw std::logic_error("First Matrix's column count and Second Matrix's row count are not equal\n");
            }
            std::vector<std::vector<T>>data;
            data.clear();
            for (size_t i = 0; i < row; i++){
                for (size_t j = 0; j < rhs.columns(); j++){
                    for (size_t k = 0; k < col; k++){
                        data[i][k]=(matrix[i][k] * rhs[j][k]);
                    }
                }
            }
            col=rhs.columns();
            matrix.swap(data);
            return *this;
        }

        Matrix& operator +=(const Matrix& rhs){
            if (row != rhs.rows() || col != rhs.columns()){
            throw std::logic_error("either or both of row count and column count of two matrices are not equal\n");
            }
            for (size_t i = 0; i < row; i++){
                for (size_t j = 0; j < col; j++){
                    matrix[i][j] += rhs[i][j];
                }
            }
            return *this;
        }

        void transpose();// Transpose Matrix
        void inverse();//Inverse Matrix
        void guasselimination();//guass elliminationssss   
};
template <typename T>
bool operator==(const Matrix<T>& lhs, const Matrix<T>& rhs){
    if (lhs.rows() != rhs.rows() || lhs.columns() != rhs.columns()){
        return false;
    }
    for (int i = 0; i < lhs.rows(); i++){
        for (int j = 0; j < lhs.columns(); j++){
            if (lhs[i][j] != rhs[i][j]){
                return false;
            }
        }
    }
    return true;
}

template <typename T>        
bool operator != (const Matrix<T>& lhs, const Matrix<T>& rhs){
    return !(lhs == rhs);
}
template <typename T>
Matrix<T> operator + (Matrix<T> lhs, const Matrix<T>& rhs){   
    return lhs += rhs;
}
template <typename T>
Matrix<T> operator * (Matrix<T> lhs, const Matrix<T>& rhs){
    return lhs *= rhs;
}
template <typename T>
Matrix<T> operator * (Matrix<T> lhs, const T& rhs){
    return lhs *= rhs;
}
template <typename T>
Matrix<T> operator * (const T& lhs, Matrix<T> rhs){
    return rhs *= lhs;
}
/*
template <typename T>
std::istream& operator >> (std::istream& is, Matrix<T>& matrix){
    for (size_t i = 0; i < matrix.rows(); i++){
        for (size_t j = 0; j < matrix.columns(); j++){
            is >> matrix[i][j];
        }
    }
    return is;
}*/
template <typename T>
std::ostream& operator << (std::ostream& os, const Matrix<T>& matrix){
    for (size_t i = 0; i < matrix.rows(); i++){
        for (size_t j = 0; j < matrix.columns(); j++){
            os << matrix[i][j] << ' ';
        }
        os << "\n";
        }   
    return os;
}


int main()
{
    size_t rowCount = 2, columntCount = 2;
    std::vector<std::vector<int>>a={{1 , 0} , {0 ,1}};
    Matrix<int> firstMatrix(rowCount, columntCount, a);

    Matrix<int> secondMatrix(rowCount, columntCount, a);

    std::cout << "Choose operator (+ or *): ";
    char op;
    std::cin >> op;

    switch (op)
    {
    case '*':
        std::cout << firstMatrix * secondMatrix;
    default:
        std::cout << firstMatrix + secondMatrix;
    }

    std::system("pause");

}