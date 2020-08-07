#include<iostream>
#include<vector>
template<typename T>
class Matrix{
    private:
        size_t row:
        size_t col;
        std::vector<T>matrix;
    public:
        Matrix(size_t _row,size_t _col,  std::vector<T>&_mat): //Generater
            row(_row),col(_col),matrix(_mat)
        {
            if(matrix.size()!=(row*col){
                std::cerr<<"Matrix size error"<<endl;
                ~Matrix();
            }
        }
        ~Matrix();//delete
        size_t size(){
            return row*col;//Size of Matrix
        }
        std::vector<T>& operator[](size_t row){
            return matrix[row];
        }
        const  std::vector<T>& operator[](size_t row) const{
            return matrix[row];//index Matrix
        }
        Matrix& operator=(Matrix&& other){
            std::swap(matrix, other.matrix);
            rowCount = other.rowCount;
            columnCount = other.columnCount;
            return *this;
        }

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
        Matrix& operator*=(const T& rhs){
            for (auto& row : matrix){
                for (auto& cell : row){
                cell *= rhs;
                }   
            }
            return *this;
        }
        Matrix& operator*=(const Matrix& rhs){
            if (columnCount != rhs.rowCount){
                throw std::logic_error("First Matrix's column count and Second Matrix's row count are not equal\n");
            }
            Matrix temp(rowCount, rhs.columnCount);
                for (size_t i = 0; i < temp.rowCount; i++){
            for (size_t j = 0; j < temp.columnCount; j++)
            {
                for (size_t k = 0; k < columnCount; k++)
                {
                    temp[i][j] += matrix[i][k] * rhs[j][k];
                }
            }
        }
        std::swap(matrix, temp.matrix);

        return *this;
    }

    Matrix& operator+=(const Matrix& rhs)
    {
        if (rowCount != rhs.rowCount || columnCount != rhs.columnCount)
        {
            throw std::logic_error("either or both of row count and column count of lhs and rhs are not equal\n");
        }

        for (size_t i = 0; i < rowCount; i++)
        {
            for (size_t j = 0; j < columnCount; j++)
            {
                matrix[i][j] += rhs[i][j];
            }
        }

        return *this;
    }
        bool operator!=(const Matrix<T>& lhs, const Matrix<T>& rhs){
            return !(lhs == rhs);
        }
        Matrix<T> operator+(Matrix<T> lhs, const Matrix<T>& rhs){
            return lhs += rhs;
        }
        Matrix<T> operator*(Matrix<T> lhs, const Matrix<T>& rhs){
            return lhs *= rhs;
        }
        Matrix<T> operator*(Matrix<T> lhs, const T& rhs){
            return lhs *= rhs;
        }
        Matrix<T> operator*(const T& lhs, Matrix<T> rhs){
            return rhs *= lhs;
        }        
        bool operator!=(const Matrix<T>& lhs, const Matrix<T>& rhs){
            return !(lhs == rhs);
        }
        Matrix<T> operator+(Matrix<T> lhs, const Matrix<T>& rhs){   
            return lhs += rhs;
        }
        Matrix<T> operator*(Matrix<T> lhs, const Matrix<T>& rhs){
            return lhs *= rhs;
        }
        Matrix<T> operator*(Matrix<T> lhs, const T& rhs){
            return lhs *= rhs;
        }
        Matrix<T> operator*(const T& lhs, Matrix<T> rhs){
            return rhs *= lhs;
}
        std::istream& operator >> (std::istream& is, Matrix<T>& matrix){
            for (size_t i = 0; i < matrix.rows(); i++){
                for (size_t j = 0; j < matrix.columns(); j++){
                    is >> matrix[i][j];
                }
            }
            return is;
        }
        std::ostream& operator << (std::ostream& os, const Matrix<T>& matrix){
            for (size_t i = 0; i < matrix.rows(); i++){
                for (size_t j = 0; j < matrix.columns(); j++){
                    os << matrix[i][j] << ' ';
                }
                os << "\n";
            }   
            return os;
        }
        void transpose();// Transpose Matrix
        void inverse();//Inverse Matrix
        void guasselimination();//guass elliminationssss   
};