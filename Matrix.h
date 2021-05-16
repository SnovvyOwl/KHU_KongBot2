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

        //Call Private Variable;
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

        //Indexing Matrix
        std::vector<T>& operator[](size_t row){
            return matrix[row];
        }
        const std::vector<T>& operator[](size_t row) const{
            return matrix[row];//index Matrix
        }
        std::vector<std::vector<T>> element(){
            return matrix;
        }
        //Assign;
        Matrix(const Matrix& other) = default;
        Matrix(Matrix&& other) :
        matrix(std::move(other.element()))
        {
            row = other.rows();
            col = other.columns();
        }
        Matrix& operator=(const Matrix& other) = default;
        Matrix& operator=(Matrix&& other){
            std::swap(matrix, other.element());
            row = other.rows();
            col = other.columns();
            return *this;
        }
        //Multiply Matrix
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
            data.resize(row);
            T temp=0;
            for (size_t i = 0; i < row; i++){
                
                for (size_t j = 0; j < rhs.columns(); j++){
                    temp=0;
                    for (size_t k = 0; k < col; k++){
                        temp+=matrix[i][k] * rhs[k][j];
                        //std::cout<<matrix[i][k]<<"*"<<rhs[k][j]<<std::endl;
                        
                    }
                    data[i].push_back(temp);
                }
            }
            col=rhs.columns();
            matrix.swap(data);
            return *this;
        }
        //ADD Matrix
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
        //subs
        Matrix& operator -=(const Matrix& rhs){
            if (row != rhs.rows() || col != rhs.columns()){
            throw std::logic_error("either or both of row count and column count of two matrices are not equal\n");
            }
            for (size_t i = 0; i < row; i++){
                for (size_t j = 0; j < col; j++){
                    matrix[i][j] -= rhs[i][j];
                }
            }
            return *this;
        }
        // Transpose Matrix
        Matrix& transpose(){
            std::vector<std::vector<T>>data;
            data.resize(col);
            for(int j=0;j<col;j++){
                for(int i=0;i<row;i++){
                    data[j].push_back(matrix[i][j]);
                }
            }
            size_t temp;
            temp=row;
            row=col;
            col=temp; 
            std::swap(matrix,data);
            return *this;
        }
        Matrix Gaussian_Elimination(Matrix& RHS){//guass elliminationssss 
            if (matrix[0][0]==0){
                throw std::logic_error("Matrix[0][0]==0");
            }
            else if(row!=col){
                throw std::logic_error("This matrix is NOT SQUARE MATRIX");
            }
            else{
                T tmp;
                for(int i=0;i<row;i++){
                    for(int j=i+1;j<row;j++){
                        tmp=-1*matrix[j][i]/matrix[i][i];
                        for (int k=0;k<row;k++){
                            matrix[j][k]+=tmp*matrix[i][k];
                        }
                        for(int m=0;m<RHS.columns();m++){
                            RHS[j][m]+=tmp*RHS[j][m];
                        }
                    }
                }
                //this->back_subsititution(RHS);   
                return *this;
            }        
        }

        Matrix back_subsititution(Matrix& RHS){
            T tmp=0;
            if(matrix[row-1][col-1]==0){
                throw std::logic_error("Matrix[row][col]==0");
            }
            else{
                 for(int i=row-1;i>=0;i--){
                    tmp=1/matrix[i][i];
                    matrix[i][i]=1;
                    for(int n = 0;n < RHS.columns();n++){
                        RHS[i][n]*=tmp;
                    }
                    for(int j=0;j<i;j++){
                        tmp=-1*matrix[j][i];
                        for (int k=0;k<row;k++){
                            matrix[j][k]+=tmp*matrix[i][k];
                        }
                        for(int m=0;m< RHS.columns();m++){
                            RHS[j][m]+=tmp*RHS[i][m];
                        }
                    }
                }
                matrix=RHS.element();
                row=RHS.rows();
                col=RHS.columns();
                return *this;
            }
        }
        Matrix inv(){
            std::vector<std::vector<T>>e;
            e.resize(row);
            e[0].resize(col);
            for(int i=0;i<row;i++){
                e[i].resize(col);
                e[i][i]=1;
            }
            Matrix<T>E(row,col,e);
            this->Gaussian_Elimination(E);
            return *this;
        }
    };

// correct
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
//  NOT
template <typename T>        
bool operator != (const Matrix<T>& lhs, const Matrix<T>& rhs){
    return !(lhs == rhs);
}
// Add Matrix
template <typename T>
Matrix<T> operator + (Matrix<T> lhs, const Matrix<T>& rhs){   
    return lhs += rhs;
}
template <typename T>
Matrix<T> operator - (Matrix<T> lhs, const Matrix<T>& rhs){   
    return lhs -= rhs;
}
//Multiply Matrix
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
template <typename T> //print Matrix
std::ostream& operator << (std::ostream& os, const Matrix<T>& matrix){
    for (size_t i = 0; i < matrix.rows(); i++){
        for (size_t j = 0; j < matrix.columns(); j++){
            os << matrix[i][j] << ' ';
        }
        os << "\n";
        }   
    return os;
}