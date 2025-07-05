class Solution {
public:

    bool isValid(vector<vector<char>>& board, int row, int col, char ans){
        // we have to check row col and the box 3x3
        for(int i = 0; i< 9; i++){
            if(board[row][i] == ans)
                return false;
            if(board[i][col] == ans )
                return false;
            
            // row position 
            int r = (3*((row/3))) + i/3;
            int c = (3*((col/3))) + (i%3);
            if(board[r][c] == ans)
                return false;
        }
        return true;
    }


    bool solve(vector<vector<char>>& board){
        for(int i = 0; i < 9 ; i++){
            for(int j = 0; j < 9 ; j++){
                if(board[i][j] == '.'){
                    // now we fill it 0-->9
                    for(char ans = '1'; ans  <= '9'; ans++){
                        if(isValid(board, i, j, ans)){
                            board[i][j] = char(ans);

                            // now for this ans we fill the soduku completely and if that is correct solution  
                            // then return zero else we have to assign . agian 
                            if(solve(board) == true)
                                return true;
                            else   
                                board[i][j] = '.';
                        }
                    }
                    return false;
                }
            }
        }

        return true;
    }
    void solveSudoku(vector<vector<char>>& board) {
        solve(board);
    }
};
