#include <iostream>
#include <array>
#include <vector>
#include <queue>
#include <unordered_set>
#include <string>
#include <functional>

using namespace std;
using CubeState = array<int, 54>;

// Cube layout: Front(0-8), Back(9-17), Up(18-26), Down(27-35), Left(36-44), Right(45-53)

struct Node {
    CubeState cube;
    int g, h;
    string path;

    Node(const CubeState& c, int g_, int h_, const string& p) 
        : cube(c), g(g_), h(h_), path(p) {}

    int f() const { return g + h; }
    bool operator<(const Node& other) const { return f() > other.f(); }
};

int heuristic(const CubeState& current, const CubeState& goal) {
    int mismatch = 0;
    for (int i = 0; i < 54; i++) {
        if (current[i] != goal[i]) mismatch++;
    }
    return mismatch;
}

namespace std {
    template<> struct hash<CubeState> {
        size_t operator()(const CubeState& arr) const {
            size_t seed = 0;
            for (int i : arr) {
                seed ^= hash<int>{}(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
}

// Helper to rotate face clockwise
void rotateFaceClockwise(CubeState& cube, int offset) {
    int temp = cube[offset];
    cube[offset] = cube[offset + 6];
    cube[offset + 6] = cube[offset + 8];
    cube[offset + 8] = cube[offset + 2];
    cube[offset + 2] = temp;

    temp = cube[offset + 1];
    cube[offset + 1] = cube[offset + 3];
    cube[offset + 3] = cube[offset + 7];
    cube[offset + 7] = cube[offset + 5];
    cube[offset + 5] = temp;
}

// Helper to cycle 4 positions
void cycle4(CubeState& cube, int a, int b, int c, int d) {
    int temp = cube[a];
    cube[a] = cube[b];
    cube[b] = cube[c];
    cube[c] = cube[d];
    cube[d] = temp;
}

// All 6 face moves
CubeState moveF(const CubeState& cube) {
    CubeState newCube = cube;
    // Rotate front face clockwise
    newCube[0] = cube[6];
    newCube[1] = cube[3];
    newCube[2] = cube[0];
    newCube[3] = cube[7];
    newCube[4] = cube[4];
    newCube[5] = cube[1];
    newCube[6] = cube[8];
    newCube[7] = cube[5];
    newCube[8] = cube[2];

    // Save U6, U7, U8
    int t0 = cube[24], t1 = cube[25], t2 = cube[26];
    // U6, U7, U8 -> L8, L5, L2
    newCube[44] = t0;
    newCube[41] = t1;
    newCube[38] = t2;
    // L8, L5, L2 -> D2, D1, D0
    newCube[20] = cube[44];
    newCube[19] = cube[41];
    newCube[18] = cube[38];
    // D2, D1, D0 -> R0, R3, R6
    newCube[45] = cube[20];
    newCube[48] = cube[19];
    newCube[51] = cube[18];
    // R0, R3, R6 -> U6, U7, U8
    newCube[24] = cube[51];
    newCube[25] = cube[48];
    newCube[26] = cube[45];

    return newCube;
}

CubeState moveB(const CubeState& cube) {
    CubeState newCube = cube;
    // Rotate back face clockwise
    newCube[9] = cube[15];
    newCube[10] = cube[12];
    newCube[11] = cube[9];
    newCube[12] = cube[16];
    newCube[13] = cube[13];
    newCube[14] = cube[10];
    newCube[15] = cube[17];
    newCube[16] = cube[14];
    newCube[17] = cube[11];

    // Save U0, U1, U2
    int t0 = cube[18], t1 = cube[19], t2 = cube[20];
    // U0, U1, U2 -> R2, R5, R8
    newCube[47] = t0;
    newCube[50] = t1;
    newCube[53] = t2;
    // R2, R5, R8 -> D8, D7, D6
    newCube[35] = cube[47];
    newCube[34] = cube[50];
    newCube[33] = cube[53];
    // D8, D7, D6 -> L6, L3, L0
    newCube[36] = cube[35];
    newCube[39] = cube[34];
    newCube[42] = cube[33];
    // L6, L3, L0 -> U0, U1, U2
    newCube[18] = cube[36];
    newCube[19] = cube[39];
    newCube[20] = cube[42];

    return newCube;
}

CubeState moveU(const CubeState& cube) {
    CubeState result = cube;
    rotateFaceClockwise(result, 18);  // Up face
    cycle4(result, 0, 9, 45, 36);
    cycle4(result, 1, 10, 46, 37);
    cycle4(result, 2, 11, 47, 38);
    return result;
}

CubeState moveD(const CubeState& cube) {
    CubeState result = cube;
    rotateFaceClockwise(result, 27);  // Down face
    cycle4(result, 6, 42, 15, 51);
    cycle4(result, 7, 43, 16, 52);
    cycle4(result, 8, 44, 17, 53);
    return result;
}

CubeState moveL(const CubeState& cube) {
    CubeState result = cube;
    rotateFaceClockwise(result, 36);  // Left face
    cycle4(result, 0, 18, 27, 17);
    cycle4(result, 3, 21, 30, 14);
    cycle4(result, 6, 24, 33, 11);
    return result;
}

CubeState moveR(const CubeState& cube) {
    CubeState result = cube;
    rotateFaceClockwise(result, 45);  // Right face
    cycle4(result, 2, 29, 9, 20);
    cycle4(result, 5, 32, 12, 23);
    cycle4(result, 8, 35, 15, 26);
    return result;
}

// Inverse moves (apply 3 times)
CubeState moveFp(const CubeState& cube) {
    CubeState newCube = cube;
    // Rotate front face counter-clockwise
    newCube[0] = cube[2];
    newCube[1] = cube[5];
    newCube[2] = cube[8];
    newCube[3] = cube[1];
    newCube[4] = cube[4];
    newCube[5] = cube[7];
    newCube[6] = cube[0];
    newCube[7] = cube[3];
    newCube[8] = cube[6];

    // Save U6, U7, U8
    int t0 = cube[24], t1 = cube[25], t2 = cube[26];
    // U6, U7, U8 -> R0, R3, R6
    newCube[45] = t2;
    newCube[48] = t1;
    newCube[51] = t0;
    // R0, R3, R6 -> D2, D1, D0
    newCube[20] = cube[45];
    newCube[19] = cube[48];
    newCube[18] = cube[51];
    // D2, D1, D0 -> L8, L5, L2
    newCube[44] = cube[20];
    newCube[41] = cube[19];
    newCube[38] = cube[18];
    // L8, L5, L2 -> U6, U7, U8
    newCube[24] = cube[38];
    newCube[25] = cube[41];
    newCube[26] = cube[44];

    return newCube;
}

CubeState moveBp(const CubeState& cube) {
    CubeState newCube = cube;
    // Rotate back face counter-clockwise
    newCube[9] = cube[11];
    newCube[10] = cube[14];
    newCube[11] = cube[17];
    newCube[12] = cube[10];
    newCube[13] = cube[13];
    newCube[14] = cube[16];
    newCube[15] = cube[9];
    newCube[16] = cube[12];
    newCube[17] = cube[15];

    // Save U0, U1, U2
    int t0 = cube[18], t1 = cube[19], t2 = cube[20];
    // U0, U1, U2 -> L6, L3, L0
    newCube[36] = t0;
    newCube[39] = t1;
    newCube[42] = t2;
    // L6, L3, L0 -> D8, D7, D6
    newCube[35] = cube[36];
    newCube[34] = cube[39];
    newCube[33] = cube[42];
    // D8, D7, D6 -> R2, R5, R8
    newCube[47] = cube[35];
    newCube[50] = cube[34];
    newCube[53] = cube[33];
    // R2, R5, R8 -> U0, U1, U2
    newCube[18] = cube[47];
    newCube[19] = cube[50];
    newCube[20] = cube[53];

    return newCube;
}

CubeState moveUp(const CubeState& cube) {
    CubeState result = moveU(cube);
    result = moveU(result);
    return moveU(result);
}

CubeState moveDp(const CubeState& cube) {
    CubeState result = moveD(cube);
    result = moveD(result);
    return moveD(result);
}

CubeState moveLp(const CubeState& cube) {
    CubeState result = moveL(cube);
    result = moveL(result);
    return moveL(result);
}

CubeState moveRp(const CubeState& cube) {
    CubeState result = moveR(cube);
    result = moveR(result);
    return moveR(result);
}

CubeState moveCube(const CubeState& cube, const string& move) {
    if (move == "F") return moveF(cube);
    if (move == "F'") return moveFp(cube);
    if (move == "B") return moveB(cube);
    if (move == "B'") return moveBp(cube);
    if (move == "U") return moveU(cube);
    if (move == "U'") return moveUp(cube);
    if (move == "D") return moveD(cube);
    if (move == "D'") return moveDp(cube);
    if (move == "L") return moveL(cube);
    if (move == "L'") return moveLp(cube);
    if (move == "R") return moveR(cube);
    if (move == "R'") return moveRp(cube);
    return cube;
}

string AStarSolver(const CubeState& start, const CubeState& goal) {
    if (start == goal) return "Already solved";

    priority_queue<Node> open;
    unordered_set<CubeState> closed;

    open.push({start, 0, heuristic(start, goal), ""});

    vector<string> moves = {"F", "F'", "B", "B'", "U", "U'", "D", "D'", "L", "L'", "R", "R'"};
    const int MAX_DEPTH = 12;  // Increased depth limit

    int iterations = 0;
    const int MAX_ITERATIONS = 10000000;  // Prevent infinite loops

    while (!open.empty() && iterations < MAX_ITERATIONS) {
        iterations++;

        if (iterations % 10000 == 0) {
            cout << "Searched " << iterations << " states, queue size: " << open.size() << endl;
        }

        Node current = open.top();
        open.pop();

        if (current.cube == goal) {
            cout << "Found solution after " << iterations << " iterations!" << endl;
            return current.path;
        }

        if (current.g >= MAX_DEPTH) continue;
        if (closed.count(current.cube)) continue;
        closed.insert(current.cube);

        for (const string& move : moves) {
            // Enhanced move pruning
            if (!current.path.empty()) {
                size_t lastSpace = current.path.find_last_of(' ');
                string lastMove = (lastSpace == string::npos) ? 
                    current.path : current.path.substr(lastSpace + 1);

                // Skip opposite moves
                if ((move == "F" && lastMove == "F'") || (move == "F'" && lastMove == "F") ||
                    (move == "B" && lastMove == "B'") || (move == "B'" && lastMove == "B") ||
                    (move == "U" && lastMove == "U'") || (move == "U'" && lastMove == "U") ||
                    (move == "D" && lastMove == "D'") || (move == "D'" && lastMove == "D") ||
                    (move == "L" && lastMove == "L'") || (move == "L'" && lastMove == "L") ||
                    (move == "R" && lastMove == "R'") || (move == "R'" && lastMove == "R")) {
                    continue;
                }
            }

            CubeState nextCube = moveCube(current.cube, move);
            if (closed.count(nextCube)) continue;

            string newPath = current.path.empty() ? 
                move : current.path + " " + move;

            open.push({nextCube, current.g + 1, 
                      heuristic(nextCube, goal), newPath});
        }
    }

    if (iterations >= MAX_ITERATIONS) {
        return "Search limit reached. Try increasing MAX_ITERATIONS.";
    } else {
        return "No solution found within depth limit.";
    }
}

#include <limits>

// Helper for IDA* search
bool idaSearch(const CubeState& node, const CubeState& goal, int g, int threshold, string& path, unordered_set<CubeState>& visited, string& result, const vector<string>& moves, int maxDepth) {
    int h = heuristic(node, goal);
    int f = g + h;
    if (f > threshold) return false;
    if (g > maxDepth) return false; // Prevent infinite recursion
    if (node == goal) {
        result = path;
        return true;
    }
    visited.insert(node);
    for (const string& move : moves) {
        // Prune consecutive inverses
        if (!path.empty()) {
            size_t lastSpace = path.find_last_of(' ');
            string lastMove = (lastSpace == string::npos) ? path : path.substr(lastSpace + 1);
            if ((move == "F" && lastMove == "F'") || (move == "F'" && lastMove == "F") ||
                (move == "B" && lastMove == "B'") || (move == "B'" && lastMove == "B") ||
                (move == "U" && lastMove == "U'") || (move == "U'" && lastMove == "U") ||
                (move == "D" && lastMove == "D'") || (move == "D'" && lastMove == "D") ||
                (move == "L" && lastMove == "L'") || (move == "L'" && lastMove == "L") ||
                (move == "R" && lastMove == "R'") || (move == "R'" && lastMove == "R")) {
                continue;
            }
        }
        CubeState next = moveCube(node, move);
        if (visited.count(next)) continue;
        string newPath = path.empty() ? move : path + " " + move;
        if (idaSearch(next, goal, g + 1, threshold, newPath, visited, result, moves, maxDepth)) return true;
    }
    visited.erase(node);
    return false;
}

string IDAStarSolver(const CubeState& start, const CubeState& goal, int maxDepth = 12) {
    if (start == goal) return "Already solved";
    vector<string> moves = {"F", "F'", "B", "B'", "U", "U'", "D", "D'", "L", "L'", "R", "R'"};
    int threshold = heuristic(start, goal);
    string result;
    for (int depth = threshold; depth <= maxDepth; ++depth) {
        unordered_set<CubeState> visited;
        string path;
        if (idaSearch(start, goal, 0, depth, path, visited, result, moves, maxDepth)) {
            return result;
        }
    }
    return "No solution found within depth limit.";
}


int main() {
    // CubeState start, goal;

    // cout << "Enter start cube (54 values):" << endl;
    // for (int i = 0; i < 54; i++) {
    //     cin >> start[i];
    // }

    // cout << "Enter goal cube (54 values):" << endl;
    // for (int i = 0; i < 54; i++) {
    //     cin >> goal[i];
    // }


    // CubeState solved = {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5};
    // CubeState start = moveR(solved);
    // start = moveB(start);
    // start = moveD(start);
    // start = moveU(start);
    // start = moveDp(start);
    // start = moveRp(start);
    // start = moveBp(start);
    // start = moveUp(start);
    // start = moveU(start);
    // start = moveDp(start);
    // CubeState goal = solved;
    // // CubeState test = moveRp(start);

    CubeState solved = {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5};
    CubeState start = moveF(solved);   // 1
    start = moveU(start);              // 2
    start = moveR(start);              // 3
    // start = moveL(start);              // 4
    start = moveD(start);              // 5
    // start = moveB(start);              // 6
    // start = moveFp(start);             // 7
    // start = moveUp(start);             // 8
    // start = moveRp(start);             // 9
    start = moveLp(start);             // 10
    // start = moveDp(start);             // 11
    start = moveBp(start);             // 12
    CubeState goal = solved;

    // int diff = 0;
    // for (int i = 0; i < 54; ++i) if (test[i] != goal[i]) diff++;
    // cout << "After F then F': " << diff << " stickers differ (should be 0)" << endl;



    // for(int i = 0; i < 54; i++)
    //     cout<<start[i]<<" ";


    // cout << "Starting search with all 12 moves (F, F', B, B', U, U', D, D', L, L', R, R')..." << endl;
    // cout << "Initial heuristic: " << heuristic(start, goal) << " misplaced pieces" << endl;

    string solution = AStarSolver(start, goal);
    cout << "Solution: " << solution << endl;

    // string solution = IDAStarSolver(start, goal, 40); // 6 is a reasonable max depth for simple tests
    // cout << "Solution: " << solution << endl;

    return 0;
}