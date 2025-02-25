#ifndef GAME_HPP
#define GAME_HPP

#include <vector>
#include <string>
#include <unordered_set>
#include <stack>
#include <queue>
#include <map>
#include <functional>

struct Cell {
    char letter;
    bool visited;
    bool walls[4];  // top, right, bottom, left walls
    bool isPath;    // For visualization of final path
    Cell() : letter(' '), visited(false), isPath(false) {
        walls[0] = walls[1] = walls[2] = walls[3] = true;
    }
};

class Game {
private:
    std::vector<std::vector<Cell>> grid;
    std::vector<std::vector<bool>> adjacencyMatrix;
    std::unordered_set<std::string> dictionary;
    std::map<std::pair<std::pair<int,int>, std::pair<int,int>>, int> wordWeights;
    int gridSize;
    int score;
    std::pair<int, int> startPos;
    std::pair<int, int> endPos;
    
    struct PathInfo {
        std::vector<std::pair<int, int>> path;
        int wordCount;
        int score;
        
        PathInfo(const std::vector<std::pair<int, int>>& p, int w, int s) 
            : path(p), wordCount(w), score(s) {}
    };
    
    std::vector<PathInfo> allPaths;  // Store all valid paths
    int calculateScore(const std::vector<std::pair<int, int>>& path) const;
    
    std::vector<std::pair<int, int>> bfsSearch(bool skipVisualization = false);
    std::vector<std::pair<int, int>> dfsSearch(bool skipVisualization = false);
    
public:
    Game(int size);
    void loadDictionary(const std::string& filename);
    void generateGridDFS();
    void placeWordsInGrid();
    void setRandomStartEnd();
    std::vector<std::pair<int, int>> findPath(bool useBFS);
    void visualizeTraversal(const std::vector<std::pair<int, int>>& path);
    void printGridWithWalls(bool showPath = false) const;
    int getScore() const;
    bool isValidWord(const std::string& word) const;
    void printGrid() const;
    bool checkPath(const std::vector<std::pair<int, int>>& path, std::string& word);
    int calculatePathScore(const std::vector<std::pair<int, int>>& path);
    enum class Direction { UP, RIGHT, DOWN, LEFT };
    bool makeMove(Direction dir);  // Returns true if move is valid
    std::pair<int, int> getCurrentPos() const { return currentPos; }
    bool hasReachedEnd() const { return currentPos == endPos; }
    void startManualGame();
    std::vector<std::pair<int, int>> findPathDijkstra();  // New method for Dijkstra's algorithm
    enum class Difficulty { EASY, MEDIUM, HARD };
    void startVersusAI(Difficulty diff);
    std::vector<std::pair<int, int>> findAIPath(Difficulty diff);
    
private:
    bool isValidPosition(int x, int y) const;
    void initializeAdjacencyMatrix();
    void updateAdjacencyMatrix();
    void generateMazeUsingDFS();
    std::vector<std::pair<int, int>> getUnvisitedNeighbors(int x, int y);
    void removeWall(int x1, int y1, int x2, int y2);
    bool canPlaceWord(const std::string& word, int x, int y, int dx, int dy);
    void placeWord(const std::string& word, int x, int y, int dx, int dy);
    void clearVisited();
    void delay() const;  // For visualization
    void assignRandomLetters();
    void showFinalPath(const std::vector<std::pair<int, int>>& path);
    int countWordsInPath(const std::vector<std::pair<int, int>>& path) const;
    std::vector<std::pair<int, int>> bestPath;
    int maxWordCount;
    std::pair<int, int> currentPos;  // Current position for manual play
    std::vector<std::pair<int, int>> manualPath;  // Store player's path
    
    struct Node {
        std::pair<int, int> pos;
        int distance;
        Node(std::pair<int, int> p, int d) : pos(p), distance(d) {}
        bool operator>(const Node& other) const { return distance > other.distance; }
    };
    
    // Add helper methods for AI paths
    std::vector<std::pair<int, int>> findFirstPath();  // For easy difficulty
    std::vector<std::pair<int, int>> findShortestPath();  // For medium difficulty
    // Hard difficulty will use existing dfsSearch()
    
    int coordToIndex(int x, int y) const { return x * gridSize + y; }
    std::pair<int, int> indexToCoord(int index) const { 
        return {index / gridSize, index % gridSize}; 
    }
};

#endif 