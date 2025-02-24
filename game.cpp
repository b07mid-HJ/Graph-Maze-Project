#include "game.hpp"
#include <iostream>
#include <fstream>
#include <random>
#include <stack>
#include <algorithm>
#include <chrono>
#include <thread>
#include <queue>
#include <conio.h>  // For _getch()
#include <cctype>   // For toupper()

Game::Game(int size) : gridSize(size), score(0), startPos({0,0}), endPos({0,0}) {
    grid.resize(size, std::vector<Cell>(size));
    initializeAdjacencyMatrix();
    generateGridDFS();
}

void Game::initializeAdjacencyMatrix() {
    int totalCells = gridSize * gridSize;
    adjacencyMatrix.resize(totalCells, std::vector<bool>(totalCells, false));
    
    // Fill the adjacency matrix based on wall information
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            int currentIndex = i * gridSize + j;
            
            // Check all four directions
            // Up
            if (i > 0 && !grid[i][j].walls[0] && !grid[i-1][j].walls[2]) {
                int neighborIndex = (i-1) * gridSize + j;
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Right
            if (j < gridSize-1 && !grid[i][j].walls[1] && !grid[i][j+1].walls[3]) {
                int neighborIndex = i * gridSize + (j+1);
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Down
            if (i < gridSize-1 && !grid[i][j].walls[2] && !grid[i+1][j].walls[0]) {
                int neighborIndex = (i+1) * gridSize + j;
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Left
            if (j > 0 && !grid[i][j].walls[3] && !grid[i][j-1].walls[1]) {
                int neighborIndex = i * gridSize + (j-1);
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
        }
    }
}

void Game::generateGridDFS() {
    generateMazeUsingDFS();
    // Update adjacency matrix after maze generation
    updateAdjacencyMatrix();
    // Initialize all cells with spaces before placing words
    for (auto& row : grid) {
        for (auto& cell : row) {
            cell.letter = ' ';  // Initialize with spaces
        }
    }
    // Place words in the grid
    placeWordsInGrid();
}

void Game::generateMazeUsingDFS() {
    // Reset all cells
    for (auto& row : grid) {
        for (auto& cell : row) {
            cell.visited = false;
            cell.walls[0] = cell.walls[1] = cell.walls[2] = cell.walls[3] = true;
        }
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Start from a random cell
    int startX = gen() % gridSize;
    int startY = gen() % gridSize;
    
    std::stack<std::pair<int, int>> stack;
    stack.push({startX, startY});
    grid[startX][startY].visited = true;

    while (!stack.empty()) {
        auto current = stack.top();
        auto neighbors = getUnvisitedNeighbors(current.first, current.second);
        
        if (neighbors.empty()) {
            stack.pop();
            continue;
        }

        // Randomly choose next neighbor
        std::uniform_int_distribution<> dis(0, neighbors.size() - 1);
        auto next = neighbors[dis(gen)];
        
        // Remove wall between current and next
        removeWall(current.first, current.second, next.first, next.second);
        
        grid[next.first][next.second].visited = true;
        stack.push(next);
    }
}

std::vector<std::pair<int, int>> Game::getUnvisitedNeighbors(int x, int y) {
    std::vector<std::pair<int, int>> neighbors;
    
    // Check all four directions (top, right, bottom, left)
    const int dx[] = {-1, 0, 1, 0};
    const int dy[] = {0, 1, 0, -1};
    
    for (int i = 0; i < 4; i++) {
        int newX = x + dx[i];
        int newY = y + dy[i];
        
        if (isValidPosition(newX, newY) && !grid[newX][newY].visited) {
            neighbors.push_back({newX, newY});
        }
    }
    
    return neighbors;
}

void Game::removeWall(int x1, int y1, int x2, int y2) {
    // Determine which wall to remove based on relative positions
    if (x1 == x2) {
        if (y1 < y2) {
            grid[x1][y1].walls[1] = false;  // Remove right wall
            grid[x2][y2].walls[3] = false;  // Remove left wall
        } else {
            grid[x1][y1].walls[3] = false;  // Remove left wall
            grid[x2][y2].walls[1] = false;  // Remove right wall
        }
    } else {
        if (x1 < x2) {
            grid[x1][y1].walls[2] = false;  // Remove bottom wall
            grid[x2][y2].walls[0] = false;  // Remove top wall
        } else {
            grid[x1][y1].walls[0] = false;  // Remove top wall
            grid[x2][y2].walls[2] = false;  // Remove bottom wall
        }
    }
    
    // Update adjacency matrix
    int index1 = coordToIndex(x1, y1);
    int index2 = coordToIndex(x2, y2);
    adjacencyMatrix[index1][index2] = true;
    adjacencyMatrix[index2][index1] = true;
}

void Game::assignRandomLetters() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 25);
    
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            grid[i][j].letter = 'A' + dis(gen);
        }
    }
}

void Game::printGridWithWalls(bool showPath) const {
    // Print top border
    for (int j = 0; j < gridSize; j++) {
        std::cout << "+---";
    }
    std::cout << "+" << std::endl;

    for (int i = 0; i < gridSize; i++) {
        // Print cells and vertical walls
        std::cout << "|";
        for (int j = 0; j < gridSize; j++) {
            char displayChar = (grid[i][j].letter == ' ') ? '_' : grid[i][j].letter;
            if (showPath) {
                if (std::make_pair(i, j) == startPos) {
                    std::cout << "\033[32m " << displayChar << " \033[0m";  // Green for start
                } else if (std::make_pair(i, j) == endPos) {
                    std::cout << "\033[31m " << displayChar << " \033[0m";  // Red for end
                } else if (grid[i][j].isPath) {
                    std::cout << "\033[33m " << displayChar << " \033[0m";  // Yellow for path
                } else if (grid[i][j].visited) {
                    std::cout << "\033[34m " << displayChar << " \033[0m";  // Blue for visited
                } else {
                    std::cout << " " << displayChar << " ";
                }
            } else {
                std::cout << " " << displayChar << " ";
            }
            
            if (grid[i][j].walls[1]) std::cout << "|";
            else std::cout << " ";
        }
        std::cout << std::endl;

        // Print horizontal walls
        for (int j = 0; j < gridSize; j++) {
            if (grid[i][j].walls[2]) std::cout << "+---";
            else std::cout << "+   ";
        }
        std::cout << "+" << std::endl;
    }
}

void Game::loadDictionary(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open dictionary file: " << filename << std::endl;
        return;
    }
    
    std::string word;
    while (std::getline(file, word)) {
        if (!word.empty()) {  // Skip empty lines
            dictionary.insert(word);
        }
    }
    
    // Add debug print
    std::cout << "Loaded " << dictionary.size() << " words from dictionary\n";
    
    if (dictionary.empty()) {
        std::cerr << "Warning: No words loaded from dictionary" << std::endl;
    }
}

bool Game::isValidWord(const std::string& word) const {
    return dictionary.find(word) != dictionary.end();
}

void Game::printGrid() const {
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            std::cout << grid[i][j].letter << " ";
        }
        std::cout << std::endl;
    }
}

bool Game::isValidPosition(int x, int y) const {
    return x >= 0 && x < gridSize && y >= 0 && y < gridSize;
}

int Game::getScore() const {
    return score;
}

bool Game::checkPath(const std::vector<std::pair<int, int>>& path, std::string& word) {
    word.clear();
    
    for (const auto& pos : path) {
        if (!isValidPosition(pos.first, pos.second)) {
            return false;
        }
        word += grid[pos.first][pos.second].letter;
    }
    
    if (isValidWord(word)) {
        score += word.length();  // Simple scoring: 1 point per letter
        return true;
    }
    return false;
}

void Game::placeWordsInGrid() {
    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (const auto& word : dictionary) {
        if (word.length() > 2) {  // Only place words longer than 2 letters
            int attempts = 0;
            bool placed = false;
            
            while (!placed && attempts < 10) {
                int x = gen() % gridSize;
                int y = gen() % gridSize;
                
                // Try different directions
                const int dx[] = {-1, 0, 1, 0, -1, -1, 1, 1};
                const int dy[] = {0, 1, 0, -1, -1, 1, -1, 1};
                
                for (int dir = 0; dir < 8 && !placed; dir++) {
                    if (canPlaceWord(word, x, y, dx[dir], dy[dir])) {
                        placeWord(word, x, y, dx[dir], dy[dir]);
                        placed = true;
                    }
                }
                attempts++;
            }
        }
    }
}

bool Game::canPlaceWord(const std::string& word, int x, int y, int dx, int dy) {
    if (!isValidPosition(x, y)) return false;
    
    for (char c : word) {
        if (!isValidPosition(x, y) || 
            (grid[x][y].letter != ' ' && grid[x][y].letter != c)) {
            return false;
        }
        x += dx;
        y += dy;
    }
    return true;
}

void Game::placeWord(const std::string& word, int x, int y, int dx, int dy) {
    int startX = x;
    int startY = y;
    
    for (size_t i = 0; i < word.length(); i++) {
        grid[x][y].letter = word[i];
        if (i < word.length() - 1) {
            int nextX = x + dx;
            int nextY = y + dy;
            removeWall(x, y, nextX, nextY);
            // Add weight to the path
            wordWeights[{{x,y}, {nextX,nextY}}] = word.length();  // Weight based on word length
            wordWeights[{{nextX,nextY}, {x,y}}] = word.length();  // Add reverse direction
        }
        x += dx;
        y += dy;
    }
}

void Game::setRandomStartEnd() {
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Set random start position
    startPos = {gen() % gridSize, gen() % gridSize};
    
    // Set random end position (ensure it's different from start)
    do {
        endPos = {gen() % gridSize, gen() % gridSize};
    } while (endPos == startPos);
}

void Game::delay() const {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

std::vector<std::pair<int, int>> Game::findPath(bool useBFS) {
    clearVisited();
    auto path = useBFS ? bfsSearch() : dfsSearch();
    if (!path.empty()) {
        showFinalPath(path);
    }
    return path;
}

int Game::countWordsInPath(const std::vector<std::pair<int, int>>& path) const {
    // Get the sequence of letters in the path
    std::string pathLetters;
    for (const auto& pos : path) {
        pathLetters += grid[pos.first][pos.second].letter;
    }
    
    // Count how many dictionary words appear in the path sequence
    int wordCount = 0;
    for (const auto& word : dictionary) {
        if (pathLetters.find(word) != std::string::npos) {
            wordCount++;
        }
    }
    return wordCount;
}

std::vector<std::pair<int, int>> Game::bfsSearch() {
    int totalCells = gridSize * gridSize;
    std::vector<bool> visited(totalCells, false);
    std::vector<int> parent(totalCells, -1);
    
    // Convert start and end positions to indices
    int startIndex = coordToIndex(startPos.first, startPos.second);
    int endIndex = coordToIndex(endPos.first, endPos.second);
    
    std::queue<int> q;
    q.push(startIndex);
    visited[startIndex] = true;
    
    bool foundPath = false;
    while (!q.empty() && !foundPath) {
        int currentIndex = q.front();
        q.pop();
        
        // Visualize current position
        visualizeTraversal({{currentIndex / gridSize, currentIndex % gridSize}});
        
        if (currentIndex == endIndex) {
            foundPath = true;
            break;
        }
        
        // Check all possible connections in adjacency matrix
        for (int nextIndex = 0; nextIndex < totalCells; nextIndex++) {
            if (adjacencyMatrix[currentIndex][nextIndex] && !visited[nextIndex]) {
                visited[nextIndex] = true;
                parent[nextIndex] = currentIndex;
                q.push(nextIndex);
            }
        }
    }
    
    if (!foundPath) return {};
    
    // Reconstruct path from end to start
    std::vector<std::pair<int, int>> path;
    for (int current = endIndex; current != -1; current = parent[current]) {
        path.push_back({current / gridSize, current % gridSize});
    }
    std::reverse(path.begin(), path.end());
    
    return path;
}

int Game::calculateScore(const std::vector<std::pair<int, int>>& path) const {
    // Get the sequence of letters in the path
    std::string pathLetters;
    for (const auto& pos : path) {
        pathLetters += grid[pos.first][pos.second].letter;
    }
    
    // Calculate score based on words found
    int score = 0;
    for (const auto& word : dictionary) {
        if (pathLetters.find(word) != std::string::npos) {
            score += word.length() * 2;  // More points for longer words
        }
    }
    
    // Subtract points for path length to favor shorter paths
    score -= path.size();
    return score;
}

std::vector<std::pair<int, int>> Game::dfsSearch() {
    allPaths.clear();
    int totalCells = gridSize * gridSize;
    std::vector<bool> visited(totalCells, false);
    std::vector<int> currentPath;
    
    int startIndex = startPos.first * gridSize + startPos.second;
    int endIndex = endPos.first * gridSize + endPos.second;
    
    std::function<void(int)> dfs = [&](int currentIndex) {
        visited[currentIndex] = true;
        currentPath.push_back(currentIndex);
        
        // Convert current path to coordinates for visualization
        std::vector<std::pair<int, int>> coordPath;
        for (int idx : currentPath) {
            coordPath.push_back({idx / gridSize, idx % gridSize});
        }
        visualizeTraversal(coordPath);
        
        if (currentIndex == endIndex) {
            int wordCount = countWordsInPath(coordPath);
            int score = calculateScore(coordPath);
            allPaths.emplace_back(coordPath, wordCount, score);
        }
        
        // Check all possible connections in adjacency matrix
        for (int nextIndex = 0; nextIndex < totalCells; nextIndex++) {
            if (adjacencyMatrix[currentIndex][nextIndex] && !visited[nextIndex]) {
                dfs(nextIndex);
            }
        }
        
        visited[currentIndex] = false;
        currentPath.pop_back();
    };
    
    dfs(startIndex);
    
    if (allPaths.empty()) return {};
    
    // Sort paths by word count, score, and length
    std::sort(allPaths.begin(), allPaths.end(), 
        [](const PathInfo& a, const PathInfo& b) {
            if (a.wordCount != b.wordCount) return a.wordCount > b.wordCount;
            if (a.score != b.score) return a.score > b.score;
            return a.path.size() < b.path.size();
        });
    
    return allPaths[0].path;
}

void Game::visualizeTraversal(const std::vector<std::pair<int, int>>& path) {
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
    
    // Mark path cells
    for (const auto& pos : path) {
        grid[pos.first][pos.second].isPath = true;
    }
    
    printGridWithWalls(true);
    delay();
    
    // Unmark path cells except for the final path
    if (path.back() != endPos) {
        for (const auto& pos : path) {
            grid[pos.first][pos.second].isPath = false;
        }
    }
}

// Add this method to show final path clearly
void Game::showFinalPath(const std::vector<std::pair<int, int>>& path) {
    clearVisited();
    score = 0;
    
    // Get the sequence of letters in the path
    std::string pathLetters;
    for (const auto& pos : path) {
        pathLetters += grid[pos.first][pos.second].letter;
        grid[pos.first][pos.second].isPath = true;
    }
    
    std::cout << "\nFinal Path Found:\n";
    
    // Print maze with X markers
    for (int j = 0; j < gridSize; j++) {
        std::cout << "+---";
    }
    std::cout << "+" << std::endl;

    for (int i = 0; i < gridSize; i++) {
        std::cout << "|";
        for (int j = 0; j < gridSize; j++) {
            char displayChar = (grid[i][j].letter == ' ') ? '_' : grid[i][j].letter;
            if (std::make_pair(i, j) == startPos) {
                std::cout << "\033[32m " << displayChar << " \033[0m";
            } else if (std::make_pair(i, j) == endPos) {
                std::cout << "\033[31m " << displayChar << " \033[0m";
            } else if (grid[i][j].isPath) {
                std::cout << "\033[33m X \033[0m";
            } else {
                std::cout << " " << displayChar << " ";
            }
            
            if (grid[i][j].walls[1]) std::cout << "|";
            else std::cout << " ";
        }
        std::cout << std::endl;

        for (int j = 0; j < gridSize; j++) {
            if (grid[i][j].walls[2]) std::cout << "+---";
            else std::cout << "+   ";
        }
        std::cout << "+" << std::endl;
    }
    
    // Show words found in the path
    std::cout << "\nWords found in path:\n";
    int totalWords = 0;
    for (const auto& word : dictionary) {
        if (pathLetters.find(word) != std::string::npos) {
            totalWords++;
            score += word.length()*2;
            std::cout << "- " << word << " (score: " << word.length() << ")\n";
        }
    }
    
    std::cout << "\nTotal words found: " << totalWords << "\n";
    std::cout << "Final score: " << score << "\n";
    std::cout << "Path length: " << path.size() << "\n";
    
    std::cout << "\nPath sequence: ";
    for (const auto& pos : path) {
        std::cout << grid[pos.first][pos.second].letter << " ";
    }
    std::cout << "\n";
}

void Game::clearVisited() {
    for (auto& row : grid) {
        for (auto& cell : row) {
            cell.visited = false;
            cell.isPath = false;
        }
    }
}

int Game::calculatePathScore(const std::vector<std::pair<int, int>>& path) {
    int pathScore = 0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        auto current = path[i];
        auto next = path[i + 1];
        auto weight = wordWeights[{current, next}];
        pathScore += weight > 0 ? weight : 1;  // Default weight of 1 for non-word paths
    }
    return pathScore;
}

void Game::startManualGame() {
    clearVisited();
    currentPos = startPos;
    manualPath.clear();
    manualPath.push_back(currentPos);
    int currentScore = 0;
    
    while (!hasReachedEnd()) {
        system("cls");  // Clear screen
        
        // Mark current path
        for (const auto& pos : manualPath) {
            grid[pos.first][pos.second].isPath = true;
        }
        
        // Print current state
        std::cout << "\nCurrent Position: (" << currentPos.first << "," << currentPos.second << ")\n";
        std::cout << "Path Length: " << manualPath.size() << "\n";
        std::cout << "Current Score: " << currentScore << "\n\n";
        
        printGridWithWalls(true);
        std::cout << "\nControls:\n";
        std::cout << "W = Up\n";
        std::cout << "S = Down\n";
        std::cout << "A = Left\n";
        std::cout << "D = Right\n";
        std::cout << "Q = Quit\n";
        
        char key = _getch();  // Get character without Enter
        bool moved = false;
        
        switch (toupper(key)) {
            case 'W': moved = makeMove(Direction::UP); break;
            case 'D': moved = makeMove(Direction::RIGHT); break;
            case 'S': moved = makeMove(Direction::DOWN); break;
            case 'A': moved = makeMove(Direction::LEFT); break;
            case 'Q': return;
        }
        
        if (moved) {
            // Check if current path forms any words
            std::string pathWord;
            if (checkPath(manualPath, pathWord)) {
                std::cout << "\nFound word: " << pathWord << "!\n";
                currentScore += pathWord.length();
                system("pause");
            }
        }
    }
    
    if (hasReachedEnd()) {
        system("cls");
        std::cout << "\nCongratulations! You reached the end!\n\n";
        showFinalPath(manualPath);
        
        // Calculate final score
        int wordCount = countWordsInPath(manualPath);
        int finalScore = calculateScore(manualPath);
        
        std::cout << "\nFinal Statistics:\n";
        std::cout << "Path Length: " << manualPath.size() << "\n";
        std::cout << "Words Found: " << wordCount << "\n";
        std::cout << "Final Score: " << finalScore << "\n";
    }
}

bool Game::makeMove(Direction dir) {
    int newX = currentPos.first;
    int newY = currentPos.second;
    int wallIndex;
    
    switch (dir) {
        case Direction::UP:
            newX--;
            wallIndex = 0;
            break;
        case Direction::RIGHT:
            newY++;
            wallIndex = 1;
            break;
        case Direction::DOWN:
            newX++;
            wallIndex = 2;
            break;
        case Direction::LEFT:
            newY--;
            wallIndex = 3;
            break;
    }
    
    // Check if move is valid
    if (!isValidPosition(newX, newY) || grid[currentPos.first][currentPos.second].walls[wallIndex]) {
        std::cout << "\nInvalid move! There's a wall there.\n";
        system("pause");
        return false;
    }
    
    currentPos = {newX, newY};
    manualPath.push_back(currentPos);
    return true;
}

std::vector<std::pair<int, int>> Game::findPathDijkstra() {
    // First show initial maze
    std::cout << "\nInitial Maze:\n";
    printGridWithWalls(true);
    std::cout << "\nPress any key to start pathfinding...";
    _getch();
    
    allPaths.clear();
    clearVisited();  // Clear any previous visited marks
    
    int totalCells = gridSize * gridSize;
    std::vector<int> distance(totalCells, INT_MAX);
    std::vector<bool> visited(totalCells, false);
    std::map<int, std::vector<std::pair<int, int>>> paths;
    
    // Convert start and end positions to indices
    int startIndex = coordToIndex(startPos.first, startPos.second);
    int endIndex = coordToIndex(endPos.first, endPos.second);
    
    // Priority queue for Dijkstra's algorithm (min heap)
    std::priority_queue<std::pair<int,int>, std::vector<std::pair<int,int>>, 
        std::greater<std::pair<int,int>>> pq;
    
    // Initialize start position
    distance[startIndex] = 0;
    pq.push({0, startIndex});
    paths[startIndex] = {startPos};
    
    while (!pq.empty()) {
        int currentDist = pq.top().first;
        int currentIndex = pq.top().second;
        pq.pop();
        
        if (visited[currentIndex]) continue;
        visited[currentIndex] = true;
        
        auto currentCoord = indexToCoord(currentIndex);
        grid[currentCoord.first][currentCoord.second].visited = true;
        
        // Show exploration progress
        system("cls");
        std::cout << "\nExploring maze using Dijkstra's algorithm...\n";
        printGridWithWalls(true);
        delay();
        
        if (currentIndex == endIndex) {
            // Found a path to end
            int wordCount = countWordsInPath(paths[currentIndex]);
            int score = calculateScore(paths[currentIndex]);
            allPaths.emplace_back(paths[currentIndex], wordCount, score);
            continue;  // Continue searching for other possible paths
        }
        
        // Check all possible connections in adjacency matrix
        for (int nextIndex = 0; nextIndex < totalCells; nextIndex++) {
            if (!adjacencyMatrix[currentIndex][nextIndex] || visited[nextIndex]) continue;
            
            auto nextCoord = indexToCoord(nextIndex);
            auto currentPath = paths[currentIndex];
            currentPath.push_back(nextCoord);
            
            // Calculate edge weight based on potential words
            std::string pathWord;
            for (const auto& pos : currentPath) {
                pathWord += grid[pos.first][pos.second].letter;
            }
            
            int weight = isValidWord(pathWord) ? 1 : 2;
            int newDist = currentDist + weight;
            
            if (newDist < distance[nextIndex]) {
                distance[nextIndex] = newDist;
                paths[nextIndex] = currentPath;
                pq.push({newDist, nextIndex});
            }
        }
    }
    
    if (allPaths.empty()) return {};
    
    // Sort paths based on criteria
    std::sort(allPaths.begin(), allPaths.end(), 
        [](const PathInfo& a, const PathInfo& b) {
            if (a.wordCount != b.wordCount) return a.wordCount > b.wordCount;
            if (a.score != b.score) return a.score > b.score;
            return a.path.size() < b.path.size();
        });
    
    // Show final path with X markers
    system("cls");
    std::cout << "\nFinal Path Found:\n";
    clearVisited();  // Clear exploration marks
    
    // Mark final path with X
    for (const auto& pos : allPaths[0].path) {
        grid[pos.first][pos.second].isPath = true;
    }
    showFinalPath(allPaths[0].path);
    
    return allPaths[0].path;
}

void Game::startVersusAI(Difficulty diff) {
    // Player's turn
    std::cout << "\nPLAYER'S TURN\n";
    std::cout << "=============\n";
    startManualGame();
    
    // Store player's path and score
    auto playerPath = manualPath;
    int playerWordScore = score;
    int pathLengthBonus = std::max(0, 50 - (int)playerPath.size()) * 2;
    int playerScore = playerWordScore + pathLengthBonus;
    
    // AI's turn
    std::cout << "\nPress any key for AI's turn...";
    _getch();
    std::cout << "\n\nAI'S TURN\n";
    std::cout << "=========\n";
    clearVisited();  // Reset the grid
    auto aiPath = findAIPath(diff);
    int aiWordScore = score;
    int aiPathBonus = std::max(0, 50 - (int)aiPath.size()) * 2;
    int aiScore = aiWordScore + aiPathBonus;
    
    // Show both paths side by side
    system("cls");
    std::cout << "\nFINAL PATHS COMPARISON:\n";
    std::cout << "======================\n\n";
    
    std::cout << "Player's Path:";
    std::cout << std::string(gridSize * 4 + 5, ' ') << "AI's Path:\n";
    
    // Print top borders
    for (int j = 0; j < gridSize; j++) {
        std::cout << "+---";
    }
    std::cout << "+     ";
    for (int j = 0; j < gridSize; j++) {
        std::cout << "+---";
    }
    std::cout << "+\n";
    
    // Print grid contents
    for (int i = 0; i < gridSize; i++) {
        // First row - Player's maze
        std::cout << "|";
        for (int j = 0; j < gridSize; j++) {
            char displayChar = (grid[i][j].letter == ' ') ? '_' : grid[i][j].letter;
            if (std::make_pair(i, j) == startPos) {
                std::cout << "\033[32m " << displayChar << " \033[0m";
            } else if (std::make_pair(i, j) == endPos) {
                std::cout << "\033[31m " << displayChar << " \033[0m";
            } else {
                bool isInPath = false;
                for (const auto& pos : playerPath) {
                    if (pos.first == i && pos.second == j) {
                        isInPath = true;
                        break;
                    }
                }
                if (isInPath) {
                    std::cout << "\033[33m X \033[0m";
                } else {
                    std::cout << " " << displayChar << " ";
                }
            }
            if (grid[i][j].walls[1]) std::cout << "|";
            else std::cout << " ";
        }
        
        // Spacing between mazes
        std::cout << "     ";
        
        // Second row - AI's maze
        std::cout << "|";
        for (int j = 0; j < gridSize; j++) {
            char displayChar = (grid[i][j].letter == ' ') ? '_' : grid[i][j].letter;
            if (std::make_pair(i, j) == startPos) {
                std::cout << "\033[32m " << displayChar << " \033[0m";
            } else if (std::make_pair(i, j) == endPos) {
                std::cout << "\033[31m " << displayChar << " \033[0m";
            } else {
                bool isInPath = false;
                for (const auto& pos : aiPath) {
                    if (pos.first == i && pos.second == j) {
                        isInPath = true;
                        break;
                    }
                }
                if (isInPath) {
                    std::cout << "\033[33m X \033[0m";
                } else {
                    std::cout << " " << displayChar << " ";
                }
            }
            if (grid[i][j].walls[1]) std::cout << "|";
            else std::cout << " ";
        }
        std::cout << "\n";
        
        // Print horizontal walls for both mazes
        for (int j = 0; j < gridSize; j++) {
            if (grid[i][j].walls[2]) std::cout << "+---";
            else std::cout << "+   ";
        }
        std::cout << "+     ";
        for (int j = 0; j < gridSize; j++) {
            if (grid[i][j].walls[2]) std::cout << "+---";
            else std::cout << "+   ";
        }
        std::cout << "+\n";
    }
    
    // Show scores
    std::cout << "\nPlayer Score Breakdown:";
    std::cout << std::string(gridSize * 4 - 5, ' ') << "AI Score Breakdown:\n";
    std::cout << "Word Score: " << playerWordScore;
    std::cout << std::string(gridSize * 4 - 5, ' ') << "Word Score: " << aiWordScore << "\n";
    std::cout << "Path Length: " << playerPath.size();
    std::cout << std::string(gridSize * 4 - 5, ' ') << "Path Length: " << aiPath.size() << "\n";
    std::cout << "Length Bonus: " << pathLengthBonus;
    std::cout << std::string(gridSize * 4 - 5, ' ') << "Length Bonus: " << aiPathBonus << "\n";
    std::cout << "Total Score: " << playerScore;
    std::cout << std::string(gridSize * 4 - 5, ' ') << "Total Score: " << aiScore << "\n\n";
    
    // Show winner
    std::cout << "RESULT: ";
    if (playerScore > aiScore) {
        std::cout << "Player wins!\n";
    } else if (aiScore > playerScore) {
        std::cout << "AI wins!\n";
    } else {
        std::cout << "It's a tie!\n";
    }
}

std::vector<std::pair<int, int>> Game::findAIPath(Difficulty diff) {
    switch (diff) {
        case Difficulty::EASY:
            return findFirstPath();
        case Difficulty::MEDIUM:
            return findPathDijkstra();
        case Difficulty::HARD:
            return dfsSearch();  // Uses existing implementation that finds optimal path
        default:
            return {};
    }
}

std::vector<std::pair<int, int>> Game::findFirstPath() {
    std::vector<bool> visited(gridSize * gridSize, false);
    std::vector<std::pair<int, int>> path;
    
    std::function<bool(int)> dfs = [&](int currentIndex) {
        visited[currentIndex] = true;
        path.push_back(indexToCoord(currentIndex));
        
        if (indexToCoord(currentIndex) == endPos) {
            return true;
        }
        
        // Check all possible connections in adjacency matrix
        for (int nextIndex = 0; nextIndex < gridSize * gridSize; nextIndex++) {
            if (adjacencyMatrix[currentIndex][nextIndex] && !visited[nextIndex]) {
                if (dfs(nextIndex)) {
                    return true;
                }
            }
        }
        
        path.pop_back();
        return false;
    };
    
    // Convert start position to index
    int startIndex = coordToIndex(startPos.first, startPos.second);
    dfs(startIndex);
    showFinalPath(path);
    return path;
}

// std::vector<std::pair<int, int>> Game::findShortestPath() {
//     int totalCells = gridSize * gridSize;
//     std::queue<std::vector<int>> pathQueue;
//     std::vector<bool> visited(totalCells, false);
    
//     // Convert start position to index and initialize
//     int startIndex = coordToIndex(startPos.first, startPos.second);
//     int endIndex = coordToIndex(endPos.first, endPos.second);
//     pathQueue.push({startIndex});
//     visited[startIndex] = true;
    
//     while (!pathQueue.empty()) {
//         auto currentPath = pathQueue.front();
//         pathQueue.pop();
        
//         int currentIndex = currentPath.back();
        
//         if (currentIndex == endIndex) {
//             // Convert indices back to coordinates for return
//             std::vector<std::pair<int, int>> coordPath;
//             for (int index : currentPath) {
//                 coordPath.push_back(indexToCoord(index));
//             }
//             showFinalPath(coordPath);
//             return coordPath;
//         }
        
//         // Check all possible connections in adjacency matrix
//         for (int nextIndex = 0; nextIndex < totalCells; nextIndex++) {
//             if (adjacencyMatrix[currentIndex][nextIndex] && !visited[nextIndex]) {
//                 visited[nextIndex] = true;
//                 auto newPath = currentPath;
//                 newPath.push_back(nextIndex);
//                 pathQueue.push(newPath);
//             }
//         }
//     }
    
//     return {};
// }

void Game::updateAdjacencyMatrix() {
    int totalCells = gridSize * gridSize;
    // Reset adjacency matrix
    for (auto& row : adjacencyMatrix) {
        std::fill(row.begin(), row.end(), false);
    }
    
    // Fill the adjacency matrix based on wall information
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            int currentIndex = i * gridSize + j;
            
            // Check all four directions
            // Up
            if (i > 0 && !grid[i][j].walls[0] && !grid[i-1][j].walls[2]) {
                int neighborIndex = (i-1) * gridSize + j;
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Right
            if (j < gridSize-1 && !grid[i][j].walls[1] && !grid[i][j+1].walls[3]) {
                int neighborIndex = i * gridSize + (j+1);
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Down
            if (i < gridSize-1 && !grid[i][j].walls[2] && !grid[i+1][j].walls[0]) {
                int neighborIndex = (i+1) * gridSize + j;
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Left
            if (j > 0 && !grid[i][j].walls[3] && !grid[i][j-1].walls[1]) {
                int neighborIndex = i * gridSize + (j-1);
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
        }
    }
} 