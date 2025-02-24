# Maze Generation, Pathfinding, and AI Gameplay

This document explains the maze generation process, pathfinding algorithms, scoring systems, and AI gameplay mechanics. We also delve into how AI difficulty levels work in a versus mode.

---

## Maze Generation Process

### 1. Initial Setup (Constructor)

- **Creates grid** of size `gridSize × gridSize`.
- Initializes the **adjacency matrix**.
- Starts the **maze generation process**.

```cpp
Game::Game(int size) : gridSize(size), score(0), startPos({0,0}), endPos({0,0}) {
    grid.resize(size, std::vector<Cell>(size));
    initializeAdjacencyMatrix();
    generateGridDFS();
}
```

### 2. `initializeAdjacencyMatrix()`

- Creates a `gridSize² × gridSize²` matrix.
- Each cell `(i,j)` represents a potential connection between cells `i` and `j`.
- Initially, all connections are set to `false`.

**Example:** For a `5 × 5` grid, creates a `25 × 25` matrix.
2. Cell Indexing:

```cpp
int coordToIndex(int x, int y) const { return x * gridSize + y; }
```

- Converts 2D coordinates to 1D index
For 5×5 grid:
  - Cell (0,0) → index 0
  - Cell (0,1) → index 1
  - Cell (1,0) → index 5
  - Cell (4,4) → index 24

```
Grid:           Cell Indices:
+--+--+--+     +--+--+--+
|A |B |C |     |0 |1 |2 |
+--+--+--+     +--+--+--+
|D |E |F |     |3 |4 |5 |
+--+--+--+     +--+--+--+
|G |H |I |     |6 |7 |8 |
+--+--+--+     +--+--+--+

Adjacency Matrix (9×9):
    0  1  2  3  4  5  6  7  8
0   0  1  0  1  0  0  0  0  0
1   1  0  1  0  1  0  0  0  0
2   0  1  0  0  0  1  0  0  0
3   1  0  0  0  1  0  1  0  0
4   0  1  0  1  0  1  0  1  0
5   0  0  1  0  1  0  0  0  1
6   0  0  0  1  0  0  0  1  0
7   0  0  0  0  1  0  1  0  1
8   0  0  0  0  0  1  0  1  0
```

```cpp
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
```

### 3. `generateGridDFS()`

- Orchestrates maze generation:
  - Calls `generateMazeUsingDFS()` to create the maze structure.
  - Updates the adjacency matrix.
  - Assigns random letters to cells.
  - Prepares cells for word placement.

```cpp
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
```

### 4. `generateMazeUsingDFS()`

- Core maze generation algorithm:
  - Initializes all cells with walls.
  - Starts from a random position.
  - Uses a stack to track the path.

#### For Each Cell:

1. Gets unvisited neighbors using `getUnvisitedNeighbors()`.
2. Randomly selects a neighbor.
3. Removes the wall between cells using `removeWall()`.
4. Marks the new cell as visited.
5. Continues until no unvisited cells remain.

```cpp
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
```



### Working Together

#### Initialization:

1. Grid is created.
2. Adjacency matrix is initialized.
3. All cells start with four walls.

#### Maze Generation:

1. DFS starts from a random cell.
2. For the current cell:
   - Gets unvisited neighbors.
   - Randomly picks one.
   - Removes the wall between them.
   - Updates the adjacency matrix.
3. Process continues until all cells are visited.

#### Final Steps:

1. Adjacency matrix updated to reflect all connections.
2. Letters assigned to cells.

## Word Placement:

## `placeWordsInGrid()` Function

### Process:
1. Iterates through dictionary words.
2. For each word longer than 2 letters:
   - Makes multiple attempts to place it.
   - Tries different positions and directions.
   - Checks if placement is valid.
3. Updates walls and weights when the word is successfully placed.


```cpp
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
```


---

## Pathfinding Algorithms

### Search Algorithms

#### BFS Search

- Uses **queue** for level-by-level exploration.
- Converts grid positions to indices for adjacency matrix.
- Tracks all paths that reach the end.
- Returns the path with the best word score or shortest length.

#### DFS Search

- Uses **recursive approach** with backtracking.
- Explores paths deeply before backtracking.
- Uses adjacency matrix for connections.
- Collects all possible paths to find the optimal one.

#### Dijkstra's Algorithm

- Uses a **priority queue** for optimal path selection.
- Considers word weights in path selection.
- Finds the shortest weighted path to the end.


---

## Scoring System

### `calculateScore()`

```cpp
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
```
- Scoring criteria:
1. Words found in path: +2 points per letter
2. Path length penalty: -1 point per cell
3. Encourages finding longer words
4. Discourages unnecessarily long paths

### `countWordsInPath()`

```cpp
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
```

- Counts number of dictionary words in path
- Used for path evaluation and scoring
---

## Versus AI Gameplay

### Main Function

**Process:**

1. **Player's Turn:**
    - The player navigates the maze manually.

2. **AI's Turn:**
    - The AI uses the selected difficulty level to find a path.

3. **Score Comparison:**
    - Scores for the player and AI are calculated and compared.

4. **Display Results:**
    - Shows both paths side by side.
    - Displays a detailed score breakdown.
    - Declares the winner based on total score.

### Scoring Example

Player's Path: H-E-L-L-O

- Word "HELLO" found: +10 points (5 letters × 2)
- Path length: -5 points
- Length bonus: +80 points (50 - 5 = 45 × 2)
- **Total:** 85 points

AI's Path: W-O-R-D

- Word "WORD" found: +8 points (4 letters × 2)
- Path length: -4 points
- Length bonus: +82 points (50 - 4 = 46 × 2)
- **Total:** 86 points

**Result:** AI wins by 1 point!

### AI Difficulty Levels

#### 1. Easy Difficulty - First Valid Path

- Uses simple DFS to find the first valid path.
- Stops as soon as the end position is reached.

#### 2. Medium Difficulty - Shortest Path

- Uses BFS to guarantee the shortest path.
- Explores level by level, returning the shortest valid path.

#### 3. Hard Difficulty - Optimal Path (DFS)

**How it works:**

- Uses DFS to find **all possible paths**.
- For each path:
  - Counts words found.
  - Calculates score.
  - Stores path length.
- Optimizes for:
  - Maximum word count.
  - Highest score.
  - Shortest path (as a tiebreaker).

**Outcome:**

- Most intelligent but slowest solution.

```cpp
std::vector<std::pair<int, int>> Game::bfsSearch() {
    allPaths.clear();
    int totalCells = gridSize * gridSize;
    std::vector<bool> visited(totalCells, false);
    
    // Convert start and end positions to indices
    int startIndex = coordToIndex(startPos.first, startPos.second);
    int endIndex = coordToIndex(endPos.first, endPos.second);
    
    std::queue<std::vector<int>> pathQueue;
    pathQueue.push({startIndex});
    visited[startIndex] = true;
    
    while (!pathQueue.empty()) {
        auto currentPath = pathQueue.front();
        pathQueue.pop();
        
        int currentIndex = currentPath.back();
        
        // Convert current index path to coordinates for visualization
        std::vector<std::pair<int, int>> coordPath;
        for (int idx : currentPath) {
            coordPath.push_back({idx / gridSize, idx % gridSize});
        }
        visualizeTraversal(coordPath);
        
        if (currentIndex == endIndex) {
            int wordCount = countWordsInPath(coordPath);
            int score = calculateScore(coordPath);
            allPaths.emplace_back(coordPath, wordCount, score);
            continue;  // Continue searching for other possible paths
        }
        
        // Check all possible connections in adjacency matrix
        for (int nextIndex = 0; nextIndex < totalCells; nextIndex++) {
            if (adjacencyMatrix[currentIndex][nextIndex] && !visited[nextIndex]) {
                visited[nextIndex] = true;
                auto newPath = currentPath;
                newPath.push_back(nextIndex);
                pathQueue.push(newPath);
            }
        }
    }
    
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
```



