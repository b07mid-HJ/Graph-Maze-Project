# Product Requirement Document: Labyrinthe de Mots

## **Introduction**

The "Labyrinthe de Mots" project aims to develop an interactive word maze game where players navigate through a grid of letters, forming valid words while adhering to specific rules to maximize their score. The grid is represented as a graph, enabling efficient management of movements, connections, and word validations.

## **Goals and Objectives**

- Provide an engaging and educational game experience that challenges players' vocabulary and problem-solving skills.
- Leverage graph-based algorithms for efficient grid navigation and word validation.
- Implement a flexible system that allows different levels of difficulty and dynamic grid generation.

---

## **Main Features**

### **1. Grid Generation**

- **Dynamic Generation**: The grid is dynamically generated based on a provided dictionary of words.
- **Graph Representation**: Each cell in the grid is represented as a node, and valid moves between cells are represented as edges.
- **Flexible Grid Shapes**: The grid can take various shapes (not necessarily square) and adjusts based on criteria such as dictionary word length or game difficulty.

### **2. Gameplay Mechanics**

- **Starting Position**: The player starts from a random or user-defined starting cell.
- **Movement Rules**: Players move through the grid along the graph’s connections, forming words by traversing letters.
- **Word Formation**: Letters from traversed cells are collected to create words.

### **3. Word Validation**

- **Dictionary Check**: Formed words are validated against the provided dictionary.
- **Reusable Cells**: Once a word is validated, the cells used to form the word are freed and can be reused for other words.

### **4. Scoring System**

- **Word Length**: Players earn points based on the number of letters in a word (e.g., 1 point per letter).
- **Difficulty Level**: Rare or long words are awarded more points.
- **Path Efficiency**: Shorter paths to the target provide additional bonuses.
- **Additional Bonuses**:
  - Forming multiple words in a single path.
  - Utilizing special cells that provide extra points.

---

## **Technical Requirements**

### **1. Algorithms**

- **Graph Traversal**: Implement BFS and DFS for grid navigation.
- **Shortest Path**: Use Dijkstra's algorithm or similar to compute optimal paths for bonus scoring.

### **2. Data Structures**

- **Graph**: Represents the grid with nodes (cells) and edges (connections).
- **Trie/HashMap**: Optimizes dictionary word lookup for validation.

### **3. Input and Output**

- **Input**: A text file containing a dictionary of words (one word per line).
- **Output**: Game results, including the player's score and formed words.

### **4. Platform and Language**

- The application must be developed in **C++**.
- The grid visualization can be implemented in CLI or using libraries like **Qt** for a graphical interface.

---

## **Game Flow**

1. **Initialization**:
   - Load the dictionary.
   - Generate the grid based on difficulty level.
   - Set the starting and ending positions.
2. **Gameplay**:
   - Allow the player to move and form words.
   - Validate words and update scores.
3. **Completion**:
   - Calculate the final score.
   - Display results and optional achievements.
   .

---
