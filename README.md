# Génération de Labyrinthe, Recherche de Chemin et Gameplay IA

Ce document explique le processus de génération de labyrinthe, les algorithmes de recherche de chemin, les systèmes de score et les mécaniques de gameplay de l'IA. Nous explorons également le fonctionnement des niveaux de difficulté de l'IA en mode versus.

---

## Processus de Génération du Labyrinthe

### 1. Configuration Initiale (Constructeur)

- **Crée une grille** de taille `gridSize × gridSize`.
- Initialise la **matrice d'adjacence**.
- Démarre le **processus de génération du labyrinthe**.

```cpp
Game::Game(int size) : gridSize(size), score(0), startPos({0,0}), endPos({0,0}) {
    grid.resize(size, std::vector<Cell>(size));
    initializeAdjacencyMatrix();
    generateGridDFS();
}
```

### 2. `initializeAdjacencyMatrix()`

- Crée une matrice `gridSize² × gridSize²`.
- Chaque cellule `(i,j)` représente une connexion potentielle entre les cellules `i` et `j`.
- Initialement, toutes les connexions sont définies à `false`.

**Exemple :** Pour une grille `5 × 5`, crée une matrice `25 × 25`.
2. Indexation des Cellules :

```cpp
int coordToIndex(int x, int y) const { return x * gridSize + y; }
```

- Convertit les coordonnées 2D en index 1D
Pour une grille 5×5 :
  - Cellule (0,0) → index 0
  - Cellule (0,1) → index 1
  - Cellule (1,0) → index 5
  - Cellule (4,4) → index 24

```
Grille :         Indices des Cellules :
+--+--+--+      +--+--+--+
|A |B |C |      |0 |1 |2 |
+--+--+--+      +--+--+--+
|D |E |F |      |3 |4 |5 |
+--+--+--+      +--+--+--+
|G |H |I |      |6 |7 |8 |
+--+--+--+      +--+--+--+

Matrice d'Adjacence (9×9) :
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
    
    // Remplit la matrice d'adjacence basée sur l'information des murs
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            int currentIndex = i * gridSize + j;
            
            // Vérifie les quatre directions
            // Haut
            if (i > 0 && !grid[i][j].walls[0] && !grid[i-1][j].walls[2]) {
                int neighborIndex = (i-1) * gridSize + j;
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Droite
            if (j < gridSize-1 && !grid[i][j].walls[1] && !grid[i][j+1].walls[3]) {
                int neighborIndex = i * gridSize + (j+1);
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Bas
            if (i < gridSize-1 && !grid[i][j].walls[2] && !grid[i+1][j].walls[0]) {
                int neighborIndex = (i+1) * gridSize + j;
                adjacencyMatrix[currentIndex][neighborIndex] = true;
                adjacencyMatrix[neighborIndex][currentIndex] = true;
            }
            // Gauche
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

- Orchestre la génération du labyrinthe :
  - Appelle `generateMazeUsingDFS()` pour créer la structure du labyrinthe.
  - Met à jour la matrice d'adjacence.
  - Attribue des lettres aléatoires aux cellules.
  - Prépare les cellules pour le placement des mots.

```cpp
void Game::generateGridDFS() {
    generateMazeUsingDFS();
    // Met à jour la matrice d'adjacence après la génération du labyrinthe
    updateAdjacencyMatrix();
    // Initialise toutes les cellules avec des espaces avant de placer les mots
    for (auto& row : grid) {
        for (auto& cell : row) {
            cell.letter = ' ';  // Initialise avec des espaces
        }
    }
    // Place les mots dans la grille
    placeWordsInGrid();
}
```

### 4. `generateMazeUsingDFS()`

- Algorithme principal de génération du labyrinthe :
  - Initialise toutes les cellules avec des murs.
  - Commence à partir d'une position aléatoire.
  - Utilise une pile pour suivre le chemin.

#### Pour Chaque Cellule :

1. Obtient les voisins non visités avec `getUnvisitedNeighbors()`.
2. Sélectionne aléatoirement un voisin.
3. Supprime le mur entre les cellules avec `removeWall()`.
4. Marque la nouvelle cellule comme visitée.
5. Continue jusqu'à ce qu'il n'y ait plus de cellules non visitées.

```cpp
void Game::generateMazeUsingDFS() {
    // Réinitialise toutes les cellules
    for (auto& row : grid) {
        for (auto& cell : row) {
            cell.visited = false;
            cell.walls[0] = cell.walls[1] = cell.walls[2] = cell.walls[3] = true;
        }
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Commence à partir d'une cellule aléatoire
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

        // Choisit aléatoirement le prochain voisin
        std::uniform_int_distribution<> dis(0, neighbors.size() - 1);
        auto next = neighbors[dis(gen)];
        
        // Supprime le mur entre current et next
        removeWall(current.first, current.second, next.first, next.second);
        
        grid[next.first][next.second].visited = true;
        stack.push(next);
    }
}
```

### Fonctionnement Général

#### Initialisation :

1. La grille est créée.
2. La matrice d'adjacence est initialisée.
3. Toutes les cellules commencent avec quatre murs.

#### Génération du Labyrinthe :

1. DFS commence à partir d'une cellule aléatoire.
2. Pour la cellule courante :
   - Obtient les voisins non visités.
   - En choisit un aléatoirement.
   - Supprime le mur entre eux.
   - Met à jour la matrice d'adjacence.
3. Le processus continue jusqu'à ce que toutes les cellules soient visitées.

#### Étapes Finales :

1. La matrice d'adjacence est mise à jour pour refléter toutes les connexions.
2. Les lettres sont attribuées aux cellules.

## Placement des Mots :

## Fonction `placeWordsInGrid()`

### Processus :
1. Parcourt les mots du dictionnaire.
2. Pour chaque mot de plus de 2 lettres :
   - Fait plusieurs tentatives de placement.
   - Essaie différentes positions et directions.
   - Vérifie si le placement est valide.
3. Met à jour les murs et les poids lorsque le mot est placé avec succès.

```cpp
void Game::placeWordsInGrid() {
    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (const auto& word : dictionary) {
        if (word.length() > 2) {  // Place uniquement les mots de plus de 2 lettres
            int attempts = 0;
            bool placed = false;
            
            while (!placed && attempts < 10) {
                int x = gen() % gridSize;
                int y = gen() % gridSize;
                
                // Essaie différentes directions
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

## Algorithmes de Recherche de Chemin

### Algorithmes de Recherche

#### Recherche BFS

- Utilise une **file** pour l'exploration niveau par niveau.
- Convertit les positions de la grille en indices pour la matrice d'adjacence.
- Suit tous les chemins qui atteignent la fin.
- Retourne le chemin avec le meilleur score de mots ou la longueur la plus courte.

#### Recherche DFS

- Utilise une **approche récursive** avec retour sur trace.
- Explore les chemins en profondeur avant de revenir en arrière.
- Utilise la matrice d'adjacence pour les connexions.
- Collecte tous les chemins possibles pour trouver l'optimal.

#### Algorithme de Dijkstra

- Utilise une **file prioritaire** pour la sélection optimale du chemin.
- Prend en compte les poids des mots dans la sélection du chemin.
- Trouve le chemin pondéré le plus court jusqu'à la fin.

---

## Système de Score

### `calculateScore()`

```cpp
int Game::calculateScore(const std::vector<std::pair<int, int>>& path) const {
    // Obtient la séquence de lettres dans le chemin
    std::string pathLetters;
    for (const auto& pos : path) {
        pathLetters += grid[pos.first][pos.second].letter;
    }
    
    // Calcule le score basé sur les mots trouvés
    int score = 0;
    for (const auto& word : dictionary) {
        if (pathLetters.find(word) != std::string::npos) {
            score += word.length() * 2;  // Plus de points pour les mots longs
        }
    }
    
    // Soustrait des points pour la longueur du chemin pour favoriser les chemins courts
    score -= path.size();
    return score;
}
```

- Critères de score :
1. Mots trouvés dans le chemin : +2 points par lettre
2. Pénalité de longueur du chemin : -1 point par cellule
3. Encourage à trouver des mots plus longs
4. Décourage les chemins inutilement longs

### `countWordsInPath()`

```cpp
int Game::countWordsInPath(const std::vector<std::pair<int, int>>& path) const {
    // Obtient la séquence de lettres dans le chemin
    std::string pathLetters;
    for (const auto& pos : path) {
        pathLetters += grid[pos.first][pos.second].letter;
    }
    
    // Compte combien de mots du dictionnaire apparaissent dans la séquence du chemin
    int wordCount = 0;
    for (const auto& word : dictionary) {
        if (pathLetters.find(word) != std::string::npos) {
            wordCount++;
        }
    }
    return wordCount;
}
```

- Compte le nombre de mots du dictionnaire dans le chemin
- Utilisé pour l'évaluation du chemin et le scoring

---

## Gameplay Versus IA

### Fonction Principale

**Processus :**

1. **Tour du Joueur :**
    - Le joueur navigue manuellement dans le labyrinthe.

2. **Tour de l'IA :**
    - L'IA utilise le niveau de difficulté sélectionné pour trouver un chemin.

3. **Comparaison des Scores :**
    - Les scores du joueur et de l'IA sont calculés et comparés.

4. **Affichage des Résultats :**
    - Montre les deux chemins côte à côte.
    - Affiche une décomposition détaillée du score.
    - Déclare le gagnant basé sur le score total.

### Exemple de Score

Chemin du Joueur : H-E-L-L-O

- Mot "HELLO" trouvé : +10 points (5 lettres × 2)
- Longueur du chemin : -5 points
- Bonus de longueur : +80 points (50 - 5 = 45 × 2)
- **Total :** 85 points

Chemin de l'IA : W-O-R-D

- Mot "WORD" trouvé : +8 points (4 lettres × 2)
- Longueur du chemin : -4 points
- Bonus de longueur : +82 points (50 - 4 = 46 × 2)
- **Total :** 86 points

**Résultat :** L'IA gagne d'1 point !

### Niveaux de Difficulté de l'IA

#### 1. Difficulté Facile - Premier Chemin Valide

- Utilise un DFS simple pour trouver le premier chemin valide.
- S'arrête dès que la position finale est atteinte.

#### 2. Difficulté Moyenne - Chemin le Plus Court

- Utilise BFS pour garantir le chemin le plus court.
- Explore niveau par niveau, retournant le chemin valide le plus court.

#### 3. Difficulté Difficile - Chemin Optimal (DFS)

**Fonctionnement :**

- Utilise DFS pour trouver **tous les chemins possibles**.
- Pour chaque chemin :
  - Compte les mots trouvés.
  - Calcule le score.
  - Stocke la longueur du chemin.
- Optimise pour :
  - Nombre maximum de mots.
  - Score le plus élevé.
  - Chemin le plus court (comme départage).

**Résultat :**

- Solution la plus intelligente mais la plus lente.

```cpp
std::vector<std::pair<int, int>> Game::bfsSearch() {
    allPaths.clear();
    int totalCells = gridSize * gridSize;
    std::vector<bool> visited(totalCells, false);
    
    // Convertit les positions de départ et de fin en indices
    int startIndex = coordToIndex(startPos.first, startPos.second);
    int endIndex = coordToIndex(endPos.first, endPos.second);
    
    std::queue<std::vector<int>> pathQueue;
    pathQueue.push({startIndex});
    visited[startIndex] = true;
    
    while (!pathQueue.empty()) {
        auto currentPath = pathQueue.front();
        pathQueue.pop();
        
        int currentIndex = currentPath.back();
        
        // Convertit le chemin d'indices actuel en coordonnées pour la visualisation
        std::vector<std::pair<int, int>> coordPath;
        for (int idx : currentPath) {
            coordPath.push_back({idx / gridSize, idx % gridSize});
        }
        visualizeTraversal(coordPath);
        
        if (currentIndex == endIndex) {
            int wordCount = countWordsInPath(coordPath);
            int score = calculateScore(coordPath);
            allPaths.emplace_back(coordPath, wordCount, score);
            continue;  // Continue la recherche d'autres chemins possibles
        }
        
        // Vérifie toutes les connexions possibles dans la matrice d'adjacence
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
    
    // Trie les chemins par nombre de mots, score et longueur
    std::sort(allPaths.begin(), allPaths.end(), 
        [](const PathInfo& a, const PathInfo& b) {
            if (a.wordCount != b.wordCount) return a.wordCount > b.wordCount;
            if (a.score != b.score) return a.score > b.score;
            return a.path.size() < b.path.size();
        });
    
    return allPaths[0].path;
}
```



