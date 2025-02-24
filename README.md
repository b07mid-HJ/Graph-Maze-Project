# Générateur de Labyrinthe avec IA

Ce projet implémente un générateur de labyrinthe avec recherche de mots et un mode versus IA. Le labyrinthe est généré procéduralement et contient des mots cachés que les joueurs doivent trouver en traçant des chemins.

## Architecture

### Structure de Base
- **Grille** : Matrice de cellules où chaque cellule contient :
  - 4 murs (haut, droite, bas, gauche)
  - Une lettre
  - Des états (visité, fait partie du chemin)
- **Matrice d'Adjacence** : Représente les connexions possibles entre les cellules

### Génération du Labyrinthe

1. **Initialisation**
   - Création d'une grille vide de taille N×N
   - Toutes les cellules commencent avec 4 murs

2. **Algorithme DFS**
   - Commence à une position aléatoire
   - Pour chaque cellule non visitée :
     - Choisit un voisin aléatoire non visité
     - Supprime le mur entre les cellules
     - Marque la nouvelle cellule comme visitée
   - Continue jusqu'à ce que toutes les cellules soient visitées

3. **Placement des Mots**
   - Sélectionne des mots du dictionnaire
   - Place les mots dans des directions aléatoires
   - Crée des chemins spéciaux pour les mots (supprime les murs)

## Mode Versus IA

### Déroulement
1. **Tour du Joueur**
   - Navigation manuelle dans le labyrinthe
   - Contrôles WASD pour se déplacer
   - Score basé sur les mots trouvés

2. **Tour de l'IA**
   - Utilise différents algorithmes selon le niveau
   - Trouve des chemins optimisés pour les mots

### Niveaux de Difficulté

1. **Facile**
   - Trouve le premier chemin valide
   - Utilise DFS simple
   - S'arrête au premier succès

2. **Moyen**
   - Trouve le chemin le plus court
   - Utilise BFS
   - Optimise la longueur du chemin

3. **Difficile**
   - Trouve le chemin optimal
   - Utilise DFS avancé
   - Optimise pour :
     - Maximum de mots trouvés
     - Meilleur score
     - Chemin le plus court

### Système de Score
- +2 points par lettre dans les mots trouvés
- -1 point par cellule traversée
- Bonus de longueur : (50 - longueur_chemin) × 2

## Exemple de Partie

```
Grille de Jeu :      
+---+---+---+
|H E|L L|O  |
+   +   +---+
|W O|R D|   |
+---+   +---+
|   |   |   |
+---+---+---+

Score du Joueur : 85
- Mot "HELLO" : +10
- Longueur : -5
- Bonus : +80

Score de l'IA : 86
- Mot "WORD" : +8
- Longueur : -4
- Bonus : +82
```

## Références

### Algorithmes de Génération
- [Maze Generation Algorithm - Depth First Search](https://www.algosome.com/articles/maze-generation-depth-first.html) - Description détaillée de l'algorithme DFS utilisé pour la génération du labyrinthe
- [Maze Generation: Recursive Backtracking](http://weblog.jamisbuck.org/2010/12/27/maze-generation-recursive-backtracking) - Implémentation alternative de l'algorithme de retour sur trace
- [Think Labyrinth: Maze Algorithms](http://www.astrolog.org/labyrnth/algrithm.htm) - Collection d'algorithmes de génération de labyrinthes

### Algorithmes de Pathfinding
- [Dijkstra's Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) - Algorithme utilisé pour trouver le chemin optimal pondéré
- [Breadth First Search vs Depth First Search](https://www.geeksforgeeks.org/difference-between-bfs-and-dfs/) - Analyse comparative des algorithmes BFS et DFS utilisés dans le projet
- [Visualizing Dijkstra's Algorithm](https://www.redblobgames.com/pathfinding/dijkstra/) - Explication visuelle de l'algorithme de Dijkstra

### Intelligence Artificielle
- [Game AI Pro](http://www.gameaipro.com/) - Collection de techniques d'IA pour les jeux
- [Pathfinding for Games](https://www.redblobgames.com/pathfinding/tower-defense/) - Implémentation d'algorithmes de pathfinding dans les jeux




