#include "game.hpp"
#include <iostream>
#include <limits>

void clearInput() {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

int main() {
    char playAgain;
    do {
        // Get grid size from user
        int size;
        do {
            std::cout << "Enter maze size (4-10): ";
            std::cin >> size;
            clearInput();
        } while (size < 4 || size > 10);
        
        Game game(size);
        game.loadDictionary("dictionary.txt");
        
        // Generate maze and place words
        std::cout << "Generating maze and placing words...\n";
        game.generateGridDFS();
        game.placeWordsInGrid();
        game.setRandomStartEnd();
        
        // Print initial maze
        std::cout << "\nInitial Maze (S = Start, E = End):\n";
        game.printGridWithWalls(true);
        
        // Get algorithm choice from user
        char choice;
        do {
            std::cout << "\nSelect mode:\n";
            std::cout << "1. BFS Algorithm\n";
            std::cout << "2. DFS Algorithm\n";
            std::cout << "3. Dijkstra Algorithm\n";
            std::cout << "4. Manual Play\n";
            std::cout << "5. Versus AI\n";
            std::cout << "Choice (1-5): ";
            std::cin >> choice;
            clearInput();
        } while (choice < '1' || choice > '5');
        
        if (choice == '4') {
            game.startManualGame();
        } else if (choice == '5') {
            char diffChoice;
            do {
                std::cout << "\nSelect AI difficulty:\n";
                std::cout << "1. Easy (First path found)\n";
                std::cout << "2. Medium (Shortest path)\n";
                std::cout << "3. Hard (Optimal path with most words)\n";
                std::cout << "Choice (1-3): ";
                std::cin >> diffChoice;
                clearInput();
            } while (diffChoice < '1' || diffChoice > '3');
            
            Game::Difficulty diff;
            switch (diffChoice) {
                case '1': diff = Game::Difficulty::EASY; break;
                case '2': diff = Game::Difficulty::MEDIUM; break;
                case '3': diff = Game::Difficulty::HARD; break;
            }
            
            game.startVersusAI(diff);
        } else {
            std::string algoName;
            std::vector<std::pair<int, int>> path;
            
            if (choice == '1') {
                algoName = "BFS";
                path = game.findPath(true);
            } else if (choice == '2') {
                algoName = "DFS";
                path = game.findPath(false);
            } else {  // choice == '3'
                algoName = "Dijkstra";
                path = game.findPathDijkstra();
            }
            
            std::cout << "\nFinding path using " << algoName << "...\n";
            
            if (!path.empty()) {
                std::cout << "\nPath found! Final score: " << game.getScore() << "\n";
            } else {
                std::cout << "\nNo path found!\n";
            }
        }
        
        std::cout << "\nPlay again? (Y/N or Q to quit): ";
        std::cin >> playAgain;
        clearInput();
        playAgain = toupper(playAgain);
        system("cls");  // Clear screen before next game
        
    } while (playAgain == 'Y');
    
    return 0;
} 