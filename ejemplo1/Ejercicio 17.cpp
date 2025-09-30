/*EJERCICIO #17
Create a randomly connected graph using std::map
declare a struct Node{ int id; std::vector<int> links;} ;
declare the map: std::map<int, Node> graph;
initialize it: 
for(int i=0; i<100; i++)
  graph.emplace(std::make_pair(i, Node{i, std::vector<int>()}));
connect it
for(auto &[key, value] : graph)
  {
      vecinos = get_random_number_between 0 and 5
      for(int j=0; j<vecinos; j++)
         value.links.emplace_back( random_number_between 0 and 99);
  }*/

#include <iostream>
#include <map>
#include <vector>
#include <cstdlib>
#include <ctime>

// Estructura que representa un nodo en el grafo
struct Node {
    int id;                 // Identificador del nodo
    std::vector<int> links; // Lista de enlaces (vecinos) del nodo
};

// Función para generar un número aleatorio entre min y max
int get_random_number_between(int min, int max) {
    return min + rand() % (max - min + 1);
}

int main() {
    // Inicializar la semilla de los números aleatorios
    srand(static_cast<unsigned int>(time(0)));

    // Crear el grafo como un map de enteros a nodos
    std::map<int, Node> graph;

    // Inicializar los nodos del grafo
    for (int i = 0; i < 100; i++) {
        graph.emplace(std::make_pair(i, Node{i, std::vector<int>()}));
    }

    // Conectar los nodos aleatoriamente
    for (auto &[key, value] : graph) {
        // Obtener un número aleatorio de vecinos entre 0 y 5
        int vecinos = get_random_number_between(0, 5);

        // Agregar enlaces aleatorios a los vecinos
        for (int j = 0; j < vecinos; j++) {
            // Asegurarnos de que no agregamos el nodo como vecino de sí mismo
            int random_neighbor;
            do {
                random_neighbor = get_random_number_between(0, 99);
            } while (random_neighbor == key); // No permitir que el nodo se conecte a sí mismo

            value.links.emplace_back(random_neighbor);
        }
    }

    // Imprimir los enlaces de cada nodo (para verificar que el grafo se ha creado correctamente)
    for (const auto &[key, value] : graph) {
        std::cout << "Nodo " << key << " está conectado a: ";
        for (int neighbor : value.links) {
            std::cout << neighbor << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}

/*
Estructura Node: Esta estructura representa a cada nodo del grafo. Tiene un id único para cada nodo 
y un vector links que almacena los identificadores de los nodos a los que está conectado (sus vecinos).

Inicialización del grafo: Creamos un std::map<int, Node> donde cada clave es un identificador único (id) 
para cada nodo. Usamos un bucle for para inicializar los 100 nodos, y cada uno tiene un id y un vector 
vacío de enlaces.

Conexión aleatoria de nodos: En el bucle que recorre cada nodo del grafo, generamos un número aleatorio 
de vecinos (entre 0 y 5). Luego, por cada vecino, seleccionamos aleatoriamente otro nodo en el rango de
0 a 99, asegurándonos de no conectar un nodo a sí mismo.

Impresión de los resultados: Finalmente, mostramos las conexiones de cada nodo (los nodos a los que está
conectado) para verificar que la conexión aleatoria se haya realizado correctamente.
*/