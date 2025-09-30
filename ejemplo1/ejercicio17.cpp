//
// Created by jairo on 30/09/2025.
//

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>  // Para poder usar transform

int main() {
    // Creamos un vector 'vs' de strings
    std::vector<std::string> vs = {
        "Victor",
        "Antonio",
        "Jairo"
    };

    // Creamos el vector destino de enteros y le asignamos el mismo tamaño que el de vs
    std::vector<int> vi;
    vi.resize(vs.size());

    // usamos transform: para cada string, calculamos la suma de sus caracteres
    std::transform(vs.begin(), vs.end(), vi.begin(),
        [](const std::string &s) {
            int sum = 0;
            for (unsigned char c : s) {
                sum += c;
            }
            return sum;
        }
    );

    // mostrar el resultado
    for (size_t i = 0; i < vs.size(); ++i) {
        std::cout << vs[i] << " -> " << vi[i] << "\n";
    }

    return 0;
}