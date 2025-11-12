/*
EJERCICIO #16
In C++20, you can initialize a struct using the names of the fields,
  struct A
  {  float init = 0, end = 0, step = 0;  };
  auto B = A{.init=1, .end=2, .step=3}
*/

#include <iostream>

struct A {
    float init;
    float end;
    float step;
};

int main() {
    auto B = A{.init = 1, .end = 2, .step = 3};  
    
    std::cout << "init: " << B.init << std::endl;
    std::cout << "end: " << B.end << std::endl;
    std::cout << "step: " << B.step << std::endl;

    return 0;
}

/*En este código, definí una estructura llamada A con tres campos de tipo float: init, end y step.
  En lugar de inicializarlos con valores predeterminados, los dejé sin inicializar, lo que convierte 
  a la estructura en un tipo agregado en C++. Luego, utilicé la característica de inicialización 
  estructurada de C++20 para crear una instancia de la estructura (B) y asignar valores específicos 
  a cada campo usando sus nombres (.init = 1, .end = 2, .step = 3). Finalmente, imprimí los valores 
  de los campos de la estructura para verificar que se habían asignado correctamente.*/

  