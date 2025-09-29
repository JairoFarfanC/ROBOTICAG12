#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);



	show();
	connect(stop_button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(reset_button, SIGNAL(clicked()), this, SLOT(reset()) );
	connect(horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(doSlide()) );

	/**
	 * MODIFICADO A PARTIR DE LA SEGUNDA PARTE DEL EJERCICIO 13, DESPUES DE LAS PREGUNTAS
	 * Antes teniamos: timer.connect(std::bind(&ejemplo1::doCount, this));
	 * Ahora aniadimos el parametro '1' que sera la cantidad en la que se ira incrementando el contador
	 */
	timer.connect(std::bind(&ejemplo1::doCount, this,doRandom()));

	static int cSpeed=500;
	timer.start(cSpeed);

	horizontalSlider->setRange(0,5000); //max cada 5 segundos cuenta
	horizontalSlider->setTickInterval(500);
	cont = 0;

	//reemplaza por lectura de slider

}

void ejemplo1::doButton()
{
	static bool counting = true;

	qDebug() << "click on button";
	if (counting == true) {
		timer.stop();
		//change button label to start
		stop_button->setText("Start");
		counting = false;
	} else {
		timer.start(500);
		//change button label to stop
		stop_button->setText("Stop");
		counting = true;
	}

}

/**
 * ANIADIDO A PARTIR DE LA SEGUNDA PARTE DEL EJERCICIO 13, DESPUES DE LAS PREGUNTAS
 * @param step he aniadido este parametro para poder indicar la cantidad que se suma en el contador en cada momento
 * ya que el ejercicio me pedia aniadir un parametro en este metodo para tener mas flexibilidad y poder modificarlo cuando queramos.
*/
void ejemplo1::doCount(int step){
	cont +=step;
	lcdNumber->display(cont);
}

void ejemplo1::doSlide()
{
	qDebug() << "slide detected";
	timer.setInterval(horizontalSlider->value());
}

void ejemplo1::reset()
{
	qDebug() << "doing reset";
	cont = 0;
}

/**
 * SOLUCION EJERCICIO 14
 * @return El ejercicio 14 me pide inplmentar el 'random' de alguna forma en mi metodo, he pensado en implementarlo en el
 * numero que se va incrementando en el contador, con un rango del 1 al 3, para ello he configurado este metodo y cada linea
 * encima explica que hace cada cosa, se configura todo para generar el numero y se llama a la funcion dice(gen) que es la que
 * contiene ese numero random en cada ejecucion del 1 al 3 en este caso. Luego este metodo es llamado arriba en el conector del Timer
 * y este metodo al retornar un entero sera este el que se le pase al metodo doCount().
*/
int ejemplo1::doRandom() {

	// 1. Create a random device (seed source)
	std::random_device rd;
	// 2. Create a random number generator
	std::mt19937 gen(rd());
	// 3. Create distribution (solo enteros del 1 al 3)
	std::uniform_int_distribution<int> dice(1, 3);

	return dice(gen);

}

void ejemplo1::testContainers() {
    std::cout << "=== Test de Velocidad ===" << std::endl;

    // ---------- VECTOR ----------
    size_t N = 1'000'000; // tamaño (ajústalo según tu PC)
    std::vector<int> v(N);

    // Generador de aleatorios
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(1, 100);

    // Rellenar vector
    for (size_t i = 0; i < N; ++i) {
        v[i] = dist(gen);
    }

    // Sort
    auto start = std::chrono::high_resolution_clock::now();
    std::sort(v.begin(), v.end());
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Sort took " << duration << " ms" << std::endl;

    // Min element
    start = std::chrono::high_resolution_clock::now();
    auto minIt = std::min_element(v.begin(), v.end());
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "Min element took " << duration << " µs, value = " << *minIt << std::endl;

    // Shuffle
    start = std::chrono::high_resolution_clock::now();
    std::shuffle(v.begin(), v.end(), gen);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Shuffle took " << duration << " ms" << std::endl;

    // Copy
    start = std::chrono::high_resolution_clock::now();
    std::vector<int> v_copy = v;
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Copy took " << duration << " ms" << std::endl;


    // ---------- MAP ----------
    std::cout << "\n=== Test Map ===" << std::endl;
    std::map<int, int> m;

    // Insert N elementos
    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < N; ++i) {
        m[i] = dist(gen);
    }
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Insert " << N << " elements took " << duration << " ms" << std::endl;

    // Buscar N/10 claves
    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < N/10; ++i) {
        auto it = m.find(i);
        if (it == m.end()) {
            std::cerr << "Key not found: " << i << std::endl;
        }
    }
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Find " << N/10 << " elements took " << duration << " ms" << std::endl;
}





