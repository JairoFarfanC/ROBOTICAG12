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
	timer.connect(std::bind(&ejemplo1::doCount, this,1));

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




