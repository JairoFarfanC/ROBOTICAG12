#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(stop_button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(reset_button, SIGNAL(clicked()), this, SLOT(reset()) );
	connect(horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(doSlide()) );

	//connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	timer.connect(std::bind(&ejemplo1::doCount, this));

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

void ejemplo1::doCount(){
	lcdNumber->display(cont++);
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




