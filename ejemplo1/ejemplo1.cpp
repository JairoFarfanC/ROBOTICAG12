#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	connect(horizontalSlider, SIGNAL(valueChanged()), this, SLOT(doSlide()) );

	static int cSpeed=500;
	timer.start(500);l

	horizontalSlider->setRange(0,5000); //max cada 5 segundos cuenta
	horizontalSlider->setValue(cSpeed);


	//reemplaza por lectura de slider
}

void ejemplo1::doButton()
{
	static bool counting = true;

	qDebug() << "click on button";
	if (counting == true) {
		timer.stop();
		//change button label to start
		button->setText("Start");
		counting = false;
	} else {
		timer.start(500);
		//change button label to stop
		button->setText("Stop");
		counting = true;
	}

}

void ejemplo1::doCount(){
	static int value = 0;
	lcdNumber->display(value++);
}

int ejemplo1::doSlide()
{


}





