#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <QTimer>
#include <QSlider>

#include "Timer.h"


class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
    ejemplo1();

public slots:
    void doButton();
    void doCount(int step);
    void doSlide();
    void reset();

private:
    Timer timer;
    //QTimer timer;
    QSlider *slider;
    int cont = 0;
};



#endif // ejemplo1_H
