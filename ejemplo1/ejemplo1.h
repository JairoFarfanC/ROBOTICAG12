#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <QTimer>
#include <QSlider>


class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
    ejemplo1();

public slots:
    void doButton();
    void doCount();
    int doSlide();

private:
    QTimer timer;
    QSlider *slider;
};

#endif // ejemplo1_H
