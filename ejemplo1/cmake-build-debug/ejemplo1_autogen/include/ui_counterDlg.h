/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QPushButton *stop_button;
    QLCDNumber *lcdNumber;
    QSlider *horizontalSlider;
    QPushButton *reset_button;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName("Counter");
        Counter->resize(504, 300);
        stop_button = new QPushButton(Counter);
        stop_button->setObjectName("stop_button");
        stop_button->setGeometry(QRect(50, 200, 161, 71));
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName("lcdNumber");
        lcdNumber->setGeometry(QRect(50, 40, 421, 91));
        horizontalSlider = new QSlider(Counter);
        horizontalSlider->setObjectName("horizontalSlider");
        horizontalSlider->setGeometry(QRect(170, 160, 160, 16));
        horizontalSlider->setOrientation(Qt::Horizontal);
        reset_button = new QPushButton(Counter);
        reset_button->setObjectName("reset_button");
        reset_button->setGeometry(QRect(290, 200, 151, 71));

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QCoreApplication::translate("Counter", "Counter", nullptr));
        stop_button->setText(QCoreApplication::translate("Counter", "STOP", nullptr));
        reset_button->setText(QCoreApplication::translate("Counter", "RESET", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
