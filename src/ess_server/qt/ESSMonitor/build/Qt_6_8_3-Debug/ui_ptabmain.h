/********************************************************************************
** Form generated from reading UI file 'ptabmain.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PTABMAIN_H
#define UI_PTABMAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_pTabMain
{
public:
    QLabel *pMap;
    QWidget *pZone1;
    QWidget *pZone4;
    QWidget *pZone5;
    QWidget *pZone2;
    QWidget *pZone6;
    QWidget *pZone3;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *pBattery3;
    QWidget *widget_7;
    QWidget *widget_8;
    QWidget *widget_9;
    QVBoxLayout *pBattery2;
    QWidget *widget_4;
    QWidget *widget_5;
    QWidget *widget_6;
    QVBoxLayout *pBattery1;
    QWidget *widget;
    QWidget *widget_2;
    QWidget *widget_3;
    QWidget *verticalLayoutWidget_9;
    QVBoxLayout *verticalLayout_7;
    QVBoxLayout *verticalLayout_8;
    QLabel *label;
    QHBoxLayout *horizontalLayout_7;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_7;
    QLabel *label_8;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_12;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_13;
    QLabel *label_14;
    QHBoxLayout *horizontalLayout_10;
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_15;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_6;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_10;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_5;

    void setupUi(QWidget *pTabMain)
    {
        if (pTabMain->objectName().isEmpty())
            pTabMain->setObjectName("pTabMain");
        pTabMain->resize(676, 487);
        pMap = new QLabel(pTabMain);
        pMap->setObjectName("pMap");
        pMap->setGeometry(QRect(20, 10, 371, 251));
        pMap->setPixmap(QPixmap(QString::fromUtf8("img/ess_map.jpg")));
        pMap->setScaledContents(true);
        pZone1 = new QWidget(pTabMain);
        pZone1->setObjectName("pZone1");
        pZone1->setGeometry(QRect(30, 20, 120, 121));
        pZone4 = new QWidget(pTabMain);
        pZone4->setObjectName("pZone4");
        pZone4->setGeometry(QRect(30, 140, 120, 121));
        pZone5 = new QWidget(pTabMain);
        pZone5->setObjectName("pZone5");
        pZone5->setGeometry(QRect(150, 140, 120, 121));
        pZone2 = new QWidget(pTabMain);
        pZone2->setObjectName("pZone2");
        pZone2->setGeometry(QRect(150, 20, 120, 121));
        pZone6 = new QWidget(pTabMain);
        pZone6->setObjectName("pZone6");
        pZone6->setGeometry(QRect(270, 140, 120, 121));
        pZone3 = new QWidget(pTabMain);
        pZone3->setObjectName("pZone3");
        pZone3->setGeometry(QRect(270, 20, 120, 121));
        horizontalLayoutWidget = new QWidget(pTabMain);
        horizontalLayoutWidget->setObjectName("horizontalLayoutWidget");
        horizontalLayoutWidget->setGeometry(QRect(20, 280, 371, 181));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        pBattery3 = new QVBoxLayout();
        pBattery3->setObjectName("pBattery3");
        widget_7 = new QWidget(horizontalLayoutWidget);
        widget_7->setObjectName("widget_7");

        pBattery3->addWidget(widget_7);

        widget_8 = new QWidget(horizontalLayoutWidget);
        widget_8->setObjectName("widget_8");

        pBattery3->addWidget(widget_8);

        widget_9 = new QWidget(horizontalLayoutWidget);
        widget_9->setObjectName("widget_9");

        pBattery3->addWidget(widget_9);


        horizontalLayout->addLayout(pBattery3);

        pBattery2 = new QVBoxLayout();
        pBattery2->setObjectName("pBattery2");
        widget_4 = new QWidget(horizontalLayoutWidget);
        widget_4->setObjectName("widget_4");

        pBattery2->addWidget(widget_4);

        widget_5 = new QWidget(horizontalLayoutWidget);
        widget_5->setObjectName("widget_5");

        pBattery2->addWidget(widget_5);

        widget_6 = new QWidget(horizontalLayoutWidget);
        widget_6->setObjectName("widget_6");

        pBattery2->addWidget(widget_6);


        horizontalLayout->addLayout(pBattery2);

        pBattery1 = new QVBoxLayout();
        pBattery1->setObjectName("pBattery1");
        widget = new QWidget(horizontalLayoutWidget);
        widget->setObjectName("widget");

        pBattery1->addWidget(widget);

        widget_2 = new QWidget(horizontalLayoutWidget);
        widget_2->setObjectName("widget_2");

        pBattery1->addWidget(widget_2);

        widget_3 = new QWidget(horizontalLayoutWidget);
        widget_3->setObjectName("widget_3");

        pBattery1->addWidget(widget_3);


        horizontalLayout->addLayout(pBattery1);

        verticalLayoutWidget_9 = new QWidget(pTabMain);
        verticalLayoutWidget_9->setObjectName("verticalLayoutWidget_9");
        verticalLayoutWidget_9->setGeometry(QRect(440, 30, 211, 431));
        verticalLayout_7 = new QVBoxLayout(verticalLayoutWidget_9);
        verticalLayout_7->setObjectName("verticalLayout_7");
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName("verticalLayout_8");
        label = new QLabel(verticalLayoutWidget_9);
        label->setObjectName("label");

        verticalLayout_8->addWidget(label);


        verticalLayout_7->addLayout(verticalLayout_8);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        label_2 = new QLabel(verticalLayoutWidget_9);
        label_2->setObjectName("label_2");

        horizontalLayout_4->addWidget(label_2);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        label_7 = new QLabel(verticalLayoutWidget_9);
        label_7->setObjectName("label_7");

        horizontalLayout_3->addWidget(label_7);

        label_8 = new QLabel(verticalLayoutWidget_9);
        label_8->setObjectName("label_8");

        horizontalLayout_3->addWidget(label_8);


        verticalLayout->addLayout(horizontalLayout_3);


        horizontalLayout_7->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        label_12 = new QLabel(verticalLayoutWidget_9);
        label_12->setObjectName("label_12");

        horizontalLayout_5->addWidget(label_12);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        label_13 = new QLabel(verticalLayoutWidget_9);
        label_13->setObjectName("label_13");

        horizontalLayout_6->addWidget(label_13);

        label_14 = new QLabel(verticalLayoutWidget_9);
        label_14->setObjectName("label_14");

        horizontalLayout_6->addWidget(label_14);


        verticalLayout_2->addLayout(horizontalLayout_6);


        horizontalLayout_7->addLayout(verticalLayout_2);


        verticalLayout_7->addLayout(horizontalLayout_7);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName("horizontalLayout_10");
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName("verticalLayout_5");
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName("verticalLayout_6");
        label_15 = new QLabel(verticalLayoutWidget_9);
        label_15->setObjectName("label_15");

        verticalLayout_6->addWidget(label_15);


        verticalLayout_5->addLayout(verticalLayout_6);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName("horizontalLayout_9");
        label_6 = new QLabel(verticalLayoutWidget_9);
        label_6->setObjectName("label_6");

        horizontalLayout_9->addWidget(label_6);


        verticalLayout_5->addLayout(horizontalLayout_9);


        horizontalLayout_10->addLayout(verticalLayout_5);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName("verticalLayout_4");
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName("verticalLayout_3");
        label_10 = new QLabel(verticalLayoutWidget_9);
        label_10->setObjectName("label_10");

        verticalLayout_3->addWidget(label_10);


        verticalLayout_4->addLayout(verticalLayout_3);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName("horizontalLayout_8");
        label_5 = new QLabel(verticalLayoutWidget_9);
        label_5->setObjectName("label_5");

        horizontalLayout_8->addWidget(label_5);


        verticalLayout_4->addLayout(horizontalLayout_8);


        horizontalLayout_10->addLayout(verticalLayout_4);


        verticalLayout_7->addLayout(horizontalLayout_10);

        verticalLayout_7->setStretch(0, 2);
        verticalLayout_7->setStretch(1, 5);
        verticalLayout_7->setStretch(2, 5);

        retranslateUi(pTabMain);

        QMetaObject::connectSlotsByName(pTabMain);
    } // setupUi

    void retranslateUi(QWidget *pTabMain)
    {
        pTabMain->setWindowTitle(QCoreApplication::translate("pTabMain", "Form", nullptr));
        pMap->setText(QString());
        label->setText(QCoreApplication::translate("pTabMain", "\354\213\234\352\260\204", nullptr));
        label_2->setText(QCoreApplication::translate("pTabMain", "\354\230\250\353\217\204", nullptr));
        label_7->setText(QCoreApplication::translate("pTabMain", "\354\230\250\353\217\204", nullptr));
        label_8->setText(QCoreApplication::translate("pTabMain", "\342\204\203", nullptr));
        label_12->setText(QCoreApplication::translate("pTabMain", "\354\212\265\353\217\204", nullptr));
        label_13->setText(QCoreApplication::translate("pTabMain", "\354\212\265\353\217\204", nullptr));
        label_14->setText(QCoreApplication::translate("pTabMain", "%", nullptr));
        label_15->setText(QCoreApplication::translate("pTabMain", "\352\260\200\354\212\244", nullptr));
        label_6->setText(QCoreApplication::translate("pTabMain", "-", nullptr));
        label_10->setText(QCoreApplication::translate("pTabMain", "\353\260\234\354\227\264", nullptr));
        label_5->setText(QCoreApplication::translate("pTabMain", "-", nullptr));
    } // retranslateUi

};

namespace Ui {
    class pTabMain: public Ui_pTabMain {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PTABMAIN_H
