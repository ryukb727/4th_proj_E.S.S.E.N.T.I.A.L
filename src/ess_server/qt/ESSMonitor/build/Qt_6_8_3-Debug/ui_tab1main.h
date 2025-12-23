/********************************************************************************
** Form generated from reading UI file 'tab1main.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TAB1MAIN_H
#define UI_TAB1MAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Tab1Main
{
public:
    QHBoxLayout *horizontalLayout_2;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_9;
    QVBoxLayout *pMapLayout;
    QHBoxLayout *pBatteryrackLayout;
    QVBoxLayout *pBatteryRack1;
    QVBoxLayout *pBatteryRack2;
    QVBoxLayout *pBatteryRack3;
    QVBoxLayout *pInfoLayout;
    QSpacerItem *verticalSpacer_3;
    QVBoxLayout *verticalLayout_8;
    QLabel *pLabelTime;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_7;
    QLabel *label;
    QHBoxLayout *horizontalLayout_7;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *pLabelTemp;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_12;
    QHBoxLayout *horizontalLayout_6;
    QLabel *pLabelHumi;
    QVBoxLayout *verticalLayout_10;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_10;
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_15;
    QHBoxLayout *horizontalLayout_9;
    QLabel *pLabelGas;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_10;
    QHBoxLayout *horizontalLayout_8;
    QLabel *pLabelThermal;
    QSpacerItem *verticalSpacer_2;

    void setupUi(QWidget *Tab1Main)
    {
        if (Tab1Main->objectName().isEmpty())
            Tab1Main->setObjectName("Tab1Main");
        Tab1Main->resize(676, 487);
        horizontalLayout_2 = new QHBoxLayout(Tab1Main);
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setObjectName("verticalLayout_9");
        pMapLayout = new QVBoxLayout();
        pMapLayout->setObjectName("pMapLayout");

        verticalLayout_9->addLayout(pMapLayout);

        pBatteryrackLayout = new QHBoxLayout();
        pBatteryrackLayout->setObjectName("pBatteryrackLayout");
        pBatteryRack1 = new QVBoxLayout();
        pBatteryRack1->setObjectName("pBatteryRack1");

        pBatteryrackLayout->addLayout(pBatteryRack1);

        pBatteryRack2 = new QVBoxLayout();
        pBatteryRack2->setObjectName("pBatteryRack2");

        pBatteryrackLayout->addLayout(pBatteryRack2);

        pBatteryRack3 = new QVBoxLayout();
        pBatteryRack3->setObjectName("pBatteryRack3");

        pBatteryrackLayout->addLayout(pBatteryRack3);


        verticalLayout_9->addLayout(pBatteryrackLayout);

        verticalLayout_9->setStretch(0, 3);
        verticalLayout_9->setStretch(1, 2);

        horizontalLayout->addLayout(verticalLayout_9);

        pInfoLayout = new QVBoxLayout();
        pInfoLayout->setObjectName("pInfoLayout");
        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        pInfoLayout->addItem(verticalSpacer_3);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName("verticalLayout_8");
        pLabelTime = new QLabel(Tab1Main);
        pLabelTime->setObjectName("pLabelTime");
        QFont font;
        font.setPointSize(15);
        font.setBold(true);
        pLabelTime->setFont(font);
        pLabelTime->setTextFormat(Qt::TextFormat::AutoText);
        pLabelTime->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_8->addWidget(pLabelTime);


        pInfoLayout->addLayout(verticalLayout_8);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        pInfoLayout->addItem(verticalSpacer);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName("verticalLayout_7");
        label = new QLabel(Tab1Main);
        label->setObjectName("label");
        label->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_7->addWidget(label);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        label_2 = new QLabel(Tab1Main);
        label_2->setObjectName("label_2");
        label_2->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout_4->addWidget(label_2);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        pLabelTemp = new QLabel(Tab1Main);
        pLabelTemp->setObjectName("pLabelTemp");
        pLabelTemp->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout_3->addWidget(pLabelTemp);


        verticalLayout->addLayout(horizontalLayout_3);


        horizontalLayout_7->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        label_12 = new QLabel(Tab1Main);
        label_12->setObjectName("label_12");
        label_12->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout_5->addWidget(label_12);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        pLabelHumi = new QLabel(Tab1Main);
        pLabelHumi->setObjectName("pLabelHumi");
        pLabelHumi->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout_6->addWidget(pLabelHumi);


        verticalLayout_2->addLayout(horizontalLayout_6);


        horizontalLayout_7->addLayout(verticalLayout_2);


        verticalLayout_7->addLayout(horizontalLayout_7);

        verticalLayout_7->setStretch(0, 1);
        verticalLayout_7->setStretch(1, 3);

        pInfoLayout->addLayout(verticalLayout_7);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setObjectName("verticalLayout_10");
        label_3 = new QLabel(Tab1Main);
        label_3->setObjectName("label_3");
        label_3->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_10->addWidget(label_3);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName("horizontalLayout_10");
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName("verticalLayout_5");
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName("verticalLayout_6");
        label_15 = new QLabel(Tab1Main);
        label_15->setObjectName("label_15");
        label_15->setFocusPolicy(Qt::FocusPolicy::NoFocus);
        label_15->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_6->addWidget(label_15);


        verticalLayout_5->addLayout(verticalLayout_6);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName("horizontalLayout_9");
        pLabelGas = new QLabel(Tab1Main);
        pLabelGas->setObjectName("pLabelGas");
        pLabelGas->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout_9->addWidget(pLabelGas);


        verticalLayout_5->addLayout(horizontalLayout_9);


        horizontalLayout_10->addLayout(verticalLayout_5);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName("verticalLayout_4");
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName("verticalLayout_3");
        label_10 = new QLabel(Tab1Main);
        label_10->setObjectName("label_10");
        label_10->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_3->addWidget(label_10);


        verticalLayout_4->addLayout(verticalLayout_3);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName("horizontalLayout_8");
        pLabelThermal = new QLabel(Tab1Main);
        pLabelThermal->setObjectName("pLabelThermal");
        pLabelThermal->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout_8->addWidget(pLabelThermal);


        verticalLayout_4->addLayout(horizontalLayout_8);


        horizontalLayout_10->addLayout(verticalLayout_4);


        verticalLayout_10->addLayout(horizontalLayout_10);

        verticalLayout_10->setStretch(0, 1);
        verticalLayout_10->setStretch(1, 3);

        pInfoLayout->addLayout(verticalLayout_10);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        pInfoLayout->addItem(verticalSpacer_2);

        pInfoLayout->setStretch(1, 1);
        pInfoLayout->setStretch(3, 3);
        pInfoLayout->setStretch(4, 3);

        horizontalLayout->addLayout(pInfoLayout);

        horizontalLayout->setStretch(0, 6);
        horizontalLayout->setStretch(1, 4);

        horizontalLayout_2->addLayout(horizontalLayout);


        retranslateUi(Tab1Main);

        QMetaObject::connectSlotsByName(Tab1Main);
    } // setupUi

    void retranslateUi(QWidget *Tab1Main)
    {
        Tab1Main->setWindowTitle(QCoreApplication::translate("Tab1Main", "Form", nullptr));
        pLabelTime->setText(QCoreApplication::translate("Tab1Main", "YYYY.MM.DD hh:mm:ss", nullptr));
        label->setText(QCoreApplication::translate("Tab1Main", "ESS environment", nullptr));
        label_2->setText(QCoreApplication::translate("Tab1Main", "\354\230\250\353\217\204", nullptr));
        pLabelTemp->setText(QCoreApplication::translate("Tab1Main", "--\342\204\203", nullptr));
        label_12->setText(QCoreApplication::translate("Tab1Main", "\354\212\265\353\217\204", nullptr));
        pLabelHumi->setText(QCoreApplication::translate("Tab1Main", "--%", nullptr));
        label_3->setText(QCoreApplication::translate("Tab1Main", "Emergency Alert", nullptr));
        label_15->setText(QCoreApplication::translate("Tab1Main", "\352\260\200\354\212\244", nullptr));
        pLabelGas->setText(QCoreApplication::translate("Tab1Main", "-", nullptr));
        label_10->setText(QCoreApplication::translate("Tab1Main", "\353\260\234\354\227\264", nullptr));
        pLabelThermal->setText(QCoreApplication::translate("Tab1Main", "-", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Tab1Main: public Ui_Tab1Main {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TAB1MAIN_H
