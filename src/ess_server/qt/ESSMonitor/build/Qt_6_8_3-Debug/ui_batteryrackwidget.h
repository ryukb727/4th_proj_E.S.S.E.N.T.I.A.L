/********************************************************************************
** Form generated from reading UI file 'batteryrackwidget.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BATTERYRACKWIDGET_H
#define UI_BATTERYRACKWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_BatteryRackWidget
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QLabel *pLabelRack;

    void setupUi(QWidget *BatteryRackWidget)
    {
        if (BatteryRackWidget->objectName().isEmpty())
            BatteryRackWidget->setObjectName("BatteryRackWidget");
        BatteryRackWidget->resize(400, 300);
        verticalLayout_2 = new QVBoxLayout(BatteryRackWidget);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        pLabelRack = new QLabel(BatteryRackWidget);
        pLabelRack->setObjectName("pLabelRack");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pLabelRack->sizePolicy().hasHeightForWidth());
        pLabelRack->setSizePolicy(sizePolicy);
        pLabelRack->setStyleSheet(QString::fromUtf8("background-color:  #dcdcdc;\n"
"border: 1px solid #b0b0b0;"));
        pLabelRack->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout->addWidget(pLabelRack);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(BatteryRackWidget);

        QMetaObject::connectSlotsByName(BatteryRackWidget);
    } // setupUi

    void retranslateUi(QWidget *BatteryRackWidget)
    {
        BatteryRackWidget->setWindowTitle(QCoreApplication::translate("BatteryRackWidget", "Form", nullptr));
        pLabelRack->setText(QCoreApplication::translate("BatteryRackWidget", "Ess Battery Rack View", nullptr));
    } // retranslateUi

};

namespace Ui {
    class BatteryRackWidget: public Ui_BatteryRackWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BATTERYRACKWIDGET_H
