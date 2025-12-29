/********************************************************************************
** Form generated from reading UI file 'essmapwidget.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ESSMAPWIDGET_H
#define UI_ESSMAPWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ESSMapWidget
{
public:
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QLabel *pLabelMap;

    void setupUi(QWidget *ESSMapWidget)
    {
        if (ESSMapWidget->objectName().isEmpty())
            ESSMapWidget->setObjectName("ESSMapWidget");
        ESSMapWidget->resize(400, 300);
        verticalLayout = new QVBoxLayout(ESSMapWidget);
        verticalLayout->setObjectName("verticalLayout");
        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        pLabelMap = new QLabel(ESSMapWidget);
        pLabelMap->setObjectName("pLabelMap");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pLabelMap->sizePolicy().hasHeightForWidth());
        pLabelMap->setSizePolicy(sizePolicy);
        pLabelMap->setPixmap(QPixmap(QString::fromUtf8(":/images/img/ess_map.jpg")));
        pLabelMap->setAlignment(Qt::AlignmentFlag::AlignCenter);

        gridLayout->addWidget(pLabelMap, 0, 0, 1, 1);


        verticalLayout->addLayout(gridLayout);


        retranslateUi(ESSMapWidget);

        QMetaObject::connectSlotsByName(ESSMapWidget);
    } // setupUi

    void retranslateUi(QWidget *ESSMapWidget)
    {
        ESSMapWidget->setWindowTitle(QCoreApplication::translate("ESSMapWidget", "Form", nullptr));
        pLabelMap->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class ESSMapWidget: public Ui_ESSMapWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ESSMAPWIDGET_H
