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
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ESSMapWidget
{
public:
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *pLabelMap;

    void setupUi(QWidget *ESSMapWidget)
    {
        if (ESSMapWidget->objectName().isEmpty())
            ESSMapWidget->setObjectName("ESSMapWidget");
        ESSMapWidget->resize(400, 300);
        gridLayoutWidget = new QWidget(ESSMapWidget);
        gridLayoutWidget->setObjectName("gridLayoutWidget");
        gridLayoutWidget->setGeometry(QRect(130, 100, 160, 80));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName("gridLayout");
        gridLayout->setContentsMargins(0, 0, 0, 0);
        pLabelMap = new QLabel(gridLayoutWidget);
        pLabelMap->setObjectName("pLabelMap");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pLabelMap->sizePolicy().hasHeightForWidth());
        pLabelMap->setSizePolicy(sizePolicy);
        pLabelMap->setPixmap(QPixmap(QString::fromUtf8(":/images/img/ess_map.jpg")));
        pLabelMap->setAlignment(Qt::AlignmentFlag::AlignCenter);

        gridLayout->addWidget(pLabelMap, 0, 0, 1, 1);


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
