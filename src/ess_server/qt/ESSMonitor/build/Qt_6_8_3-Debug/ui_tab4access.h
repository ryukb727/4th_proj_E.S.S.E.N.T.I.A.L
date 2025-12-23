/********************************************************************************
** Form generated from reading UI file 'tab4access.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TAB4ACCESS_H
#define UI_TAB4ACCESS_H

#include <QtCore/QDate>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDateTimeEdit>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Tab4Access
{
public:
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QDateTimeEdit *pDateTimeFrom;
    QDateTimeEdit *pDateTimeTo;
    QComboBox *pCBAccessPoints;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton;
    QVBoxLayout *verticalLayout;
    QTableWidget *tableWidget;

    void setupUi(QWidget *Tab4Access)
    {
        if (Tab4Access->objectName().isEmpty())
            Tab4Access->setObjectName("Tab4Access");
        Tab4Access->resize(615, 439);
        verticalLayout_3 = new QVBoxLayout(Tab4Access);
        verticalLayout_3->setObjectName("verticalLayout_3");
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        pDateTimeFrom = new QDateTimeEdit(Tab4Access);
        pDateTimeFrom->setObjectName("pDateTimeFrom");
        pDateTimeFrom->setMinimumDate(QDate(2025, 12, 1));

        horizontalLayout->addWidget(pDateTimeFrom);

        pDateTimeTo = new QDateTimeEdit(Tab4Access);
        pDateTimeTo->setObjectName("pDateTimeTo");
        pDateTimeTo->setMinimumDate(QDate(2025, 12, 1));

        horizontalLayout->addWidget(pDateTimeTo);

        pCBAccessPoints = new QComboBox(Tab4Access);
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->addItem(QString());
        pCBAccessPoints->setObjectName("pCBAccessPoints");

        horizontalLayout->addWidget(pCBAccessPoints);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pushButton = new QPushButton(Tab4Access);
        pushButton->setObjectName("pushButton");

        horizontalLayout->addWidget(pushButton);


        verticalLayout_2->addLayout(horizontalLayout);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        tableWidget = new QTableWidget(Tab4Access);
        tableWidget->setObjectName("tableWidget");

        verticalLayout->addWidget(tableWidget);


        verticalLayout_2->addLayout(verticalLayout);

        verticalLayout_2->setStretch(0, 1);
        verticalLayout_2->setStretch(1, 7);

        verticalLayout_3->addLayout(verticalLayout_2);


        retranslateUi(Tab4Access);

        QMetaObject::connectSlotsByName(Tab4Access);
    } // setupUi

    void retranslateUi(QWidget *Tab4Access)
    {
        Tab4Access->setWindowTitle(QCoreApplication::translate("Tab4Access", "Form", nullptr));
        pCBAccessPoints->setItemText(0, QCoreApplication::translate("Tab4Access", "main", nullptr));
        pCBAccessPoints->setItemText(1, QCoreApplication::translate("Tab4Access", "ew1", nullptr));
        pCBAccessPoints->setItemText(2, QCoreApplication::translate("Tab4Access", "ew2", nullptr));
        pCBAccessPoints->setItemText(3, QCoreApplication::translate("Tab4Access", "ew3", nullptr));
        pCBAccessPoints->setItemText(4, QCoreApplication::translate("Tab4Access", "ww1", nullptr));
        pCBAccessPoints->setItemText(5, QCoreApplication::translate("Tab4Access", "ww2", nullptr));
        pCBAccessPoints->setItemText(6, QCoreApplication::translate("Tab4Access", "sw1", nullptr));
        pCBAccessPoints->setItemText(7, QCoreApplication::translate("Tab4Access", "sw2", nullptr));
        pCBAccessPoints->setItemText(8, QCoreApplication::translate("Tab4Access", "nw1", nullptr));
        pCBAccessPoints->setItemText(9, QCoreApplication::translate("Tab4Access", "nw2", nullptr));

        pushButton->setText(QCoreApplication::translate("Tab4Access", "\354\241\260\355\232\214", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Tab4Access: public Ui_Tab4Access {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TAB4ACCESS_H
