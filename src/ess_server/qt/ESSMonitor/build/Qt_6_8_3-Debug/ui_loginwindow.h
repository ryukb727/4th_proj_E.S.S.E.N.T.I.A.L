/********************************************************************************
** Form generated from reading UI file 'loginwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOGINWINDOW_H
#define UI_LOGINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LoginWindow
{
public:
    QVBoxLayout *verticalLayout_8;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *pLoginLayout1;
    QSpacerItem *horizontalSpacer_7;
    QLabel *pLabel_msg1;
    QSpacerItem *horizontalSpacer_6;
    QVBoxLayout *pLoginLayout2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *pLoginLayout2_1;
    QSpacerItem *horizontalSpacer_10;
    QLabel *pLabel_ID;
    QLineEdit *pLineEdit_ID;
    QSpacerItem *horizontalSpacer_8;
    QVBoxLayout *pLoginLayout3;
    QHBoxLayout *pLoginLayout3_1;
    QSpacerItem *horizontalSpacer_11;
    QLabel *pLabel_PW;
    QLineEdit *pLineEdit_PW;
    QSpacerItem *horizontalSpacer_9;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *pLoginLayout4;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *pPBtton_Login;
    QSpacerItem *horizontalSpacer_3;
    QHBoxLayout *pLoginLayout5;
    QSpacerItem *horizontalSpacer_4;
    QLabel *pLabel_status;
    QSpacerItem *horizontalSpacer_5;
    QVBoxLayout *pLoginLayout6;

    void setupUi(QWidget *LoginWindow)
    {
        if (LoginWindow->objectName().isEmpty())
            LoginWindow->setObjectName("LoginWindow");
        LoginWindow->resize(631, 597);
        verticalLayout_8 = new QVBoxLayout(LoginWindow);
        verticalLayout_8->setObjectName("verticalLayout_8");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        pLoginLayout1 = new QHBoxLayout();
        pLoginLayout1->setObjectName("pLoginLayout1");
        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout1->addItem(horizontalSpacer_7);

        pLabel_msg1 = new QLabel(LoginWindow);
        pLabel_msg1->setObjectName("pLabel_msg1");

        pLoginLayout1->addWidget(pLabel_msg1);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout1->addItem(horizontalSpacer_6);


        verticalLayout->addLayout(pLoginLayout1);

        pLoginLayout2 = new QVBoxLayout();
        pLoginLayout2->setObjectName("pLoginLayout2");
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName("verticalLayout_3");

        pLoginLayout2->addLayout(verticalLayout_3);

        pLoginLayout2_1 = new QHBoxLayout();
        pLoginLayout2_1->setObjectName("pLoginLayout2_1");
        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout2_1->addItem(horizontalSpacer_10);

        pLabel_ID = new QLabel(LoginWindow);
        pLabel_ID->setObjectName("pLabel_ID");

        pLoginLayout2_1->addWidget(pLabel_ID);

        pLineEdit_ID = new QLineEdit(LoginWindow);
        pLineEdit_ID->setObjectName("pLineEdit_ID");

        pLoginLayout2_1->addWidget(pLineEdit_ID);

        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout2_1->addItem(horizontalSpacer_8);


        pLoginLayout2->addLayout(pLoginLayout2_1);

        pLoginLayout2->setStretch(0, 2);
        pLoginLayout2->setStretch(1, 1);

        verticalLayout->addLayout(pLoginLayout2);

        pLoginLayout3 = new QVBoxLayout();
        pLoginLayout3->setObjectName("pLoginLayout3");
        pLoginLayout3_1 = new QHBoxLayout();
        pLoginLayout3_1->setObjectName("pLoginLayout3_1");
        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout3_1->addItem(horizontalSpacer_11);

        pLabel_PW = new QLabel(LoginWindow);
        pLabel_PW->setObjectName("pLabel_PW");

        pLoginLayout3_1->addWidget(pLabel_PW);

        pLineEdit_PW = new QLineEdit(LoginWindow);
        pLineEdit_PW->setObjectName("pLineEdit_PW");
        pLineEdit_PW->setEchoMode(QLineEdit::EchoMode::Password);

        pLoginLayout3_1->addWidget(pLineEdit_PW);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout3_1->addItem(horizontalSpacer_9);


        pLoginLayout3->addLayout(pLoginLayout3_1);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName("verticalLayout_6");

        pLoginLayout3->addLayout(verticalLayout_6);

        pLoginLayout3->setStretch(0, 1);
        pLoginLayout3->setStretch(1, 1);

        verticalLayout->addLayout(pLoginLayout3);

        pLoginLayout4 = new QHBoxLayout();
        pLoginLayout4->setObjectName("pLoginLayout4");
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout4->addItem(horizontalSpacer_2);

        pPBtton_Login = new QPushButton(LoginWindow);
        pPBtton_Login->setObjectName("pPBtton_Login");

        pLoginLayout4->addWidget(pPBtton_Login);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout4->addItem(horizontalSpacer_3);


        verticalLayout->addLayout(pLoginLayout4);

        pLoginLayout5 = new QHBoxLayout();
        pLoginLayout5->setObjectName("pLoginLayout5");
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout5->addItem(horizontalSpacer_4);

        pLabel_status = new QLabel(LoginWindow);
        pLabel_status->setObjectName("pLabel_status");

        pLoginLayout5->addWidget(pLabel_status);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        pLoginLayout5->addItem(horizontalSpacer_5);


        verticalLayout->addLayout(pLoginLayout5);

        pLoginLayout6 = new QVBoxLayout();
        pLoginLayout6->setObjectName("pLoginLayout6");

        verticalLayout->addLayout(pLoginLayout6);

        verticalLayout->setStretch(0, 1);
        verticalLayout->setStretch(1, 5);
        verticalLayout->setStretch(2, 5);
        verticalLayout->setStretch(3, 3);
        verticalLayout->setStretch(4, 1);
        verticalLayout->setStretch(5, 2);

        verticalLayout_8->addLayout(verticalLayout);


        retranslateUi(LoginWindow);

        QMetaObject::connectSlotsByName(LoginWindow);
    } // setupUi

    void retranslateUi(QWidget *LoginWindow)
    {
        LoginWindow->setWindowTitle(QCoreApplication::translate("LoginWindow", "Form", nullptr));
        pLabel_msg1->setText(QCoreApplication::translate("LoginWindow", "Please Insert ID and Password", nullptr));
        pLabel_ID->setText(QCoreApplication::translate("LoginWindow", "ID              :", nullptr));
        pLabel_PW->setText(QCoreApplication::translate("LoginWindow", "Password:", nullptr));
        pPBtton_Login->setText(QCoreApplication::translate("LoginWindow", "Login", nullptr));
        pLabel_status->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class LoginWindow: public Ui_LoginWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOGINWINDOW_H
