#include "tab3alert.h"
#include "ui_tab3alert.h"

Tab3Alert::Tab3Alert(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab3Alert)
{
    ui->setupUi(this);
}

Tab3Alert::~Tab3Alert()
{
    delete ui;
}
