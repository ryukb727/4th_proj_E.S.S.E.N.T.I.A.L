#include "tab4access.h"
#include "ui_tab4access.h"

Tab4Access::Tab4Access(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab4Access)
{
    ui->setupUi(this);
}

Tab4Access::~Tab4Access()
{
    delete ui;
}
