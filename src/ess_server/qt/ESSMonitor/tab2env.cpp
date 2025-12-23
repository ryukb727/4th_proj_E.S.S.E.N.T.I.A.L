#include "tab2env.h"
#include "ui_tab2env.h"

Tab2Env::Tab2Env(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab2Env)
{
    ui->setupUi(this);
}

Tab2Env::~Tab2Env()
{
    delete ui;
}

void Tab2Env::on_pPBSearchDB_clicked()
{

}


void Tab2Env::on_pPBDeleteDB_clicked()
{

}

