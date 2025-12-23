#include "batteryrackwidget.h"
#include "ui_batteryrackwidget.h"
#include <QGridLayout>

BatteryRackWidget::BatteryRackWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::BatteryRackWidget)
{
    ui->setupUi(this);

    // QLabel 위에 overlay
    overlay = new QWidget(ui->pLabelRack);
    overlay->setStyleSheet("background: transparent;");

    setupOverlay();
}

BatteryRackWidget::~BatteryRackWidget()
{
    delete ui;
}

void BatteryRackWidget::setupOverlay()
{
    QGridLayout *grid = new QGridLayout(overlay);
    grid->setSpacing(4);
    grid->setContentsMargins(0,0,0,0);

    // 3 랙 × 3 층
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            QFrame *cell = new QFrame;
            cell->setStyleSheet(
                "background-color: rgba(0, 180, 0, 120);"
                "border: 1px solid black;"
                );

            rackCells.append(cell);
            grid->addWidget(cell, row, col);
        }
    }
}

void BatteryRackWidget::resizeEvent(QResizeEvent *)
{
    overlay->setGeometry(ui->pLabelRack->rect());
}

void BatteryRackWidget::setRackState(int rackIndex, int levelIndex, RackState state)
{
    if (rackIndex < 1 || rackIndex > 3 ||
        levelIndex < 1 || levelIndex > 3)
    {
        qDebug() << "[RACK INDEX ERROR]" << rackIndex << levelIndex;
        return;
    }

    int index = (levelIndex - 1) * 3 + (rackIndex - 1);

    switch (state)
    {
    case Normal:
        rackCells[index]->setStyleSheet("background-color: rgba(0,180,0,120);");
        break;
    case Warning:
        rackCells[index]->setStyleSheet("background-color: rgba(255,165,0,160);");
        break;
    case Critical:
        rackCells[index]->setStyleSheet("background-color: rgba(255,0,0,180);");
        break;
    }
}
