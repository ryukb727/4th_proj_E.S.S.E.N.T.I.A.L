#include "essmapwidget.h"
#include "ui_essmapwidget.h"
#include <QLabel>
#include <QGridLayout>

ESSMapWidget::ESSMapWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ESSMapWidget)
    , blinkOn(true)
{
    ui->setupUi(this);

    ui->pLabelMap = new QLabel(this);
    ui->pLabelMap->setAlignment(Qt::AlignCenter);

    mapPixmap.load(":/images/img/ess_map.jpg");
    ui->pLabelMap->setPixmap(mapPixmap);

    overlay = new QWidget(ui->pLabelMap);
    overlay->setStyleSheet("background: transparent;");

    setupOverlay();

    pblinkTimer = new QTimer(this);
    connect(pblinkTimer, &QTimer::timeout, this, [this]() {
        blinkOn = !blinkOn;
        for(int zone : criticalZones)
        {
            zones[zone]->setVisible(blinkOn);
        }
    });
    pblinkTimer->start(500);

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    layout->addWidget(ui->pLabelMap);
}

ESSMapWidget::~ESSMapWidget()
{
    delete ui;
}

void ESSMapWidget::setupOverlay()
{
    QGridLayout *grid = new QGridLayout(overlay);
    grid->setSpacing(2);
    grid->setContentsMargins(0,0,0,0);

    for (int i = 0; i < 6; ++i)
    {
        QFrame *zone = new QFrame;
        zone->setStyleSheet("background-color: rgba(0,255,0,50);");
        zones.append(zone);
        grid->addWidget(zone, i / 3, i % 3);
    }
}

void ESSMapWidget::resizeEvent(QResizeEvent *)
{
    updateMapSize();
}

void ESSMapWidget::updateMapSize()
{
    ui->pLabelMap->setPixmap(
        mapPixmap.scaled(
            ui->pLabelMap->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
            )
        );
    overlay->setGeometry(ui->pLabelMap->rect());
}

void ESSMapWidget::setZoneState(int zoneIndex, ZoneState state)
{
    if(zoneIndex < 0 || zoneIndex >= zones.size())
        return;

    QFrame *zone = zones[zoneIndex];

    switch(state)
    {
    case Normal:
        zone->setStyleSheet(
            "background-color: rgba(0, 180, 0, 120);"
            );
        break;

    case Warning:
        zone->setStyleSheet(
            "background-color: rgba(255, 215, 0, 170);"
            );
        break;

    case Critical:
        zone->setStyleSheet(
            "background-color: rgba(255, 0, 0, 190);"
            );
        break;
    }
}
