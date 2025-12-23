#include "tab1main.h"
#include "ui_tab1main.h"
#include "essmapwidget.h"
#include "batteryrackwidget.h"
#include <QTimer>
#include <QDateTime>
#include <QSqlQuery>
#include <QDebug>
#include <QSqlError>

Tab1Main::Tab1Main(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab1Main)
{
    ui->setupUi(this);

    pMapWidget = new ESSMapWidget(this);
    pRackWidget = new BatteryRackWidget(this);

    ui->pMapLayout->layout()->addWidget(pMapWidget);
    ui->pBatteryrackLayout->layout()->addWidget(pRackWidget);

    QTimer *ptimeTimer = new QTimer(this);
    connect(ptimeTimer, SIGNAL(timeout()), this, SLOT(updateTime()));
    ptimeTimer->start(1000);

    QTimer *envTimer = new QTimer(this);
    connect(envTimer, SIGNAL(timeout()), this, SLOT(updateEnvironment()));
    envTimer->start(5000);

    QTimer *alertTimer = new QTimer(this);
    connect(alertTimer, SIGNAL(timeout()), this, SLOT(updateAlert()));
    alertTimer->start(2000);

    QTimer *mapTimer = new QTimer(this);
    connect(mapTimer, SIGNAL(timeout()), this, SLOT(updateESSMap()));
    mapTimer->start(2000);

    QTimer *rackTimer = new QTimer(this);
    connect(rackTimer, SIGNAL(timeout()), this, SLOT(updateBatteryRack()));
    rackTimer->start(2000);
}

Tab1Main::~Tab1Main()
{
    delete ui;
}

void Tab1Main::updateTime()
{
    QString now = QDateTime::currentDateTime()
    .toString("yyyy.MM.dd hh:mm:ss");
    ui->pLabelTime->setText(now);
}

void Tab1Main::updateEnvironment()
{
    QSqlQuery query;

    if(!query.exec(
            "SELECT temperature, humidity "
            "FROM environment_data "
            "ORDER BY measure_time DESC LIMIT 1"
            ))
    {
        qDebug() << "[ENV QUERY ERROR]" << query.lastError().text();
        return;
    }

    if(query.next())
    {
        qDebug() << "[ENV DATA]"
                 << query.value(0).toDouble()
                 << query.value(1).toDouble();

        ui->pLabelTemp->setText(
            QString::number(query.value(0).toDouble(), 'f', 1) + " °C"
            );
        ui->pLabelHumi->setText(
            QString::number(query.value(1).toDouble(), 'f', 1) + " %"
            );
    }
    else
    {
        qDebug() << "[ENV] no row";
    }
}

void Tab1Main::updateAlert()
{
    QSqlQuery query;
    query.exec(
        "SELECT event_type, level "
        "FROM alert_events "
        "ORDER BY alert_time DESC LIMIT 1"
        );

    ui->pLabelGas->setText("-");
    ui->pLabelThermal->setText("-");

    if(query.next())
    {
        QString type = query.value(0).toString();
        QString level = query.value(1).toString();

        QLabel *target =
            (type == "gas") ? ui->pLabelGas : ui->pLabelThermal;

        target->setText(level.toUpper());

        if(level == "warning")
            target->setStyleSheet("color: orange;");
        else
            target->setStyleSheet("color: red; font-weight: bold;");
    }
}

void Tab1Main::updateESSMap()
{
    QSqlQuery query;
    query.exec(
        "SELECT location, level, alert_time "
        "FROM alert_events "
        "WHERE event_type='gas' "
        "ORDER BY alert_time DESC LIMIT 1"
        );

    // 새 알림이 있는 경우
    if (query.next())
    {
        QDateTime alertTime = query.value(2).toDateTime();

        // DB에 새 알림이 들어온 경우만 처리
        if (!lastGasAlertTime.isValid() || alertTime > lastGasAlertTime)
        {
            QString location = query.value(0).toString(); // zone_2
            QString level = query.value(1).toString();

            lastGasZone = location.split("_").last().toInt();
            lastGasAlertTime = alertTime;

            if (level == "warning")
                pMapWidget->setZoneState(lastGasZone, ESSMapWidget::Warning);
            else
                pMapWidget->setZoneState(lastGasZone, ESSMapWidget::Critical);
        }
    }

    // 마지막 알림 기준 10초 초과 시 자동 복귀
    if (lastGasAlertTime.isValid() &&
        lastGasAlertTime.secsTo(QDateTime::currentDateTime()) > 10)
    {
        for (int i = 0; i < 6; ++i)
            pMapWidget->setZoneState(i, ESSMapWidget::Normal);

        lastGasAlertTime = QDateTime(); // 리셋
        lastGasZone = -1;
    }
}

void Tab1Main::updateBatteryRack()
{
    QSqlQuery query;
    query.exec(
        "SELECT location, level, alert_time "
        "FROM alert_events "
        "WHERE event_type='thermal' "
        "ORDER BY alert_time DESC LIMIT 1"
        );

    if (query.next())
    {
        QDateTime alertTime = query.value(2).toDateTime();

        if (!lastThermalAlertTime.isValid() || alertTime > lastThermalAlertTime)
        {
            QString location = query.value(0).toString(); // rack_2_3
            QString levelStr = query.value(1).toString();

            QStringList parts = location.split("_");
            if (parts.size() == 3)
            {
                lastRackIndex = parts[1].toInt();
                lastLevelIndex = parts[2].toInt();
                lastThermalAlertTime = alertTime;

                if (levelStr == "warning")
                    pRackWidget->setRackState(
                        lastRackIndex, lastLevelIndex,
                        BatteryRackWidget::Warning
                        );
                else
                    pRackWidget->setRackState(
                        lastRackIndex, lastLevelIndex,
                        BatteryRackWidget::Critical
                        );
            }
        }
    }

    // 10초 지나면 자동 Normal
    if (lastThermalAlertTime.isValid() &&
        lastThermalAlertTime.secsTo(QDateTime::currentDateTime()) > 10)
    {
        for (int r = 1; r <= 3; ++r)
            for (int l = 1; l <= 3; ++l)
                pRackWidget->setRackState(
                    r, l, BatteryRackWidget::Normal
                    );

        lastThermalAlertTime = QDateTime();
        lastRackIndex = -1;
        lastLevelIndex = -1;
    }
}
