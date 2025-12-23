#ifndef TAB1MAIN_H
#define TAB1MAIN_H

#include <QWidget>
#include "essmapwidget.h"
#include "batteryrackwidget.h"
#include <QDateTime>

namespace Ui {
class Tab1Main;
}

class Tab1Main : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1Main(QWidget *parent = nullptr);
    ~Tab1Main();

private:
    Ui::Tab1Main *ui;
    ESSMapWidget *pMapWidget;
    BatteryRackWidget *pRackWidget;

    // === ESS Map 상태 기억 ===
    QDateTime lastGasAlertTime;
    int lastGasZone = -1;

    // === Battery Rack 상태 기억 ===
    QDateTime lastThermalAlertTime;
    int lastRackIndex = -1;
    int lastLevelIndex = -1;

private slots:
    void updateTime();
    void updateEnvironment();
    void updateAlert();
    void updateESSMap();
    void updateBatteryRack();
};

#endif // TAB1MAIN_H
