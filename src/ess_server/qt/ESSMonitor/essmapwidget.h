#ifndef ESSMAPWIDGET_H
#define ESSMAPWIDGET_H

#pragma once
#include <QWidget>
#include <QPixmap>
#include <QFrame>
#include <QVector>
#include <QTimer>
#include <QSet>

namespace Ui {
class ESSMapWidget;
}

class ESSMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ESSMapWidget(QWidget *parent = nullptr);

    enum ZoneState {
        Normal,
        Warning,
        Critical
    };

    void setZoneState(int zoneIndex, ZoneState state);
    ~ESSMapWidget();

private:
    Ui::ESSMapWidget *ui;
    QWidget *overlay;
    QVector<QFrame*> zones;
    QPixmap mapPixmap;
    QTimer *pblinkTimer;
    bool blinkOn;
    QSet<int> criticalZones;

    void setupOverlay();
    void updateMapSize();

protected:
    void resizeEvent(QResizeEvent *event) override;
};

#endif // ESSMAPWIDGET_H
