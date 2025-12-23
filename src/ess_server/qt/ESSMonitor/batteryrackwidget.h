#ifndef BATTERYRACKWIDGET_H
#define BATTERYRACKWIDGET_H

#include <QWidget>
#include <QVector>
#include <QFrame>

namespace Ui {
class BatteryRackWidget;
}

class BatteryRackWidget : public QWidget
{
    Q_OBJECT

public:
    explicit BatteryRackWidget(QWidget *parent = nullptr);
    ~BatteryRackWidget();

    enum RackState {
        Normal,
        Warning,
        Critical
    };

    // rackIndex: 0~2, levelIndex: 0~2
    void setRackState(int rackIndex, int levelIndex, RackState state);

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    Ui::BatteryRackWidget *ui;

    QWidget *overlay;
    QVector<QFrame*> rackCells;   // 총 9칸 (3×3)

    void setupOverlay();
};

#endif // BATTERYRACKWIDGET_H
