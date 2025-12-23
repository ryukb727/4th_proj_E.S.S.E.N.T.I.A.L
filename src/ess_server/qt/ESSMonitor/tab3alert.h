#ifndef TAB3ALERT_H
#define TAB3ALERT_H

#include <QWidget>

namespace Ui {
class Tab3Alert;
}

class Tab3Alert : public QWidget
{
    Q_OBJECT

public:
    explicit Tab3Alert(QWidget *parent = nullptr);
    ~Tab3Alert();

private:
    Ui::Tab3Alert *ui;
};

#endif // TAB3ALERT_H
