#ifndef TAB4ACCESS_H
#define TAB4ACCESS_H

#include <QWidget>

namespace Ui {
class Tab4Access;
}

class Tab4Access : public QWidget
{
    Q_OBJECT

public:
    explicit Tab4Access(QWidget *parent = nullptr);
    ~Tab4Access();

private:
    Ui::Tab4Access *ui;
};

#endif // TAB4ACCESS_H
