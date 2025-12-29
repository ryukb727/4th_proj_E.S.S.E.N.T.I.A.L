#include "tab2env.h"
#include "ui_tab2env.h"

Tab2Env::Tab2Env(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab2Env)
{
    ui->setupUi(this);

    if (!QSqlDatabase::database().isOpen()) {
        qDebug() << "[Tab2Env] DB not open!";
        ui->pPBSearchDB->setEnabled(false);
        ui->pPBDeleteDB->setEnabled(false);
        return;
    }

    // Chart 기본 구성
    tempLine = new QLineSeries(this);
    tempLine->setName("Temperature");

    humiLine = new QLineSeries(this);
    humiLine->setName("Humidity");

    QPen pen;
    pen.setWidth(2);
    pen.setBrush(Qt::red);
    pen.setCapStyle(Qt::FlatCap);
    pen.setJoinStyle(Qt::MiterJoin);
    tempLine->setPen(pen);
    pen.setBrush(Qt::blue);
    humiLine->setPen(pen);

    pQChart = new QChart();
    pQChart->addSeries(tempLine);
    pQChart->addSeries(humiLine);
    pQChart->legend()->setVisible(true);

    pQDateTimeAxisX = new QDateTimeAxis;
    pQDateTimeAxisX->setFormat("MM-dd HH:mm");

    QValueAxis *axisYTemp = new QValueAxis;
    axisYTemp->setRange(0, 50);
    axisYTemp->setTitleText("Temperature (°C)");

    QValueAxis *axisYHumi = new QValueAxis;
    axisYHumi->setRange(0, 100);
    axisYHumi->setTitleText("Humidity (%)");

    pQChart->addAxis(pQDateTimeAxisX, Qt::AlignBottom);
    pQChart->addAxis(axisYTemp, Qt::AlignLeft);
    pQChart->addAxis(axisYHumi, Qt::AlignRight);

    tempLine->attachAxis(pQDateTimeAxisX);
    tempLine->attachAxis(axisYTemp);
    humiLine->attachAxis(pQDateTimeAxisX);
    humiLine->attachAxis(axisYHumi);

    pQChartView = new QChartView(pQChart);
    pQChartView->setRenderHint(QPainter::Antialiasing);

    ui->pChartViewLayout->layout()->addWidget(pQChartView);
}

Tab2Env::~Tab2Env()
{
    delete ui;
}

void Tab2Env::on_pPBSearchDB_clicked()
{
    // 테이블 초기화
    ui->pEnvTable->clearContents();
    ui->pEnvTable->setRowCount(0);

    // 그래프 초기화
    tempLine->clear();
    humiLine->clear();

    QDateTime fromDateTime = ui->pDateTimeFrom->dateTime();
    QDateTime toDateTime = ui->pDateTimeTo->dateTime();
    pQDateTimeAxisX->setRange(fromDateTime, toDateTime);

    QString strFromDateTime = fromDateTime.toString("yyyy-MM-dd HH:mm:ss");
    QString strToDateTime = toDateTime.toString("yyyy-MM-dd HH:mm:ss");

    QString strQuery =
        "SELECT id, measure_time, temperature, humidity "
        "FROM environment_data "
        "WHERE measure_time BETWEEN '" + strFromDateTime +
        "' AND '" + strToDateTime + "' "
        "ORDER BY measure_time ASC";

    QSqlQuery sqlQuery;
    if (!sqlQuery.exec(strQuery)) {
        qDebug() << "[ERROR] Query failed:" << sqlQuery.lastError();
        return;
    }

    // QSqlQuery countSqlQuery(countStrQuery);
    // int rowCount = 0;
    // if(countSqlQuery.next()) {
    //     rowCount = countSqlQuery.value(0).toInt();
    // }

    int rowCount = 0;
    while (sqlQuery.next())
    {
        ui->pEnvTable->insertRow(rowCount);

        QString idStr = sqlQuery.value("id").toString();
        QString tempStr = sqlQuery.value("temperature").toString();
        QString humiStr = sqlQuery.value("humidity").toString();

        QString timeStr = sqlQuery.value("measure_time").toString();
        QDateTime xValue = QDateTime::fromString(timeStr, "yyyy-MM-dd HH:mm:ss");
        xValue.setTimeSpec(Qt::LocalTime);
        if (!xValue.isValid()) {
            xValue = QDateTime::fromString(timeStr, Qt::ISODate);
            xValue.setTimeSpec(Qt::LocalTime);
        }

        // 그래프에 값 추가
        if (xValue.isValid()) {
            tempLine->append(xValue.toMSecsSinceEpoch(), tempStr.toDouble());
            humiLine->append(xValue.toMSecsSinceEpoch(), humiStr.toDouble());
        }

        // 테이블에 값 추가
        ui->pEnvTable->setItem(rowCount, 0, new QTableWidgetItem(idStr));
        ui->pEnvTable->setItem(rowCount, 1, new QTableWidgetItem(xValue.toString("yyyy-MM-dd HH:mm:ss")));
        ui->pEnvTable->setItem(rowCount, 2, new QTableWidgetItem(tempStr));
        ui->pEnvTable->setItem(rowCount, 3, new QTableWidgetItem(humiStr));

        rowCount++;
    }

    // 컬럼 너비 자동 조정
    for (int col = 0; col < 4; ++col) {
        ui->pEnvTable->resizeColumnToContents(col);
    }
}


void Tab2Env::on_pPBDeleteDB_clicked()
{
    // 선택된 날짜 범위 가져오기
    QDateTime fromDateTime = ui->pDateTimeFrom->dateTime();
    QString strFromDateTime = fromDateTime.toString("yyyy-MM-dd HH:mm:ss");
    QDateTime toDateTime = ui->pDateTimeTo->dateTime();
    QString strToDateTime = toDateTime.toString("yyyy-MM-dd HH:mm:ss");

    // DB 삭제 쿼리
    QString strQuery =  "DELETE FROM environment_data "
                       "WHERE measure_time BETWEEN '" + strFromDateTime +
                       "' AND '" + strToDateTime + "'";
    QSqlQuery sqlQuery;
    if(!sqlQuery.exec(strQuery)) {
        qDebug() << "[ERROR] Delete Query failed:" << sqlQuery.lastError();
        return;
    } else {
        qDebug() << "Delete Query OK";
    }

    // 차트 초기화
    tempLine->clear();
    humiLine->clear();

    // 테이블 초기화 및 메모리 해제
    auto safeDeleteArray = [](QTableWidgetItem*& ptr) {
        if(ptr) { delete[] ptr; ptr = nullptr; }
    };

    safeDeleteArray(pQTableWidgetItemId);
    safeDeleteArray(pQTableWidgetItemDate);
    safeDeleteArray(pQTableWidgetItemTemp);
    safeDeleteArray(pQTableWidgetItemHumi);

    ui->pEnvTable->clearContents();
    ui->pEnvTable->setRowCount(0);

    QSqlDatabase db = QSqlDatabase::database();
    if(db.isOpen()) {
        if(!db.commit()) {
            qDebug() << "[ERROR] Commit failed:" << db.lastError();
        } else {
            qDebug() << "DB commit OK";
        }
    }
}
