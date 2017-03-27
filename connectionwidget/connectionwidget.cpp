#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QComboBox>
#include <QSerialPortInfo>
#include "connectionwidget.h"
#include "../indicator/indicator.h"
#include <QDebug>
#include <QTimer>

const QStringList baudList = {"57600","115200"};
QStringList availablePorts()
{
    QStringList ports;

    for (QSerialPortInfo port : QSerialPortInfo::availablePorts())
    {
        ports += port.portName() + " (" + port.description() + ")";
    }
    return ports;
}

ConnectionWidget::ConnectionWidget(int type, QWidget *parent)
    : QGroupBox(QObject::tr((type)?"PX4":"Xplane"),parent),mType(type)
{
    QFormLayout *formLayout = new QFormLayout;

    if(!type) {
        ipEdit = new QLineEdit;
        ipEdit->setInputMask("000.000.000.000");

        ipEdit->setText("192.168.1.3");
        formLayout->addRow(QObject::tr("IP:"), ipEdit);

        port = new QLineEdit("49005");
        port->setInputMask("00000");

        formLayout->addRow(QObject::tr("port:"), port);
    } else {
        comBox = new QComboBox;
        comBox->addItems(availablePorts());
        formLayout->addRow(QObject::tr("COM:"),comBox);

        bauds = new QComboBox;
        bauds->addItems(baudList);
        bauds->setCurrentIndex(1);
        formLayout->addRow(QObject::tr("baud:"),bauds);

        QTimer::singleShot(10,this,SLOT(updatePortList()));
    }


    indr = new Indicator(QObject::tr("Link"),this);

    QVBoxLayout *radioLayout = new QVBoxLayout;
    radioLayout->addWidget(indr);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(formLayout);
    layout->addLayout(radioLayout);
    layout->addStretch();
    setLayout(layout);
}

QString ConnectionWidget::getName() const
{
    if(!mType)
        return ipEdit->text();

    else return comBox->currentText();
}

void ConnectionWidget::setColor(const QColor &c)
{
    indr->setColor(c);
}

void ConnectionWidget::setIP(const QString &ip)
{
    ipEdit->setText(ip);
}

void ConnectionWidget::updatePortList()
{
    comBox->clear();
    comBox->addItems(availablePorts());

    QTimer::singleShot(500,this,SLOT(updatePortList()));
}


int ConnectionWidget::getBaud() const
{
    return baudList.at(bauds->currentIndex()).toInt();
}

quint16 ConnectionWidget::getPort() const
{
    return port->text().toInt();
}


