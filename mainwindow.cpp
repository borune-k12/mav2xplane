#include "mainwindow.h"
#include "connectionwidget/connectionwidget.h"
#include <QGridLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QMessageBox>
#include <QComboBox>

const QStringList px4modes = {"Manual",
                              "Altitude",
                              "Position",
                              "Mission",
                              "Hold",
                              "Takeoff",
                              "Land",
                              "Return",
                              "Acro",
                              "Offboard",
                              "Stabilized",
                              "Rattitude",
                              "Follow Me",
                              "Return to Groundstation",
                              "Ready",
                              "Simple"};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),px4_connected(true),xplane_connected(true),link(nullptr),hilEnabled(false)
{  
    centralWidget = new QWidget();
    grid = new QGridLayout;
    xplane_connection = new ConnectionWidget(0);
    xplane_connection->setColor(Qt::red);
    remoteHost = xplane_connection->getName();


    px4_connection = new ConnectionWidget(1);
    px4_connection->setColor(Qt::red);

    grid->addWidget(px4_connection,0,0,1,1);
    grid->addWidget(xplane_connection,0,1,1,1);

    startBtn = new QPushButton(tr("Start"));
    startBtn->setEnabled(true);
    connect(startBtn,&QPushButton::clicked,this,&MainWindow::onStartClicked);

    QGroupBox *vbox = new QGroupBox;
    QVBoxLayout *l = new QVBoxLayout;
    armBtn = new QPushButton(tr("ARM"));
    connect(armBtn,&QPushButton::clicked, this, &MainWindow::onArmClicked);


    armIndicator = new Indicator("ARM STATUS");
    l->addWidget(armIndicator);
    l->addWidget(armBtn);
    vbox->setLayout(l);

    grid->addWidget(vbox,1,0,2,1);

    QGroupBox *box = new QGroupBox;

    QGridLayout *grid2 = new QGridLayout;
    mode = new QLabel("FLIGHT MODE");
    mode->setAlignment(Qt::AlignCenter);

    grid2->addWidget(mode,0,0,1,2);

    takeoffBtn = new QPushButton(tr("Takeoff"));
    connect(takeoffBtn,&QPushButton::clicked, this, &MainWindow::onTakeoffClicked);

    grid2->addWidget(takeoffBtn,2,0,1,2);

    modeBox = new QComboBox;
    modeBox->addItems(px4modes);
    grid->addWidget(startBtn,5,0,1,3);

    enableHilBtn = new QPushButton(tr("Enable HIL"));

    connect(enableHilBtn,SIGNAL(released()),this,SLOT(enableHil()));
    grid->addWidget(enableHilBtn,4,0,1,3);

    setModeBtn = new QPushButton(tr("Set mode"));
    connect(setModeBtn,SIGNAL(released()),this,SLOT(setMode()));
    grid2->addWidget(setModeBtn,1,0,1,1);

    grid2->addWidget(modeBox,1,1,1,1);

    box->setLayout(grid2);

    grid->addWidget(box,2,1,1,1);

    centralWidget->setLayout(grid);
    setCentralWidget(centralWidget);

    armed = false;
    connected = false;

    uas = nullptr;

    enableButtons(false);
}

MainWindow::~MainWindow()
{
    delete grid;
    if(link)
    {
        link->deleteLater();
    }
}

void MainWindow::enableHil()
{
    if(uas)
        uas->setHilMode(hilEnabled = !hilEnabled);
    enableHilBtn->setText((hilEnabled)?("Disable HIL"):("Enable HIL"));
}

void MainWindow::setMode()
{
    if(uas)
        uas->setFlightMode(modeBox->currentText());
}

void MainWindow::armStayChanged(int armed)
{
    (armed)?armIndicator->setColor(Qt::green):armIndicator->setColor(Qt::red);
    (armed)?armIndicator->setText("ARMED"):armIndicator->setText("DISARMED");
    (armed)?armBtn->setText("DISARM"):armBtn->setText("ARM");
}

void MainWindow::flightModeChanged(QString mode)
{
    this->mode->setText(mode);
}

void MainWindow::onPx4Connected()
{
    px4_connection->setColor(Qt::green);
}

void MainWindow::onXplaneConnected(QString host)
{
    if(host.contains(remoteHost)){
        xplane_connection->setColor(Qt::green);
        link->setRemoteHost(host);
    }
    else xplane_connection->setColor(Qt::red);
}

void MainWindow::onStartClicked()
{
    if(connected)
        stopSimulation();

    else {
        if(link != nullptr)
        {
            link->terminate();
            link->wait();
            delete link;
        }

        QString port = px4_connection->getName();
        port = port.mid(0,port.indexOf("(")-1);

        uas = new UAS(port,px4_connection->getBaud());
        connect(uas,&UAS::portOpened,this,&MainWindow::onPx4Connected,Qt::QueuedConnection);
        connect(uas,&UAS::armStayChanged,this,&MainWindow::armStayChanged);
        connect(uas,&UAS::flightModeChanged,this,&MainWindow::flightModeChanged);
        if(!uas->openPort())
            return;

        link = new QGCXPlaneLink(xplane_connection->getName(),xplane_connection->getPort());

        connect(link,&QGCXPlaneLink::hostConnected,this,&MainWindow::onXplaneConnected);
        connect(uas, &UAS::hilControlsChanged,link,&QGCXPlaneLink::updateControls,Qt::QueuedConnection);

        connect(link, &QGCXPlaneLink::hilStateChanged, uas, &UAS::sendHilState,Qt::QueuedConnection);
        connect(link, &QGCXPlaneLink::sensorHilGpsChanged, uas, &UAS::sendHilGps,Qt::QueuedConnection);
        connect(link, &QGCXPlaneLink::sensorHilRawImuChanged,uas, &UAS::sendHilSensors,Qt::QueuedConnection);
        connect(uas, &UAS::hilActuatorControlsChanged,link,&QGCXPlaneLink::updateActuatorControls);

        link->start();

        enableButtons(true);
    }

    connected = !connected;
    connected?startBtn->setText("Stop"):startBtn->setText("Start");
}

void MainWindow::onPortError(QString error)
{
    QMessageBox msg(QMessageBox::Warning,tr("Error"),error);
    msg.exec();

    px4_connection->setColor(Qt::red);
}

void MainWindow::onArmClicked()
{
    armed = !armed;
    if(uas)
        uas->setArmed(armed);
}

void MainWindow::onTakeoffClicked()
{
    if(uas)
        uas->takeoff();
}

void MainWindow::stopSimulation()
{
    link->stop();
    uas->closePort();

    link->deleteLater();
    uas->deleteLater();

    disconnect(link,&QGCXPlaneLink::hostConnected,this,&MainWindow::onXplaneConnected);
    disconnect(uas, &UAS::hilControlsChanged,link,&QGCXPlaneLink::updateControls);

    disconnect(link, &QGCXPlaneLink::hilStateChanged, uas, &UAS::sendHilState);
    disconnect(link, &QGCXPlaneLink::sensorHilGpsChanged, uas, &UAS::sendHilGps);
    disconnect(link, &QGCXPlaneLink::sensorHilRawImuChanged,uas, &UAS::sendHilSensors);
    disconnect(uas, &UAS::hilActuatorControlsChanged,link,&QGCXPlaneLink::updateActuatorControls);


    px4_connection->setColor(Qt::red);
    xplane_connection->setColor(Qt::red);
    armIndicator->setColor(Qt::transparent);
    armIndicator->setText("ARM STATUS");
    mode->setText("FLIGHT MODE");
    enableButtons(false);
}

void MainWindow::enableButtons(bool enable)
{
    armBtn->setEnabled(enable);
    enableHilBtn->setEnabled(enable);
    takeoffBtn->setEnabled(enable);
    setModeBtn->setEnabled(enable);
    modeBox->setEnabled(enable);
}
