/*=====================================================================

 QGroundControl Open Source Ground Control Station

 (c) 2009 - 2014 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

 This file is part of the QGROUNDCONTROL project

 QGROUNDCONTROL is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 QGROUNDCONTROL is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/



#ifndef Vehicle_H
#define Vehicle_H

#include <QObject>


class UAS;
class UASInterface;
class FirmwarePlugin;
class FirmwarePluginManager;
class AutoPilotPlugin;
class AutoPilotPluginManager;
class MissionManager;
class ParameterLoader;
class JoystickManager;
class UASMessage;


class Vehicle : public QObject
{
    Q_OBJECT

public:
    Vehicle(LinkInterface*          link,
            int                     vehicleId,
            MAV_AUTOPILOT           firmwareType,
            MAV_TYPE                vehicleType,
            FirmwarePluginManager*  firmwarePluginManager,
            AutoPilotPluginManager* autopilotPluginManager,
            JoystickManager*        joystickManager);
    ~Vehicle();



    /// Sends a message to all links accociated with this vehicle
    void sendMessage(mavlink_message_t message);



    typedef enum {
        MessageNone,
        MessageNormal,
        MessageWarning,
        MessageError
    } MessageType_t;

    enum {
        ROLL_CHANGED,
        PITCH_CHANGED,
        HEADING_CHANGED,
        GROUNDSPEED_CHANGED,
        AIRSPEED_CHANGED,
        CLIMBRATE_CHANGED,
        ALTITUDERELATIVE_CHANGED,
        ALTITUDEWGS84_CHANGED,
        ALTITUDEAMSL_CHANGED
    };



private slots:
    void _mavlinkMessageReceived(LinkInterface* link, mavlink_message_t message);
    void _linkInactiveOrDeleted(LinkInterface* link);
    void _sendMessage(mavlink_message_t message);
    void _sendMessageOnLink(LinkInterface* link, mavlink_message_t message);
    void _sendMessageMultipleNext(void);
    void _addNewMapTrajectoryPoint(void);
    void _parametersReady(bool parametersReady);
    void _remoteControlRSSIChanged(uint8_t rssi);

    void _handleTextMessage                 (int newCount);
    void _handletextMessageReceived         (UASMessage* message);
    /** @brief Attitude from main autopilot / system state */
    void _updateAttitude                    (UASInterface* uas, double roll, double pitch, double yaw, quint64 timestamp);
    /** @brief Attitude from one specific component / redundant autopilot */
    void _updateAttitude                    (UASInterface* uas, int component, double roll, double pitch, double yaw, quint64 timestamp);
    void _updateSpeed                       (UASInterface* uas, double _groundSpeed, double _airSpeed, quint64 timestamp);
    void _updateAltitude                    (UASInterface* uas, double _altitudeAMSL, double _altitudeWGS84, double _altitudeRelative, double _climbRate, quint64 timestamp);
    void _updateNavigationControllerErrors  (UASInterface* uas, double altitudeError, double speedError, double xtrackError);
    void _updateNavigationControllerData    (UASInterface *uas, float navRoll, float navPitch, float navBearing, float targetBearing, float targetDistance);
    void _checkUpdate                       ();
    void _updateBatteryRemaining            (UASInterface*, double voltage, double, double percent, int);
    void _updateBatteryConsumedChanged      (UASInterface*, double current_consumed);
    void _updateState                       (UASInterface* system, QString name, QString description);
    void _setSatelliteCount                 (double val, QString name);
    void _setSatRawHDOP                     (double val);
    void _setSatRawVDOP                     (double val);
    void _setSatRawCOG                      (double val);
    void _setSatLoc                         (UASInterface* uas, int fix);
    /** @brief A new camera image has arrived */
    void _imageReady                        (UASInterface* uas);
    void _connectionLostTimeout(void);

private:
    bool _containsLink(LinkInterface* link);
    void _addLink(LinkInterface* link);
    void _loadSettings(void);
    void _saveSettings(void);
    void _startJoystick(bool start);
    void _handleHomePosition(mavlink_message_t& message);
    void _handleHeartbeat(mavlink_message_t& message);
    void _handleRCChannels(mavlink_message_t& message);
    void _handleRCChannelsRaw(mavlink_message_t& message);
    void _missionManagerError(int errorCode, const QString& errorMsg);
    void _mapTrajectoryStart(void);
    void _mapTrajectoryStop(void);
    void _connectionActive(void);
    void _say(const QString& text, int severity);

    void    _addChange                      (int id);
    float   _oneDecimal                     (float value);

private:
    int     _id;            ///< Mavlink system id
    bool    _active;

    MAV_AUTOPILOT       _firmwareType;
    MAV_TYPE            _vehicleType;
    FirmwarePlugin*     _firmwarePlugin;
    AutoPilotPlugin*    _autopilotPlugin;
    MAVLinkProtocol*    _mavlink;

    QList<LinkInterface*> _links;

    JoystickMode_t  _joystickMode;
    bool            _joystickEnabled;

    UAS* _uas;

    QGeoCoordinate  _coordinate;
    bool            _coordinateValid;       ///< true: vehicle has 3d lock and therefore valid location

    bool            _homePositionAvailable;
    QGeoCoordinate  _homePosition;

    UASInterface*   _mav;
    int             _currentMessageCount;
    int             _messageCount;
    int             _currentErrorCount;
    int             _currentWarningCount;
    int             _currentNormalCount;
    MessageType_t   _currentMessageType;
    QString         _latestError;
    float           _roll;
    float           _pitch;
    float           _heading;
    float           _altitudeAMSL;
    float           _altitudeWGS84;
    float           _altitudeRelative;
    float           _groundSpeed;
    float           _airSpeed;
    float           _climbRate;
    float           _navigationAltitudeError;
    float           _navigationSpeedError;
    float           _navigationCrosstrackError;
    float           _navigationTargetBearing;
    QTimer*         _refreshTimer;
    QList<int>      _changes;
    double          _batteryVoltage;
    double          _batteryPercent;
    double          _batteryConsumed;
    QString         _currentState;
    int             _satelliteCount;
    double          _satRawHDOP;
    double          _satRawVDOP;
    double          _satRawCOG;
    int             _satelliteLock;
    int             _updateCount;
    QString         _formatedMessage;
    int             _rcRSSI;
    double          _rcRSSIstore;

    // Lost connection handling
    bool                _connectionLost;
    bool                _connectionLostEnabled;
    static const int    _connectionLostTimeoutMSecs = 3500;  // Signal connection lost after 3.5 seconds of missed heartbeat
    QTimer              _connectionLostTimer;

    MissionManager*     _missionManager;
    bool                _missionManagerInitialRequestComplete;

    ParameterLoader*    _parameterLoader;

    bool    _armed;         ///< true: vehicle is armed
    uint8_t _base_mode;     ///< base_mode from HEARTBEAT
    uint32_t _custom_mode;  ///< custom_mode from HEARTBEAT

    /// Used to store a message being sent by sendMessageMultiple
    typedef struct {
        mavlink_message_t   message;    ///< Message to send multiple times
        int                 retryCount; ///< Number of retries left
    } SendMessageMultipleInfo_t;

    QList<SendMessageMultipleInfo_t> _sendMessageMultipleList;    ///< List of messages being sent multiple times

    static const int _sendMessageMultipleRetries = 5;
    static const int _sendMessageMultipleIntraMessageDelay = 500;

    QTimer  _sendMultipleTimer;
    int     _nextSendMessageMultipleIndex;

    QTimer              _mapTrajectoryTimer;
    QmlObjectListModel  _mapTrajectoryList;
    QGeoCoordinate      _mapTrajectoryLastCoordinate;
    bool                _mapTrajectoryHaveFirstCoordinate;
    static const int    _mapTrajectoryMsecsBetweenPoints = 1000;

    FirmwarePluginManager*      _firmwarePluginManager;
    AutoPilotPluginManager*     _autopilotPluginManager;
    JoystickManager*            _joystickManager;

    int                         _flowImageIndex;

    bool _allLinksInactiveSent; ///< true: allLinkInactive signal already sent one time

    // Settings keys
    static const char* _settingsGroup;
    static const char* _joystickModeSettingsKey;
    static const char* _joystickEnabledSettingsKey;
};
#endif

