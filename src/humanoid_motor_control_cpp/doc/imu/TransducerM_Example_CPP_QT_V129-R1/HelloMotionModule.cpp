/**
 * HelloMotionModule.cpp
 * @author COPYRIGHT(c) 2023 SYD Dynamics ApS
 * @see    HelloMotionModule.h for more descriptions.
 */
#include "HelloMotionModule.h"
#include "ui_HelloMotionModule.h"


// Motion Module Interface:
EasyObjectDictionary eOD;
EasyProfile          eP(&eOD);


HelloMotionModule::HelloMotionModule(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::HelloMotionModule)
{
    ui->setupUi(this);
    ui->groupBox_Example->setEnabled(false);
    serial = new QSerialPort;
}

HelloMotionModule::~HelloMotionModule()
{
    if(serial->isOpen())serial->close();
    delete ui;
}



//----------------------------------------------------------------------------------------
// Serial Receive Data Example
/**
 * @version  [This Example function applys to all TransducerM models]
 * @note     The example 'on_SerialRX()' here is a type of ISR (interrupt service routines),
 *           whenever serial port data/USB virtual COM data has received, it is called automatically by the system.
 *           It does not matter whether the received data contains full piece of information or not, the
 *           Communication Abstraction Layer (CAL) will help assemble and verify the data segment received.
 *           By modifying the 'serial->readAll()' below, you may port the code to any system supports C++ compiler.
 */
void HelloMotionModule::on_SerialRX(){
    QByteArray rxByteArray = serial->readAll();                // Step 1: read the received data buffer of the Serial Port
    char*  rxData = rxByteArray.data();                        //         and then convert it to data types acceptable by the
    int    rxSize = rxByteArray.size();                        //         Communication Abstraction Layer (CAL).

    Serial_ISR(rxData, rxSize);
    Serial_ISR(0, 0);                                          //         Flush packages in the ring buffer.
    Serial_ISR(0, 0);
    Serial_ISR(0, 0);
    Serial_ISR(0, 0);
    Serial_ISR(0, 0);
}

/**
 * @version  [This Example function applys to all TransducerM models]
 * @note   When correct data package has been received, assembled, and verified, the program goes into
 *         the switch-case branch here according to its data type.
 *         Here you may interface with your own application code and decide how to use the
 *         sensor data from TransducerM.
 */
void HelloMotionModule::Serial_ISR(char* rxData, int rxSize){
    Ep_Header header;
    if(EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header)){    // Step 2: Tell the CAL that new data has arrived.
                                                               //         It does not matter if the new data only contains a fraction
                                                               //         of a complete package, nor does it matter if the data is broken
                                                               //         during the transmission. On_RecvPkg() will only return EP_SUCC_
                                                               //         when a complete and correct package has arrived.

        // Example Reading of the Short ID of the device who send the data:
        uint32 fromId = header.fromId;                         // Step 3.1:  Now we are able to read the received payload data.
                                                               //            header.fromId tells us from which Motion Module the data comes.

        (void)fromId;                                          //            Supress "parameter unused" complier warning


        switch (header.cmd) {                                  // Step 3.2: header.cmd tells what kind of data is inside the payload.
        case EP_CMD_ACK_:{                                     //           We can use a switch() as demonstrated here to do different
            Ep_Ack ep_Ack;                                     //           tasks for different types of data.
            if(EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack)){
                if(ep_Ack.cmdAck == EP_CMD_SETTING_) ep_Settings_Ack = true;
            }
        }break;
        case EP_CMD_SETTING_:{                                 //           We can use a switch() as demonstrated here to do different
            Ep_Settings Ep_Settings;                           //           tasks for different types of data.
            if(EP_SUCC_ == eOD.Read_Ep_Settings(&Ep_Settings)){
                ep_Settings_Tmp     = Ep_Settings;
                ep_Settings_Receivd = true;
            }
        }break;
        case EP_CMD_STATUS_:{
            Ep_Status ep_Status;
            if(EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status)){
                int qos = ep_Status.sysState.bits.qos;        //  This is how you read the Quality-of-Service
                                                              //  For definition of the digits, please refer to the
                                                              //  defintion of 'Ep_Status_SysState' in EasyObjectDictionary.h
                (void)qos;
            }
        }break;
        case EP_CMD_Raw_GYRO_ACC_MAG_:{
            Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
            if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag)){

            }
        }break;
        case EP_CMD_Q_S1_S_:{
            Ep_Q_s1_s ep_Q_s1_s;
            if(EP_SUCC_ == eOD.Read_Ep_Q_s1_s(&ep_Q_s1_s)){

            }
        }break;
        case EP_CMD_Q_S1_E_:{
            Ep_Q_s1_e ep_Q_s1_e;
            if(EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e)){ // Step 3.3: If we decided that the received Quaternion should be used,
                                                            //           Here is an example of how to access the Quaternion data.
                float q1 = ep_Q_s1_e.q[0];
                float q2 = ep_Q_s1_e.q[1];
                float q3 = ep_Q_s1_e.q[2];
                float q4 = ep_Q_s1_e.q[3];
                uint32 timeStamp = ep_Q_s1_e.timeStamp;     //           TimeStamp indicates the time point (since the Module has been powered on),
                                                            //           when this particular set of Quaternion was calculated. (Unit: uS)
                                                            //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId  = ep_Q_s1_e.header.fromId; //           The ID indicates the device Short ID telling which Motion Module the data comes from.

                // Display the data on GUI:
                qDebug()<<"Q1="<<q1<<"  Q2="<<q2<<"  Q3="<<q3<<"  Q4="<<q4<<"  TimeStamp="<<timeStamp<<" Device Short ID"<<deviceId;
                ui->doubleSpinBox_RX_Q1->setValue( q1 );
                ui->doubleSpinBox_RX_Q2->setValue( q2 );
                ui->doubleSpinBox_RX_Q3->setValue( q3 );
                ui->doubleSpinBox_RX_Q4->setValue( q4 );
                ui->label_RX_Q_TimeStamp->setText(tr("%1").arg( timeStamp ));
                ui->label_RX_Q_ID->setText(tr("%1").arg(deviceId));
            }
        }break;
        case EP_CMD_EULER_S1_S_:{
            Ep_Euler_s1_s ep_Euler_s1_s;
            if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_s(&ep_Euler_s1_s)){

            }
        }break;
        case EP_CMD_EULER_S1_E_:{
            Ep_Euler_s1_e ep_Euler_s1_e;
            if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_e(&ep_Euler_s1_e)){

            }
        }break;
        case EP_CMD_RPY_:{
            Ep_RPY ep_RPY;
            if(EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY)){     //           Another Example reading of the received Roll Pitch and Yaw
                float roll  = ep_RPY.roll;
                float pitch = ep_RPY.pitch;
                float yaw   = ep_RPY.yaw;
                uint32 timeStamp = ep_RPY.timeStamp;      //           TimeStamp indicates the time point (since the Module has been powered on),
                                                          //           when this particular set of Roll-Pitch-Yaw was calculated. (Unit: uS)
                                                          //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId  = ep_RPY.header.fromId;  //           The ID indicates from wich IMU Module the data comes from.


                // Display the data on GUI:
                qDebug()<<"Roll="<<roll<<"  Pitch="<<pitch<<"  Yaw="<<yaw<<"  TimeStamp="<<timeStamp<<" Device Short ID"<<deviceId;
                ui->doubleSpinBox_RX_Roll->setValue( roll );
                ui->doubleSpinBox_RX_Pitch->setValue( pitch );
                ui->doubleSpinBox_RX_Yaw->setValue( yaw );
                ui->label_RX_RPY_TimeStamp->setText(tr("%1").arg( timeStamp ));
                ui->label_RX_RPY_ID->setText(tr("%1").arg(deviceId));
            }
        }break;
        case EP_CMD_GRAVITY_:{
            Ep_Gravity ep_Gravity;
            if(EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity)){

            }
        }break;
        case EP_CMD_CALIB_:{
            Ep_Calib ep_Calib;
            if(EP_SUCC_ == eOD.Read_Ep_Calib(&ep_Calib)){
                uint32 deviceId  = ep_Calib.header.fromId;
                uint8  calibType = ep_Calib.calibType;
                uint16 status    = ep_Calib.ctrlVal;

                (void)deviceId;                      // Remove 'variable unused' warning from compiler

                //--------------------------------------------
                // CalibCompass status report
                if(calibType == (3 + 0x10)){         // 3 means calibration type is CompassCalib.  +0x10 means it is its status report.
                    if(status == 1){
                        ui->label_RX_Calib_Status->setText(tr("Compass Calib: Operation Successful"));
                    }
                    else{
                        ui->label_RX_Calib_Status->setText(tr("Compass Calib: Operation Failed."));
                    }
                }
                // CalibCompass status report
                //--------------------------------------------

                //--------------------------------------------
                // CalibB status report
                else if(calibType == (4 + 0x10)){    // 4 means calibration type is CalibB.  +0x10 means it is its status report.
                    if(status == 1){                 // calibration successful.
                        ui->label_RX_Calib_Status->setText(tr("CalibB Successful!"));
                    }
                    else{
                        ui->label_RX_Calib_Status->setText(tr("CalibB Failed!"));
                    }
                }
                // CalibB status report
                //--------------------------------------------

                //--------------------------------------------
                // Forced Calibration status report
                else if(calibType == (5 + 0x10)){    // 5 means calibration type is Force Calibration,  +0x10 means it is its status report.
                    if(status == 0){                 // calibration successful.
                        ui->label_RX_Calib_Status->setText(tr("Forced Calibration Processing.."));
                    }
                    else if(status == 1){
                        ui->label_RX_Calib_Status->setText(tr("Forced Calibration Successful!"));
                    }
                    else if(status == 5){
                        ui->label_RX_Calib_Status->setText(tr("Forced Calibration successfully revoked!"));
                    }
                    else{
                        ui->label_RX_Calib_Status->setText(tr("Forced Calibration Failed!"));
                    }
                }
                // Forced Calibration status report
                //--------------------------------------------
            }
        }break;
        }

    }
}
// Serial Receive Data Example
//----------------------------------------------------------------------------------------








//----------------------------------------------------------------------------------------
//       ***   Code below exhibits advanced interface functions,       ***
//       ***   You may pick and choose the function in need and        ***
//       ***   disregard the rest.                                     ***
//----------------------------------------------------------------------------------------








//----------------------------------------------------------------------------------------
/**
 * @version [This Example applys to all TransducerM models]
 */
// Serial Send Request Example                                 // Note: sending a request to the Module Module is only required to feach data which is not set
                                                               //       to automatic sent. Such setting is done via the IMU Assistant.
void HelloMotionModule::on_pushButton_TX_Request_RPY_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();     // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                          //       You can also choose to boardcast the request without specifying the target device ID
    if(EP_SUCC_ == eOD.Write_Ep_Request(toId, EP_CMD_RPY_)){   // Step 2: Write the device ID into the data structure (i.e. the request itself) pending to be sent
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_REQUEST_;                  // Step 3: Define what type of data we are requesting for. (in this example, we are requesting Roll-Pitch-Yaw readings.
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){ // Step 4: create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));   // Step 5:  Send the package via Serial Port
            statusBar()->showMessage(tr("RPY Request Sent to %1 Successfully").arg(toId));
        }
    }
}

/**
 * @version [This Example applys to all TransducerM models]
 */
void HelloMotionModule::on_pushButton_TX_Request_Quaternion_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();      // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                           //       You can also choose to boardcast the request without specifying the target device ID
    if(EP_SUCC_ == eOD.Write_Ep_Request(toId, EP_CMD_Q_S1_E_)){ // Step 2: Write the device ID into the data structure (i.e. the request itself) pending to be sent
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_REQUEST_;                   // Step 3: Define what type of data we are requesting for. (in this example, we are requesting Quaternion readings.
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){ // Step 4: create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));    // Step 5:  Send the package via Serial Port
            statusBar()->showMessage(tr("RPY Request Sent to %1 Successfully").arg(toId));
        }
    }
}


/**
 * Start collecting data for correcting heading (Yaw) using local magnetic field measured by magnetometer.
 * @version [This Example applys to TransducerM TM5xx series models only]
 * @note    It is recommended, if possible, during the data collection process to rotate the TransducerM
 *          around Z axis (axis perpendicular to the horizontal plane)  to increase the accuracy of results;
 *          the rotation can be a small angle such as 3-5 degree, 30 degrees, or a large angle such as a full turn or more).
 *          The results are saved in TransducerM RAM and will be lost during power cycle.
 */
void HelloMotionModule::on_pushButton_TX_CalibCompass_AutoFix_Start_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();                        // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                                             //         You can also choose to boardcast the request without specifying the target device ID

                                                                                  // Step 2 : Set parameter
    uint32 calibrationTime = 90000;                                               //         Unit: ms. Maximum calibration time. Range (0, 90000] reflecting 0~90 seconds. 0 means infinite time.
                                                                                  //
    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 3, 0x0a, 0xaa32, calibrationTime, 0)){//         The parameter value "3, 0x0a, 0xaa32" and "0" are fixed and do not change them.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                       // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){    // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));                      // Step 5: Send the package via Serial Port
            ui->label_RX_Calib_Status->setText(tr("Compass AutoFix start command sent."));
        }
    }
}


/**
 * Finish collecting data for correcting heading (Yaw) using magnetitic north measured by magnetometer and apply results.
 * @version [This Example applys to TransducerM TM5xx series models only]
 * @note    When the AutoFix is finished, a status report is sent back to the host, telling whether it is
 *          successful or not. Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 *          The longer time interval between AutoFix Start and Stop command, the better the results will be. It
 *          is recommended to last at least 2 seconds before Stop command is sent.
 */
void HelloMotionModule::on_pushButton_TX_CalibCompass_AutoFix_Stop_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();                        // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                                             //         You can also choose to boardcast the request without specifying the target device ID

                                                                                  // Step 2 : Set parameter
    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 3, 0x0b, 0xaa32, 0, 0)){              //         The parameter value "3, 0x0b, 0xaa32, 0, 0" are fixed and do not change them.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                       // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){    // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));                      // Step 5: Send the package via Serial Port
            ui->label_RX_Calib_Status->setText(tr("Compass AutoFix stop command sent."));
        }
    }
}


/**
 * Manually assign a heading (Yaw) angle to TransducerM.
 * @version [This Example applys to TransducerM TM5xx series models only]
 * @note    When the ManualFix is finished, a status report is sent back to the host, telling whether it is
 *          successful or not. Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 */
void HelloMotionModule::on_pushButton_TX_CalibCompass_ManualFix_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();                        // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                                             //         You can also choose to boardcast the request without specifying the target device ID

                                                                                  // Step 2 : Set parameter
    uint32 newHeading =                                                           //         Unit: 0.001 degree. Value range: 0-360 degree. Manually set TransducerM Heading (Yaw) to this value.
            1000*(ui->doubleSpinBox_TX_CalibCompass_ManualFix_Value->value());
    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 3, 0x0c, 0xaa32, newHeading, 0)){     //         The parameter value "3, 0x0c, 0xaa32" and "0" are fixed and do not change them.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                       // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){    // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));                      // Step 5: Send the package via Serial Port
            ui->label_RX_Calib_Status->setText(tr("Compass ManualFix command sent."));
        }
    }
}


/**
 * @version [This Example is for TM411C and later. For TM331C, there will be no status report senting back, otherwise it works fine]
 *          [This Example is also fit for TransducerM produced since January 2019]
 *
 * @brief Start Calibration (CalibB)
 *        This function performes the same function as the "CalibB" button in the ImuAssistant V3.x.x and later versions,
 *        During this calibration, the module will stop outputting orientation data.
 *        If the calibration fails, the module will not adopt the result, and nothing will be changed.
 *
 * ** Warning **  CalibB will save results into Flash Memory of the module, changing the behavior
 *        of the module permanently. Please keep the module absolutely stationary while doing this to
 *        ensure a correct calibration!
 *
 * @note  When the calibration is finished, a status report is sent back to the host, telling whether it is
 *        successful or not. Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 */
void HelloMotionModule::on_pushButton_TX_CalibB_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();        // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                             //         You can also choose to boardcast the request without specifying the target device ID
    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 4, 2, 0xaa35, 0, 0)){ // Step 2: Write the device ID and calibration cmd parameters into the data structure,
                                                                  //         the parameter value "4, 1, 0xaa35, 0, 0" are fixed and do not change them.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                       // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){ // Step 4: create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));      // Step 5:  Send the package via Serial Port
            statusBar()->showMessage(tr("Calibration Command Sent to %1 Successfully").arg(toId));
            ui->label_RX_Calib_Status->setText(tr("Calibration result pending..."));
        }
    }
}


/**
 * @brief    Start continuous Forced Calibration (a.k.a Run-time Static Calibration)
 * @version  This function is available with TransducerM with firmware version V5.1.x and later (except V2x.x.x).
 * @see      Refer to user guide (Chapter 'Write Your Own Communication Library/EasyProtocol/ObjectTypes/Calibration') for details.
 *
 * @note Calibration status reports are sent back to the host, telling whether it is successful or not.
 *       Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 *       If the host does not get a status report (very rare, but it is possible that either the calibration
 *       command or the status report package is lost during the communication), Simply send the command again.
 */
void HelloMotionModule::on_pushButton_TX_CalibForce_Continuous_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();                        // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                                             //         You can also choose to boardcast the request without specifying the target device ID

    float  calibLimit = ui->doubleSpinBox_TX_CalibForce_Limits->value();          //         Set calibLimit between 0.05 and 5.00 degree/s. Motion data within the range of +-calibLimit will be accepted as valid calibration data.
    float  calibConfidence = ui->doubleSpinBox_TX_CalibForce_Confidence->value(); //         Set calibConfidence between 0 and 100 (meaning 0% - 100%). 0 means auto confidence mode.
    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 5, 3, 0xaa65, (uint32)(calibLimit*100), (uint32)(calibConfidence*100))){
                                                                                  //         the parameter value "5, 3, 0xaa65" are fixed and do not change them.
                                                                                  //         the parameter "(uint32)(calibLimit*100)" is a conversion and do not change it.
                                                                                  //         the parameter "(uint32)(calibConfidence*100)" is a conversion and do not change it.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                       // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){    // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));                      // Step 5: Send the package via Serial Port

            ui->label_RX_Calib_Status->setText(tr("Forced Calibration start command sent."));
        }
    }
}


/**
 * @brief    Finish continuous Forced Calibration (a.k.a Run-time Static Calibration).
 * @version  This function is available with TransducerM with firmware version V5.1.x and later (except V2x.x.x).
 * @see      Refer to user guide (Chapter 'Write Your Own Communication Library/EasyProtocol/ObjectTypes/Calibration') for details.
 *
 * @note Calibration status reports are sent back to the host, telling whether it is successful or not.
 *       Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 */
void HelloMotionModule::on_pushButton_TX_CalibForce_Finish_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();                        // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                                             //         You can also choose to boardcast the request without specifying the target device ID

    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 5, 4, 0xaa65, 0, 0)){
                                                                                  //         the parameter value "5, 4, 0xaa65" are fixed and do not change them.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                       // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){    // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));                      // Step 5: Send the package via Serial Port

            ui->label_RX_Calib_Status->setText(tr("Forced Calibration finish command sent."));
        }
    }
}


/**
 * @brief Start Forced Calibration (a.k.a Run-time Static Calibration) and finish automatically.
 *        This calibration forces the TransducerM to recalibrate itself according to current motion,
 *        by assuming the module is sitting "stationary". This calibration will only return success if
 *        the rotation rate during the calibration is within a given limitation.
 *        The module can run this calibration at the same time while it is outputting orientation data.
 *        If the calibration fails, the module will not adopt the result, and nothing will be changed.
 *        If the calibration is successful, you can still abandon the result. Please refer to the
 *        on_pushButton_TX_CalibForce_Reset_clicked() function below.
 *
 *        For example, if the module is rotating at a constant speed of 0.2 degree/second around a certain axis,
 *        after successfully performing this calibration, the module will output zero rotation speed.
 *
 * @see   Refer to user guide (Chapter 'Write Your Own Communication Library/EasyProtocol/ObjectTypes/Calibration') for details.
 * @version  This function is available with TransducerM with firmware version V3.1.6 and later (except V2x.x.x).
 *           For  firmware version V5.1.x and later, we do not recommend this function. Use
 *           'on_pushButton_TX_CalibForce_Continuous_clicked()' instead.
 *
 * @note For safety reasons, the result of this calibration is only stored in RAM,
 *       which means the result will be lost after a power cycle.
 * @note When the calibration is finished, a status report is sent back to the host, telling whether it is
 *       successful or not. Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 *       If the host doesnot get a status report (very rare, but it is possible that either the calibration
 *       command or the status report package is lost during the communication), Simply send the command again.
 */
void HelloMotionModule::on_pushButton_TX_CalibForce_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();                     // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                                          //         You can also choose to boardcast the request without specifying the target device ID

    float  calibLimit = ui->doubleSpinBox_TX_CalibForce_Limits->value();       //         Set calibLimit between 0.05 and 5.00 degree/s. Motion data within the range of +-calibLimit will be accepted as valid calibration data
                                                                               //         A recommended value for calibLimit is 0.5. The unit for the float number calibLimit is degree/s.

    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 5, 2, 0xaa65, (uint32)(calibLimit*100), 0)){ // Step 2: Write the device ID and calibration cmd parameters into the data structure,
                                                                               //                   the parameter value "5, 2, 0xaa65" are fixed and do not change them.
                                                                               //                   the parameter "(uint32)(calibLimit*100)" is a conversion and do not change it.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                    // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){ // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));                   // Step 5: Send the package via Serial Port

            ui->label_RX_Calib_Status->setText(tr("Forced Calibration result pending..."));
        }
    }
}


/**
 * @brief    To revoke the result of the Forced Calibration  (a.k.a Run-time Static Calibration),
 *           making the module behave the same as before the Force Calibration.
 * @version  This function is available with TransducerM with firmware version V3.1.6 and later (except V2x.x.x).
 *
 * @note     When the calibration result is successfully revoked (abandoned), a status report is sent
 *           back to the host. Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 */
void HelloMotionModule::on_pushButton_TX_CalibForce_Reset_clicked()
{
    uint16 toId = ui->spinBox_TX_Request_ShortId->value();                  // Step 1: Get the destination device ID to which the request is sent
    //uint16 toId = EP_ID_BROADCAST_;                                       //         You can also choose to boardcast the request without specifying the target device ID
    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 5, 1, 0xaa65, 0, 0)){           // Step 2: Write the device ID and calibration cmd parameters into the data structure,
                                                                            //         the parameter value "5, 2, 0xaa65, 0, 0" are fixed and do not change them.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                 // Step 3  Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){ // Step 4: create a package out of the data structure (i.e. the payload) to be sent
            serial->write(QByteArray(txPkgData, txPkgSize));                // Step 5: Send the package via Serial Port
            ui->label_RX_Calib_Status->setText(tr("Forced Calibration resetting..."));
        }
    }
}


/**
 * @version [This Example is for TM331C, TM333C, TM411C, TM352, TM353, TM500, TM510 and later hardware versions.
 *           Pleae also pay attention to the '** IMPORTANT**' notes written below]
 *
 * @brief Example Code - Change Setting
 *
 * This Example has the same effect as changing the settings of TransducerM on ImuAssistant GUI.
 *
 * Basic Idea: First check the firmware version running inside TransducerM,
 *             adjust automatically when legacy support should be enabled.
 *             Then read the current settings from the module, then
 *             make the wanted changes (without modifying the others),
 *             and send back to the TransducerM module, finally wait for an
 *             acknowledge to confirm operation successful.
 *
 * Functions and Global Variable involved in changing the setting:
 *      void on_pushButton_TX_RX_ChangeSetting_clicked();  <-- This example starts by calling this function.
 *      void on_SerialRX();            <-- This function is called automatically when there is serial data received.
 *      int  Example_ChangeSetting();  <-- This function runs the actual setting logic. The wanted setting is written there.
 *      Ep_Settings    ep_Settings_Tmp;
 *      bool           ep_Settings_Receivd;
 *      bool           ep_Settings_Ack;
 *
 */
void HelloMotionModule::on_pushButton_TX_RX_ChangeSetting_clicked()
{
    if( ! Example_ChangeSetting() ){
        ui->label_RX_Setting_Status->setText(tr("Change Setting Failed. Is the serial port busy?"));
    }
}


/**
 * @brief HelloMotionModule::Example_ChangeSetting
 * @return   true:  Operation Successful.
 *           false: Operation failed.
 */
int  HelloMotionModule::Example_ChangeSetting()
{
    // Read TransducerM Firmware Version
    int tryCounter = 0;
    do{
        {
            QTest::qWait(300);        // Wait 300ms
        }

        {
            uint16 toId = ui->spinBox_TX_Request_ShortId->value();    // Step 1: Get the destination device ID to which the request is sent
            //uint16 toId = EP_ID_BROADCAST_;                         //       You can also choose to boardcast the request without specifying the target device ID
            if(EP_SUCC_ == eOD.Write_Ep_Request(toId, EP_CMD_ID_)){   // Write the data into data structure (i.e. the request itself) pending to be sent
                EP_ID_TYPE_  txToId;
                char*        txPkgData;
                int          txPkgSize;
                EP_CMD_TYPE_ txCmd = EP_CMD_REQUEST_;
                if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){ // Create a package out of the data structure (i.e. the payload) to be sent
                    serial->write(QByteArray(txPkgData, txPkgSize));   //Send the package via Serial Port
                    ui->label_RX_Setting_Status->setText(tr("Requesting Firmware Version..."));
                }
                else{
                    return false;
                }
            }
            else{
                return false;
            }
        }

        // Wait and Receive Reply from TransducerM
        {
            QTest::qWait(300);        // Wait 300ms, during which  on_SerialRX() will be called upon Serial Data Receive.
        }

        // Load Setting according to different Firmware Version
        Ep_Id epId;
        eOD.Read_Ep_Id(&epId);

        // Try Maximum 5 times to increase robustness
        if((++tryCounter) >= 5) return false;
    }
    while(eOD.Get_Version() < 0);


    // Change the Setting
    tryCounter = 0;
    do{
        {
            QTest::qWait(300);        // Wait 300ms
        }

        // Firstly, Send a Request To Fetch TransducerM Current Setting:
        {
            ep_Settings_Receivd = 0;
            uint16 toId = ui->spinBox_TX_Request_ShortId->value();       // Get the destination device ID to which the request is sent
            //uint16 toId = EP_ID_BROADCAST_;                            //       You can also choose to boardcast the request without specifying the target device ID
            if(EP_SUCC_ == eOD.Write_Ep_Request(toId, EP_CMD_SETTING_)){ // Write the data into data structure (i.e. the request itself) pending to be sent
                EP_ID_TYPE_  txToId;
                char*        txPkgData;
                int          txPkgSize;
                EP_CMD_TYPE_ txCmd = EP_CMD_REQUEST_;
                if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){
                    serial->write(QByteArray(txPkgData, txPkgSize));
                    ui->label_RX_Setting_Status->setText(tr("Requesting Current Setting..."));
                }
                else{
                    continue;
                }
            }
            else{
                continue;
            }
        }

        // Wait and Receive Reply from TransducerM
        {
            QTest::qWait(300);        // Wait 300ms, during which  on_SerialRX() will be called upon Serial Data Receive.
        }

        // Make Changes as we want
        {
            if(ep_Settings_Receivd){  // This flag is set in on_SerialRX()

                // Note: The recommended value of gain_Acc for AGV is 210
                //       The recommended value of gain_Acc for UAV is 300
                //       We do not recommend setting gain_Acc to a smaller value than 200.
                ep_Settings_Tmp.gain_Acc = 210;            // Set gain for acceleromter to 2.1 (i.e. 210/100=2.1, the TransducerM will divide the value by 100 internally)
                ep_Settings_Tmp.switches.bits.output_RPY = 1;      // Turn on Roll-Pitch-Raw output
                ep_Settings_Tmp.switches.bits.output_Q_s1_e = 0;       // Turn off Quaternion output
                ep_Settings_Tmp.switches.bits.request_acknowledge  =  1;  // TransducerM must acknowledge the change of setting once it has been made.
                ep_Settings_Tmp.switches.bits.save_Setting_Permanently = 1; // Save Setting to Flash Memory of TransducerM

                // Any other settings will remain unchanged.
            }
            else{
                continue;
            }
        }

        // Send the new setting back to TransducerM
        {
            ep_Settings_Ack = false;
            uint16 toId = ep_Settings_Tmp.header.fromId;                    // EP_ID_BROADCAST_ should not be used here.
            if(EP_SUCC_ == eOD.Write_Ep_Settings(toId, ep_Settings_Tmp)){
                EP_ID_TYPE_  txToId;
                char*        txPkgData;
                int          txPkgSize;
                EP_CMD_TYPE_ txCmd = EP_CMD_SETTING_;
                if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)){
                    serial->write(QByteArray(txPkgData, txPkgSize));
                    ui->label_RX_Setting_Status->setText(tr("Current Setting Received. Changes made. Now sending back..."));
                }
                else{
                    continue;
                }
            }
            else{
                continue;
            }
        }

        // Wait and receive Acknowledge sent from TransducerM
        {
            QTest::qWait(1500);       // Wait 1500ms, during which  on_SerialRX() will be called upon Serial Data Receive.
        }

        // Verify Acknowledge
        {
            if(ep_Settings_Ack){     // This flag is set in 'on_SerialRX()'
                ui->label_RX_Setting_Status->setText(tr("Acknowledge received. Configuration Successful. ALL DONE!"));
                return true;         // Setting is successfully Finished.
            }
        }

    } while((++tryCounter) < 5);     // Try Maximum 5 times to increase robustness
    return false;
}

// Serial Send Request Example
//----------------------------------------------------------------------------------------














//----------------------------------------------------------------------------------------
// Trivial GUI utility functions, Not important
void HelloMotionModule::on_pushButton_Serialport_Open_clicked()
{
    QString portName = ui->lineEdit_Serialport_Name->text();
    serial->setPortName(portName);
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    if(serial->open(QIODevice::ReadWrite)){
       QObject::connect(serial, SIGNAL(readyRead()), this, SLOT(on_SerialRX()));
       ui->pushButton_Serialport_Open->setEnabled(false);
       ui->lineEdit_Serialport_Name->setEnabled(false);
       ui->groupBox_Example->setEnabled(true);
       statusBar()->showMessage(portName + tr(" opened successfully"));
    }
    else{
       statusBar()->showMessage(portName + tr(" failed to open"));
    }
}

void HelloMotionModule::on_pushButton_UseBroadcastId_clicked()
{
    ui->spinBox_TX_Request_ShortId->setValue(EP_ID_BROADCAST_);
    ui->pushButton_UseBroadcastId->setEnabled(false);
}

void HelloMotionModule::on_spinBox_TX_Request_ShortId_valueChanged(int arg1)
{
    Q_UNUSED(arg1);
    ui->pushButton_UseBroadcastId->setEnabled(true);
}
// Trivial GUI utility functions, Not important
//----------------------------------------------------------------------------------------


