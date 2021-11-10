#include "nivisadeviceinfo.h"
#include "ni-visa/visa.h"

#include <QStringList>
#include <QDebug>

NiVisaDeviceInfo::NiVisaDeviceInfo()
{

}

static QStringList findDevices()
{
    QStringList devList;

    ViChar instrDescriptor[256];
    ViUInt32 numInstrs;
    ViFindList findList;
    ViStatus status;
    ViSession defaultRM;//, instr;
    QStringList regExps = QStringList() << "USB?*RAW" << "USB?*INSTR" << "ASR?*INSTR";

    status = viOpenDefaultRM(&defaultRM);
    if ( status < VI_SUCCESS ) {
        qCritical() << "Could not open a session to the VISA Resource Manager!";
        return {};
    }

    QStringList descriptorStringList;

    for (int r = 0; r < regExps.size(); ++r) {
        status = viFindRsrc(defaultRM, regExps[r].toStdString().data(), &findList, &numInstrs, instrDescriptor);
        if (status == VI_SUCCESS) {
            // printf("An error occurred while finding resources.");
            // return;// status;
            descriptorStringList << QString::fromLatin1(instrDescriptor);

            for (int i = 0; i < numInstrs; ++i) {
                viFindNext(findList, instrDescriptor);
                descriptorStringList << QString::fromLatin1(instrDescriptor);
            }
        }
    }

    // QMessageBox::information(nullptr, QApplication::applicationName(), descriptorStringList.join("\n")); // DEBUG

    return descriptorStringList;

    return devList;
}
