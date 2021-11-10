#ifndef NIVISADEVICE_H
#define NIVISADEVICE_H

#include "nivisadeviceinfo.h"

#include <QString>

class NiVisaDevice
{
public:
    NiVisaDevice();

    QString getDeviceId() const { return m_deviceId; }
    virtual NiVisaDeviceInfo getInfo() const = 0;
    virtual bool connectToDevice() = 0;
    virtual void disconnect() = 0;

private:
    QString m_deviceId;
};

#endif // NIVISADEVICE_H
