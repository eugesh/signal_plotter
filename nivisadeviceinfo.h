#ifndef NIVISADEVICEINFO_H
#define NIVISADEVICEINFO_H

class QStringList;

class NiVisaDeviceInfo
{
public:
    NiVisaDeviceInfo();

    static QStringList findDevices();
};

#endif // NIVISADEVICEINFO_H
