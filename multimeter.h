#pragma once

#include <visa.h>

class MultiMeter
{
private:
	const char xVISAAddress[43] = "USB0::0x0957::0x0A07::MY48001873::0::INSTR";
	const char yVISAAddress[43] = "USB0::0x0957::0x0A07::MY48001709::0::INSTR";
	const char zVISAAddress[43] = "USB0::0x0957::0x0A07::MY48002121::0::INSTR";

    ViConstRsrc yDMM = xVISAAddress;
    ViConstRsrc xDMM = yVISAAddress;
    ViConstRsrc zDMM = zVISAAddress;

    ViStatus Status;

    ViSession defaultRMx, vix;
    ViSession defaultRMy, viy;
    ViSession defaultRMz, viz;
public:
    MultiMeter()
    {
        Status = viOpenDefaultRM(&defaultRMx);
        Status |= viOpen(defaultRMx, xDMM, VI_NULL, VI_NULL, &vix);

        Status = viOpenDefaultRM(&defaultRMy);
        Status |= viOpen(defaultRMy, yDMM, VI_NULL, VI_NULL, &viy);

        Status = viOpenDefaultRM(&defaultRMz);
        Status |= viOpen(defaultRMz, zDMM, VI_NULL, VI_NULL, &viz);

        Status |= viPrintf(vix, "*RST\n");
        Status |= viPrintf(viy, "*RST\n");
        Status |= viPrintf(viz, "*RST\n");

        Status |= viPrintf(vix, "SENS:VOLT:DC:NPLC 2\n");
        Status |= viPrintf(viy, "SENS:VOLT:DC:NPLC 2\n");
        Status |= viPrintf(viz, "SENS:VOLT:DC:NPLC 2\n");

        Status |= viPrintf(vix, "VOLT:DC:IMP:AUTO 1\n");
        Status |= viPrintf(viy, "VOLT:DC:IMP:AUTO 1\n");
        Status |= viPrintf(viz, "VOLT:DC:IMP:AUTO 1\n");

        Status |= viPrintf(vix, "VOLT:RANG 10\n");
        Status |= viPrintf(viy, "VOLT:RANG 10\n");
        Status |= viPrintf(viz, "VOLT:RANG 10\n");

        Status |= viPrintf(vix, "TRIG:SOUR IMM\n");
        Status |= viPrintf(viy, "TRIG:SOUR IMM\n");
        Status |= viPrintf(viz, "TRIG:SOUR IMM\n");

    }

    double measureBx()
    {
        double res = 0;

        for(int i = 0; i < 5; i++)
        {
            double temp;
            Status |= viPrintf(vix, "READ?\n");
            Status |= viScanf(vix, "%lf", &temp);
            res += temp / 5.0;
        }
        return 100. * res;
    }

    double measureBy()
    {
        double res = 0;

        for(int i = 0; i < 5; i++)
        {
            double temp;
            Status |= viPrintf(viy, "READ?\n");
            Status |= viScanf(viy, "%lf", &temp);
            res += temp / 5.0;
        }
        return 100. * res;
    }

    double measureBz()
    {
        double res = 0;

        for(int i = 0; i < 5; i++)
        {
            double temp;
            Status |= viPrintf(viz, "READ?\n");
            Status |= viScanf(viz, "%lf", &temp);
            res += temp / 5.0;
        }
        return 100. * res;
    }

	void getVISA_Addr()
	{
		std::cout << " X : " << xVISAAddress << std::endl;
		std::cout << " Y : " << yVISAAddress << std::endl;
		std::cout << " Z : " << zVISAAddress << std::endl;
	}

    ~MultiMeter()
    {
        Status |= viClose(vix);
        Status |= viClose(defaultRMx);
        Status |= viClose(viy);
        Status |= viClose(defaultRMy);
        Status |= viClose(viz);
        Status |= viClose(defaultRMz);
    }
};
