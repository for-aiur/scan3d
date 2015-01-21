#include <calibration_manager.h>
#include <calibration_result.h>

void main()
{
	std::vector<TCALPOINT> ptArray;
	CalibrationManager cm;
	FillRawDataFromIni("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/RawCal.ini","RawCalParams_VirtualCamera123456002",ptArray);
	cm.calParam[0].CalcCalibrationParams(ptArray);
	int a = 100;
}