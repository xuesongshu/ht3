#include <wtypes.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <memory>

#include "CyAPI.h"

int main(int argc, char** argv)
{
	std::shared_ptr<CCyUSBDevice> usb(new CCyUSBDevice);
	int icount = static_cast<int>(usb->DeviceCount());
	int ivid = 0, ipid = 0, ifd = 0;
	bool bnot_find = true;
	do {
		usb->Open(ifd);
		ifd++;
		ivid = usb->VendorID;
		ipid = usb->ProductID;
		if (ivid == 0x04b4 && ipid == 0x00f1) {
			bnot_find = false;
			break;
		}
	} while (ifd < icount);
	if (bnot_find) {
		printf("未找到CyUSB设备。程序即将退出。\r\n");
		int igetchar = getchar();
		(void)igetchar;
		return 0;
	}
	short imajor = 0, iminor = 0;
	CCyControlEndPoint* ctrl = usb->ControlEndPt;
	imajor = (usb->BcdDevice & 0xFF00) >> 8;
	iminor = usb->BcdDevice & 0x00FF;
	printf("product version: %d.%d\r\n", imajor, iminor);

	while (1) {
		printf("输入一个数字（255以内）发送给设备，输入0表示退出本程序。\r\n");
		int iinput = 0;
		std::cin >> iinput;
		if (iinput) {
			ctrl->Target = TGT_ENDPT;
			ctrl->ReqType = REQ_VENDOR;
			ctrl->Direction = DIR_TO_DEVICE;
			ctrl->ReqCode = static_cast<unsigned char>(iinput);
			ctrl->Value = 0;
			ctrl->Index = 0;
			ctrl->TimeOut = 100;
			long ilen = 512;
			uint8_t ibuf[512] = {0,};
			bool bxfer = ctrl->XferData(ibuf, ilen);
			if (bxfer) {
				printf("transfer %d success, received %d width length %d.\r\n", iinput, ibuf[0], ilen);
			} else {
				printf("transfer %d failed. error code: %d\r\n", iinput, ctrl->LastError);
			}
		} else {
			break;
		}
	}

	return 0;
}