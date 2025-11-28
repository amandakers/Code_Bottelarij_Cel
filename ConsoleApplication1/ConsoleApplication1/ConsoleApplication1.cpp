#include <iostream>
#include "modbus.h"
#include "GalaxyIncludes.h"
#include "stdafx.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <windows.h>

using namespace std;

struct THREAD_PARAM
{
	bool bRun;
	CGXStreamPointer pStream;
};

// ROI-config struct moved to file scope so threads can use it
struct ROIConfig {
	int width;
	int height;
	int offsetX;
	int offsetY;
};

// Global critical section to guard ROI changes (manual vs auto)
static CRITICAL_SECTION g_roiCs;

// Thread Function (image grab)
DWORD WINAPI GrabImgThread(LPVOID lpParam)
{
	THREAD_PARAM* pThreadParam = (THREAD_PARAM*)lpParam;
	if (NULL == pThreadParam)
	{
		cout << "lpParam is NULL!" << endl;
		system("pause");
		return 0;
	}
	modbus_t* ctx = modbus_new_tcp("127.0.0.1", 502); // lokale verbinding
	if (!ctx) return -1;

	if (modbus_connect(ctx) == -1) {
		std::cerr << "Verbinden mislukt: " << modbus_strerror(errno) << "\n";
		modbus_free(ctx);
		std::cin.get();
		return -1;
	}

	while (pThreadParam->bRun)
	{
		try
		{

			CImageDataPointer pImgData = pThreadParam->pStream->GetImage(2000);
			if (GX_FRAME_STATUS_SUCCESS == pImgData->GetStatus())
			{
				//After acquiring the image, obtain the frame attribute control object to obtain the frame data
				IGXFeatureControl* pChunkDataFeatureControl = pImgData->GetChunkDataFeatureControl();
				int64_t i64ChunkFID = pChunkDataFeatureControl->GetIntFeature("ChunkFrameID")->GetValue();

				cout << "<Successful acquisition: Width: " << pImgData->GetWidth() <<
					" Height: " << pImgData->GetHeight() << " i64ChunkFID: " << i64ChunkFID <<
					">" << endl;


				void* pBuffer = pImgData->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);
				unsigned char* rgbData = (unsigned char*)pBuffer;

				int width = pImgData->GetWidth();
				int height = pImgData->GetHeight();

				uint64_t sumR = 0, sumG = 0, sumB = 0;
				unsigned char minR = 255, minG = 255, minB = 255;

				// Histogram arrays (256 mogelijke intensiteiten per kanaal)
				std::vector<int> histR(256, 0);
				std::vector<int> histG(256, 0);
				std::vector<int> histB(256, 0);

				for (int y = 0; y < height; y++)
				{
					for (int x = 0; x < width; x++)
					{
						int index = (y * width + x) * 3;
						unsigned char R = rgbData[index + 0];
						unsigned char G = rgbData[index + 1];
						unsigned char B = rgbData[index + 2];

						sumR += R;
						sumG += G;
						sumB += B;

						if (R < minR) minR = R;
						if (G < minG) minG = G;
						if (B < minB) minB = B;

						// histogram vullen
						histR[R]++;
						histG[G]++;
						histB[B]++;
					}
				}

				int totalPixels = width * height;
				int avgR = sumR / totalPixels;
				int avgG = sumG / totalPixels;
				int avgB = sumB / totalPixels;

				// Piek (modus) zoeken
				int peakR = std::distance(histR.begin(), std::max_element(histR.begin(), histR.end()));
				int peakG = std::distance(histG.begin(), std::max_element(histG.begin(), histG.end()));
				int peakB = std::distance(histB.begin(), std::max_element(histB.begin(), histB.end()));

				int countR = histR[peakR];
				int countG = histG[peakG];
				int countB = histB[peakB];

				// percentages berekenen
				double percAtPeakR = 100.0 * countR / totalPixels;
				double percAtPeakG = 100.0 * countG / totalPixels;
				double percAtPeakB = 100.0 * countB / totalPixels;


				cout << "Average RGB = (" << avgR << ", " << avgG << ", " << avgB << ")" << std::endl;
				cout << "Minimum RGB = (" << (int)minR << ", " << (int)minG << ", " << (int)minB << ")" << std::endl;

				cout << "Percentage pixels bij piek: "
					<< "R=" << percAtPeakR << "%, "
					<< "G=" << percAtPeakG << "%, "
					<< "B=" << percAtPeakB << "%" << std::endl;

				int reg_address = 0;
				uint16_t values[10];

				values[0] = avgR;
				values[1] = avgG;
				values[2] = avgB;
				values[3] = minR;
				values[4] = minG;
				values[5] = minB;
				values[6] = (uint16_t)percAtPeakR;
				values[7] = (uint16_t)percAtPeakG;
				values[8] = (uint16_t)percAtPeakB;
				values[9] = height;

				int num_registers = 10;

				if (modbus_write_registers(ctx, reg_address, num_registers, values) == -1)
					std::cerr << "Schrijven mislukt: " << modbus_strerror(errno) << "\n";
			}
			else
			{
				cout << "<Abnormal Acquisition: Exception code: " << pImgData->GetStatus() << ">" << endl;
			}
		}
		catch (CGalaxyException& e)
		{
			cout << "<" << e.GetErrorCode() << ">" << "<" << e.what() << ">" << endl;
		}
		catch (std::exception& e)
		{
			cout << "<" << e.what() << ">" << endl;
		}
	}

	cout << "<Acquisition thread Exit!>" << endl;
	return 0;
}

// Auto-switch thread param
struct AUTOSWITCH_PARAM {
	bool bRun;
	CGXFeatureControlPointer pRemoteControl;
	CGXStreamPointer pStream;
	std::vector<ROIConfig>* pRoiConfigs;
	int intervalMs;
	int currentIndex;
};

// Auto-switch thread: cycles through ROI configs every intervalMs
DWORD WINAPI AutoSwitchROIThread(LPVOID lpParam)
{
	AUTOSWITCH_PARAM* p = (AUTOSWITCH_PARAM*)lpParam;
	if (p == NULL || p->pRoiConfigs == nullptr || p->pRoiConfigs->empty())
		return 0;

	int idx = p->currentIndex;

	while (p->bRun)
	{
		Sleep(p->intervalMs);

		if (!p->bRun) break;

		idx = (idx + 1) % (int)p->pRoiConfigs->size();

		EnterCriticalSection(&g_roiCs);
		try
		{
			// CAMERA VEILIG STOPPEN
			p->pRemoteControl->GetCommandFeature("AcquisitionStop")->Execute();
			p->pStream->StopGrab();

			// Nieuwe instellingen
			ROIConfig cfg = (*(p->pRoiConfigs))[idx];
			p->pRemoteControl->GetIntFeature("Width")->SetValue(cfg.width);
			p->pRemoteControl->GetIntFeature("Height")->SetValue(cfg.height);
			p->pRemoteControl->GetIntFeature("OffsetX")->SetValue(cfg.offsetX);
			p->pRemoteControl->GetIntFeature("OffsetY")->SetValue(cfg.offsetY);

			// CAMERA HERSTARTEN
			p->pStream->StartGrab();
			p->pRemoteControl->GetCommandFeature("AcquisitionStart")->Execute();

			cout << "[Auto] ROI ingesteld op " << cfg.width << "x" << cfg.height
				<< " offset(" << cfg.offsetX << ", " << cfg.offsetY << ")\n";
		}
		catch (CGalaxyException& e)
		{
			cout << "<AutoSwitch Exception: " << e.GetErrorCode() << " " << e.what() << ">\n";
		}
		catch (std::exception& e)
		{
			cout << "<AutoSwitch Exception: " << e.what() << ">\n";
		}
		LeaveCriticalSection(&g_roiCs);

		// update current index in param so main/manual switch can sync if needed
		p->currentIndex = idx;
	}

	return 0;
}

int main(int argc, char* argv[])
{

	try
	{
		//Initialize the device library
		IGXFactory::GetInstance().Init();

		//Enumerate camera devices
		GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
		IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);

		//Determine the number of current device connections
		if (vectorDeviceInfo.size() <= 0)
		{
			cout << "No device!" << endl;
			system("pause");
			return 0;
		}

		//Open the camera device via SN
		CGXDevicePointer pDevice = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
		//Get the camera property control object
		CGXFeatureControlPointer pRemoteControl = pDevice->GetRemoteFeatureControl();
		//Stream layer objects
		CGXStreamPointer pStream;
		if (pDevice->GetStreamCount() > 0)
		{
			pStream = pDevice->OpenStream(0);
		}
		else
		{
			cout << "Not find stream!";
			system("pause");
			return 0;
		}

		// ROI-configuraties
		std::vector<ROIConfig> roiConfigs = {
			{440, 322, 24, 26},
			{640, 480, 0, 0},
			{320, 58, 24, 26}
		};

		int configIndex = 0;

		// Start met de eerste configuratie
		ROIConfig current = roiConfigs[configIndex];
		pRemoteControl->GetIntFeature("Width")->SetValue(current.width);
		pRemoteControl->GetIntFeature("Height")->SetValue(current.height);
		pRemoteControl->GetIntFeature("OffsetX")->SetValue(current.offsetX);
		pRemoteControl->GetIntFeature("OffsetY")->SetValue(current.offsetY);

		// ====== CAMERA-INSTELLINGEN ======
		if (!pRemoteControl->IsImplemented("ChunkModeActive") || !pRemoteControl->IsWritable("ChunkModeActive"))
		{
			cout << "ChunkData not supported. Exiting.\n";
			return 0;
		}

		pRemoteControl->GetBoolFeature("ChunkModeActive")->SetValue(true);
		if (pRemoteControl->IsImplemented("ChunkSelector") && pRemoteControl->IsWritable("ChunkSelector"))
		{
			pRemoteControl->GetEnumFeature("ChunkSelector")->SetValue("FrameID");
			if (pRemoteControl->IsImplemented("ChunkEnable") && pRemoteControl->IsWritable("ChunkEnable"))
			{
				pRemoteControl->GetBoolFeature("ChunkEnable")->SetValue(true);
			}
		}

		cout << "\n***********************************************" << endl;
		cout << "<Vendor Name:   " << pDevice->GetDeviceInfo().GetVendorName() << ">" << endl;
		cout << "<Model Name:    " << pDevice->GetDeviceInfo().GetModelName() << ">" << endl;
		cout << "<Serial Number: " << pDevice->GetDeviceInfo().GetSN() << ">" << endl;
		cout << "***********************************************" << endl << endl;

		cout << "Druk op [A] + Enter om te starten, [X] + Enter om af te sluiten.\n";

		char startKey;
		cin >> startKey;
		if (startKey == 'x' || startKey == 'X') {
			cout << "<App exit!>\n";
			return 0;
		}

		// ===== Start stream =====
		pStream->StartGrab();
		pRemoteControl->GetCommandFeature("AcquisitionStart")->Execute();

		// Initialize critical section used to guard ROI switches
		InitializeCriticalSection(&g_roiCs);

		// ===== Start thread =====
		THREAD_PARAM threadParam;
		threadParam.bRun = true;
		threadParam.pStream = pStream;
		HANDLE hThread = CreateThread(NULL, 0, GrabImgThread, &threadParam, 0, NULL);
		if (hThread == NULL)
		{
			cerr << "Failed to create thread." << std::endl;
			DeleteCriticalSection(&g_roiCs);
			return 0;
		}

		// AUTOSWITCH INSTELLINGEN
		AUTOSWITCH_PARAM autoParam;
		autoParam.bRun = true;
		autoParam.pRemoteControl = pRemoteControl;
		autoParam.pStream = pStream;
		autoParam.pRoiConfigs = &roiConfigs;
		autoParam.intervalMs = 2000; 
		autoParam.currentIndex = configIndex;

		HANDLE hAutoThread = CreateThread(NULL, 0, AutoSwitchROIThread, &autoParam, 0, NULL);
		if (hAutoThread == NULL)
		{
			cerr << "Failed to create auto-switch thread." << std::endl;
			// continue without auto switching
			autoParam.bRun = false;
		}

		// ===== Interactieve loop =====
		bool running = true;
		while (running)
		{
			cout << "\nKies actie:\n"
				<< " 1 - ROI 1 (440x322)\n"
				<< " 2 - ROI 2 (640x480)\n"
				<< " 3 - ROI 3 (320x58)\n"
				<< " X - Stop\n> ";

			char key;
			cin >> key;

			switch (key)
			{
			case '1':
			case '2':
			case '3':
			{
				int newIndex = key - '1';
				if (newIndex >= 0 && newIndex < (int)roiConfigs.size())
				{
					cout << "ROI wijzigen...\n";

					EnterCriticalSection(&g_roiCs);
					// CAMERA VEILIG STOPPEN
					pRemoteControl->GetCommandFeature("AcquisitionStop")->Execute();
					pStream->StopGrab();

					// Nieuwe instellingen
					ROIConfig cfg = roiConfigs[newIndex];
					pRemoteControl->GetIntFeature("Width")->SetValue(cfg.width);
					pRemoteControl->GetIntFeature("Height")->SetValue(cfg.height);
					pRemoteControl->GetIntFeature("OffsetX")->SetValue(cfg.offsetX);
					pRemoteControl->GetIntFeature("OffsetY")->SetValue(cfg.offsetY);

					// CAMERA HERSTARTEN
					pStream->StartGrab();
					pRemoteControl->GetCommandFeature("AcquisitionStart")->Execute();

					// Keep index in sync with auto thread
					configIndex = newIndex;
					autoParam.currentIndex = newIndex;

					LeaveCriticalSection(&g_roiCs);

					cout << "ROI ingesteld op " << cfg.width << "x" << cfg.height
						<< " offset(" << cfg.offsetX << ", " << cfg.offsetY << ")\n";
				}
				break;
			}
			case 'x':
			case 'X':
				running = false;
				break;
			default:
				cout << "Ongeldige invoer.\n";
				break;
			}
		}

		// ===== Programma afsluiten =====
		// stop auto thread
		autoParam.bRun = false;
		if (hAutoThread) {
			WaitForSingleObject(hAutoThread, INFINITE);
			CloseHandle(hAutoThread);
		}

		// stop grab thread
		threadParam.bRun = false;
		WaitForSingleObject(hThread, INFINITE);
		CloseHandle(hThread);

		DeleteCriticalSection(&g_roiCs);

		pRemoteControl->GetCommandFeature("AcquisitionStop")->Execute();
		pStream->StopGrab();
		pStream->Close();
		pDevice->Close();
		IGXFactory::GetInstance().Uninit();

		cout << "<App exit!>\n";
		return 0;
	}
	catch (CGalaxyException& e)
	{
		cout << "<" << e.GetErrorCode() << ">" << "<" << e.what() << ">" << endl;
		system("pause");
		return 0;
	}
	catch (std::exception& e)
	{
		cout << "<" << e.what() << ">" << endl;
		system("pause");
		return 0;
	}
}