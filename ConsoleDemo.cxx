////////////////////////////////////////////////////////////////////////////////
// SoftKinetic DepthSense SDK
//
// COPYRIGHT AND CONFIDENTIALITY NOTICE - SOFTKINETIC CONFIDENTIAL
// INFORMATION
//
// All rights reserved to SOFTKINETIC SENSORS NV (a
// company incorporated and existing under the laws of Belgium, with
// its principal place of business at Boulevard de la Plainelaan 11,
// 1050 Brussels (Belgium), registered with the Crossroads bank for
// enterprises under company number 0811 341 454 - "Softkinetic
// Sensors").
//
// The source code of the SoftKinetic DepthSense Camera Drivers is
// proprietary and confidential information of Softkinetic Sensors NV.
//
// For any question about terms and conditions, please contact:
// info@softkinetic.com Copyright (c) 2002-2015 Softkinetic Sensors NV
////////////////////////////////////////////////////////////////////////////////


#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <iostream>
#include <vector>
#include <exception>

#include <DepthSense.hxx>

using namespace DepthSense;
using namespace std;

//using namespace System::IO::port	

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

/*----------------------------------------------------------------------------*/
// New audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
	// printf("A#%u: %d\n",g_aFrames,data.audioData.size());
	g_aFrames++;
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
	//printf("C#%u: %d\n",g_cFrames,data.colorMap.size());
	g_cFrames++;
	
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
//	cerr << " x" << data.acceleration.x << " y" << data.acceleration.y << " z" << data.acceleration.z << endl;
	//    printf("Z#%u: %d\n",g_dFrames,data.vertices.size());

//		// Project some 3D points in the Color Frame
//	if (!g_pProjHelper)
//	{
//	//	g_pProjHelper = new ProjectionHelper(data.stereoCameraParameters);
//		g_scp = data.stereoCameraParameters;
//	}
//	else if (g_scp != data.stereoCameraParameters)
//	{
////		g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
//		g_scp = data.stereoCameraParameters;
//	}

// cx = 160 cy = 120 fx = 232.637 fy = 238.448 height = 240 k1 = -0.175591 k2 = 0.168395 k3 = -0.0281906
//	cerr << "cx=" << g_scp.depthIntrinsics.cx << " cy=" << g_scp.depthIntrinsics.cy << " fx=" << g_scp.depthIntrinsics.fx << " fy=" << g_scp.depthIntrinsics.fy << " height=" << g_scp.depthIntrinsics.height << " k1=" << g_scp.depthIntrinsics.k1 << " k2=" << g_scp.depthIntrinsics.k2 << " k3=" << g_scp.depthIntrinsics.k3 << endl;
	int w, h;
	FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);

	int cx = w / 2;
	int cy = h / 2;

	FPVertex p3DPoints[4];
	//cout << (cy - h / 4) * (w + cx - w / 4) << "== (" << cy << " - " << h << " / 4) * " << w << " + " << cx << " - " << w << " / 4" << endl;
	//cout << (cy - h / 4) * (w + cx + w / 4) << "== (" << cy << " - " << h << " / 4) * " << w << " + " << cx << " + " << w << " / 4" << endl;
	//cout << (cy + h / 4) * (w + cx + w / 4) << "== (" << cy << " + " << h << " / 4) * " << w << " + " << cx << " + " << w << " / 4" << endl;
	//cout << (cy + h / 4) * (w + cx - w / 4) << "== (" << cy << " + " << h << " / 4) * " << w << " + " << cx << " - " << w << " / 4" << endl;



	//p3DPoints[0] = data.verticesFloatingPoint[(cy - h / 4) * w + cx - w / 4];		//	19280 == (120 - 240 / 4) * 320 + 160 - 320 / 4     == 19200  +80		  //  h/4w + w/4
	//p3DPoints[1] = data.verticesFloatingPoint[(cy - h / 4) * w + cx + w / 4];		//	19440 == (120 - 240 / 4) * 320 + 160 + 320 / 4     == 19200 +240     //  h/4w +  5/4w
	//p3DPoints[2] = data.verticesFloatingPoint[(cy + h / 4) * w + cx + w / 4];		//	57840 == (120 + 240 / 4) * 320 + 160 + 320 / 4	   == 57600 +240    //  5/4hw + 5/4w
	//p3DPoints[3] = data.verticesFloatingPoint[(cy + h / 4) * w + cx - w / 4];		//	57680 == (120 + 240 / 4) * 320 + 160 - 320 / 4     == 57600 +80    //  5/4hw + w/4 
	cout << "#Frame=" << g_dFrames << endl;
	cout << "#camera rotation: x" << data.acceleration.x << ", y" << data.acceleration.z << ", z" << data.acceleration.y << endl;
	//	cerr << (2.f == 2) << endl;
	//cout << (cy - h / 4) * w + cx - w / 4 << endl;
	float coef = 1000.0f; // scaling coefficient
	for (int i = 0; i < 240 * 320; i++)
		if (data.verticesFloatingPoint[i].z != -1 && data.verticesFloatingPoint[i].z != -2)
			cout << "v " << data.verticesFloatingPoint[i].x * coef << " " << data.verticesFloatingPoint[i].y * coef << " " << data.verticesFloatingPoint[i].z * -coef << endl;

	//if (p3DPoints[0].z != 32001)  cout << "v " << p3DPoints[0].x << " " << p3DPoints[0].y << " " << p3DPoints[0].z << endl;
	//if (p3DPoints[1].z != 32001)  cout << "v " << p3DPoints[1].x << " " << p3DPoints[1].y << " " << p3DPoints[1].z << endl;
	//if (p3DPoints[2].z != 32001)  cout << "v " << p3DPoints[2].x << " " << p3DPoints[2].y << " " << p3DPoints[2].z << endl;
	//if (p3DPoints[3].z != 32001)  cout << "v " << p3DPoints[3].x << " " << p3DPoints[3].y << " " << p3DPoints[3].z << endl;
	//for (int i = 0; data.vertices; i++);
	//cout << data.verticesFloatingPoint.size << endl;

	//Point2D p2DPoints[4];
	//g_pProjHelper->get2DCoordinates(p3DPoints, p2DPoints, 4, CAMERA_PLANE_COLOR);
	g_dFrames++;


	//for (int i = 0; i < p2DPoints; i++) {cout << p3DPoints[i].z; }
	cout << endl;

	// Quit the main loop after 200 depth frames received
	if (g_dFrames == 1)
		g_context.quit();
}

/*----------------------------------------------------------------------------*/
void configureAudioNode()
{
	g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

	AudioNode::Configuration config = g_anode.getConfiguration();
	config.sampleRate = 44100;

	try
	{
		g_context.requestControl(g_anode, 0);

		g_anode.setConfiguration(config);

		g_anode.setInputMixerLevel(0.5f);
	}
	catch (ArgumentException& e)
	{
		//     printf("Argument Exception: %s\n",e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		//     printf("Unauthorized Access Exception: %s\n",e.what());
	}
	catch (ConfigurationException& e)
	{
		//     printf("Configuration Exception: %s\n",e.what());
	}
	catch (StreamingException& e)
	{
		//     printf("Streaming Exception: %s\n",e.what());
	}
	catch (TimeoutException&)
	{
		//      printf("TimeoutException\n");
	}
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
	g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

	DepthNode::Configuration config = g_dnode.getConfiguration();
	config.frameFormat = FRAME_FORMAT_QVGA;
	config.framerate = 30;
	config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
	config.saturation = true;

	//g_dnode.setEnableVertices(true);
	g_dnode.setEnableVerticesFloatingPoint(true);
	try
	{
		g_context.requestControl(g_dnode, 0);
		cerr << "imu fw version" << g_dnode.getImuFwVersion() << endl;
		g_dnode.setEnableAccelerometer(true);
		cerr << "accelerometer is enabled?: " << g_dnode.getEnableAccelerometer() << endl;

//		g_dnode.setIlluminationLevel(100);
		std::cerr << "illumination level: " << g_dnode.getIlluminationLevel() << std::endl;
				g_dnode.setEnableFilter1(true);
		g_dnode.setFilter1Parameter1(16384);
		g_dnode.setFilter1Parameter2(4000);
		g_dnode.setFilter1Parameter3(200);
		g_dnode.setFilter1Parameter4(2000);
		std::cerr << "filter 1 is enabled?: " << g_dnode.getEnableFilter1() << std::endl;

		g_dnode.setConfiguration(config);
	}
	catch (ArgumentException& e)
	{
		//      printf("Argument Exception: %s\n",e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		//    printf("Unauthorized Access Exception: %s\n",e.what());
	}
	catch (IOException& e)
	{
		//    printf("IO Exception: %s\n",e.what());
	}
	catch (InvalidOperationException& e)
	{
		//    printf("Invalid Operation Exception: %s\n",e.what());
	}
	catch (ConfigurationException& e)
	{
		//    printf("Configuration Exception: %s\n",e.what());
	}
	catch (StreamingException& e)
	{
		//     printf("Streaming Exception: %s\n",e.what());
	}
	catch (TimeoutException&)
	{
		//     printf("TimeoutException\n");
	}

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
	// connect new color sample handler
	g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

	ColorNode::Configuration config = g_cnode.getConfiguration();
	config.frameFormat = FRAME_FORMAT_VGA;
	config.compression = COMPRESSION_TYPE_MJPEG;
	config.powerLineFrequency = POWER_LINE_FREQUENCY_60HZ;
	config.framerate = 30;

	g_cnode.setEnableColorMap(true);

	try
	{
		g_context.requestControl(g_cnode, 0);

		g_cnode.setConfiguration(config);
	}
	catch (ArgumentException& e)
	{
		//      printf("Argument Exception: %s\n",e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		//     printf("Unauthorized Access Exception: %s\n",e.what());
	}
	catch (IOException& e)
	{
		//     printf("IO Exception: %s\n",e.what());
	}
	catch (InvalidOperationException& e)
	{
		//     printf("Invalid Operation Exception: %s\n",e.what());
	}
	catch (ConfigurationException& e)
	{
		//     printf("Configuration Exception: %s\n",e.what());
	}
	catch (StreamingException& e)
	{
		//    printf("Streaming Exception: %s\n",e.what());
	}
	catch (TimeoutException&)
	{
		//  printf("TimeoutException\n");
	}
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
	if ((node.is<DepthNode>()) && (!g_dnode.isSet()))
	{
		g_dnode = node.as<DepthNode>();
		configureDepthNode();
		g_context.registerNode(node);
	}

	if ((node.is<ColorNode>()) && (!g_cnode.isSet()))
	{
		g_cnode = node.as<ColorNode>();
		configureColorNode();
		g_context.registerNode(node);
	}

	if ((node.is<AudioNode>()) && (!g_anode.isSet()))
	{
		g_anode = node.as<AudioNode>();
		configureAudioNode();
		g_context.registerNode(node);
	}
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
	configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
	if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
		g_anode.unset();
	if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
		g_cnode.unset();
	if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
		g_dnode.unset();
	//    printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
	if (!g_bDeviceFound)
	{
		data.device.nodeAddedEvent().connect(&onNodeConnected);
		data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
		g_bDeviceFound = true;
	}
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
	g_bDeviceFound = false;
	// printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
	////////////////////////
	HANDLE serialHandle;

	serialHandle = CreateFile((LPCWSTR)"\\\\.\\COM5", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	// Do some basic settings
	DCB serialParams = { 0 };
	serialParams.DCBlength = sizeof(serialParams);
	int baudrate = CBR_115200;
	int byteSize = 8;
	int stopBits = TWOSTOPBITS;
	int parity = NOPARITY;
	GetCommState(serialHandle, &serialParams);
	serialParams.BaudRate = baudrate;
	serialParams.ByteSize = byteSize;
	serialParams.StopBits = stopBits;
	serialParams.Parity = parity;
	SetCommState(serialHandle, &serialParams);

	// Set timeouts
	COMMTIMEOUTS timeout = { 0 };
	timeout.ReadIntervalTimeout = 50;
	timeout.ReadTotalTimeoutConstant = 50;
	timeout.ReadTotalTimeoutMultiplier = 50;
	timeout.WriteTotalTimeoutConstant = 50;
	timeout.WriteTotalTimeoutMultiplier = 10;

	SetCommTimeouts(serialHandle, &timeout);
	//////////////////////// Now you can use WriteFile() / ReadFile() to write / read bytes. Don't forget to close your connection:
	char data[300];
//	int jj = 0;
//	data[jj] = 0;
	DWORD read;
	ReadFile(serialHandle,&data,300,&read,NULL);

	for (int jj = 0; jj < 300 ; jj++)
	{
		cerr << "jj=" << jj << endl;
	}
	//
	//  Don't forget to close your connection:
	CloseHandle(serialHandle);
	// 
	//

	g_context = Context::create("localhost");

	g_context.deviceAddedEvent().connect(&onDeviceConnected);
	g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

	// Get the list of currently connected devices
	vector<Device> da = g_context.getDevices();

	// We are only interested in the first device
	if (da.size() >= 1)
	{
		g_bDeviceFound = true;
		cerr << "The " << da[0].Model_toString(da[0].getModel())  << " camera supports: " << da[0].Capabilities_toString(da[0].getCapabilities()) << endl;
		da[0].nodeAddedEvent().connect(&onNodeConnected);
		da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

		vector<Node> na = da[0].getNodes();

		//      printf("Found %u nodes\n",na.size());

		for (int n = 0; n < (int)na.size(); n++)
			configureNode(na[n]);
	}

	g_context.startNodes();

	cout << "# Blender v2.92.0 OBJ File : ''\n# www.blender.org\nmtllib goobypls.mtl\no pointcloud" << endl; //header

	g_context.run();

	//	for (int i = 0; i < 240 * 320; i++) if (data.verticesFloatingPoint[i].z != 0) cout << "v" << data.verticesFloatingPoint[i].x << " " << data.verticesFloatingPoint[i].y << " " << data.verticesFloatingPoint[i].z << endl;

	g_context.stopNodes();

	if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
	if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
	if (g_anode.isSet()) g_context.unregisterNode(g_anode);

	if (g_pProjHelper)
		delete g_pProjHelper;

	return 0;
}
