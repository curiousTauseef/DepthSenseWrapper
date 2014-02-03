#include <vector>

#include "DepthSense.h"
namespace DepthSense {

DepthSenseCapture::DepthSenseCapture()
{
	g_bDeviceFound = false;
	g_pProjHelper = 0;
	width = 0;
	height = 0;
	depthWidth = 0;
	depthHeight = 0;

	g_context = DepthSense::Context::create("localhost");

	g_context.deviceAddedEvent().connect(this, &DepthSenseCapture::onDeviceConnected);
	g_context.deviceRemovedEvent().connect(this, &DepthSenseCapture::onDeviceDisconnected);

	// Get the list of currently connected devices
	std::vector<DepthSense::Device> da = g_context.getDevices();

	// We are only interested in the first device
	if (da.size() >= 1) {
		g_bDeviceFound = true;

		da[0].nodeAddedEvent().connect(this, &DepthSenseCapture::onNodeConnected);
		da[0].nodeRemovedEvent().connect(this, &DepthSenseCapture::onNodeDisconnected);

		std::vector<DepthSense::Node> na = da[0].getNodes();
        
		for (int n = 0; n < (int)na.size();n++) {
			configureNode(na[n]);
		}
	}

	image = new unsigned char[width*height*3];
	tempImage = new unsigned char[width*height*3];

	depth = new float[width*height];
	tempDepth = new float[width*height];

	runningThread = new boost::thread(&DepthSenseCapture::run, this);
}


DepthSenseCapture::~DepthSenseCapture()
{
	delete image;
	delete tempImage;
	delete depth;
	delete tempDepth;

	g_context.quit();
	runningThread->try_join_for(boost::chrono::milliseconds(200));
	delete runningThread;
	g_context.stopNodes();

	if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
	if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
	if (g_anode.isSet()) g_context.unregisterNode(g_anode);

	if (g_pProjHelper != 0)
		delete g_pProjHelper;
}

void DepthSenseCapture::run() 
{ 
	try {
		g_context.startNodes(); 
		g_context.run(); 
	}
	catch (DepthSense::Exception& e) {
		// TODO: Handle exceptions
	}
}

void DepthSenseCapture::grab()
{
	mutex.lock();
	const unsigned char* tmpImg = tempImage;
	std::copy(tmpImg, tmpImg + height*width*3, image);
	std::copy(tempDepth, tempDepth+height*width, depth);
	mutex.unlock();
}

DepthSenseImage DepthSenseCapture::retrieve()
{
	DepthSenseImage i;
	i.depth = depth;
	i.image = image;
	i.height = height;
	i.width = width;

	return i;
}

void DepthSenseCapture::onNewAudioSample(DepthSense::AudioNode node, DepthSense::AudioNode::NewSampleReceivedData data)
{
}

void DepthSenseCapture::onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
{
	if (mutex.try_lock()) {
		const unsigned char* tmpImg = (const unsigned char*) data.colorMap;
		std::copy(tmpImg, tmpImg + data.colorMap.size(), tempImage);
		imageTime = data.timeOfCapture;
		mutex.unlock();
	}
}

void DepthSenseCapture::onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
	if (g_pProjHelper == 0) {
		g_pProjHelper = new DepthSense::ProjectionHelper(data.stereoCameraParameters);
		g_scp = data.stereoCameraParameters;
	}
	else if (g_scp != data.stereoCameraParameters) {
		g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
		g_scp = data.stereoCameraParameters;
	}

	if (mutex.try_lock()) {
		depthTime = data.timeOfCapture;
		std::fill(tempDepth, tempDepth+width*height, 0);
		for (int y=0; y<depthHeight; y++) {
			for (int x=0; x<depthWidth; x++) {
				DepthSense::FPVertex p = data.verticesFloatingPoint[y * depthWidth + x];

				DepthSense::Point2D pos;
				g_pProjHelper->get2DCoordinates(&p, &pos, 1, DepthSense::CAMERA_PLANE_COLOR);
				if (pos.x >= 0 && pos.x < width && pos.y >= 0 && pos.y < height) {
					tempDepth[(int)pos.y * width + (int)pos.x] = p.z;
				}
			}
		}
		mutex.unlock();
	}
}

void DepthSenseCapture::configureAudioNode()
{
	g_anode.newSampleReceivedEvent().connect(this, &DepthSenseCapture::onNewAudioSample);

	DepthSense::AudioNode::Configuration config = g_anode.getConfiguration();
	config.sampleRate = 44100;

	try {
		g_context.requestControl(g_anode,0);
		g_anode.setConfiguration(config);
		g_anode.setInputMixerLevel(0.5f);
	}
	catch (DepthSense::Exception& e) {
		//TODO: Handle exceptions
	}
}

void DepthSenseCapture::configureDepthNode()
{
	g_dnode.newSampleReceivedEvent().connect(this, &DepthSenseCapture::onNewDepthSample);

	DepthSense::DepthNode::Configuration config = g_dnode.getConfiguration();
	config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
	config.framerate = 25;
	config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
	config.saturation = true;

	g_dnode.setEnableVerticesFloatingPoint(true);

	try {
		g_context.requestControl(g_dnode, 0);
		g_dnode.setConfiguration(config);

		FrameFormat_toResolution(config.frameFormat, &depthWidth, &depthHeight);
	}
	catch (DepthSense::Exception& e) {
		// TODO: Handle exceptions
	}
}

void DepthSenseCapture::configureColorNode()
{
	// connect new color sample handler
	g_cnode.newSampleReceivedEvent().connect(this, &DepthSenseCapture::onNewColorSample);

	DepthSense::ColorNode::Configuration config = g_cnode.getConfiguration();
	config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
	config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
	config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
	config.framerate = 25;

	g_cnode.setEnableColorMap(true);

	try  {
		g_context.requestControl(g_cnode, 0);
		g_cnode.setConfiguration(config);

		FrameFormat_toResolution(config.frameFormat, &width, &height);
	}
	catch (DepthSense::Exception& e) {
		// TODO: handle exceptions
	}
}

void DepthSenseCapture::configureNode(DepthSense::Node node)
{
	if ((node.is<DepthSense::DepthNode>())&&(!g_dnode.isSet())) {
		g_dnode = node.as<DepthSense::DepthNode>();
		configureDepthNode();
		g_context.registerNode(node);
	}

	if ((node.is<DepthSense::ColorNode>())&&(!g_cnode.isSet())) {
		g_cnode = node.as<DepthSense::ColorNode>();
		configureColorNode();
		g_context.registerNode(node);
	}

	// Audio Node not wrapped yet
	/*if ((node.is<DepthSense::AudioNode>())&&(!g_anode.isSet())) {
		g_anode = node.as<DepthSense::AudioNode>();
		configureAudioNode();
		g_context.registerNode(node);
	}*/
}

void DepthSenseCapture::onNodeConnected(DepthSense::Device device, DepthSense::Device::NodeAddedData data)
{
	configureNode(data.node);
}

void DepthSenseCapture::onNodeDisconnected(DepthSense::Device device, DepthSense::Device::NodeRemovedData data)
{
	if (data.node.is<DepthSense::AudioNode>() && 
		(data.node.as<DepthSense::AudioNode>() == g_anode)) {
		g_anode.unset();
	}
	if (data.node.is<DepthSense::ColorNode>() && 
		(data.node.as<DepthSense::ColorNode>() == g_cnode)) {
		g_cnode.unset();
	}
	if (data.node.is<DepthSense::DepthNode>() && 
		(data.node.as<DepthSense::DepthNode>() == g_dnode)) {
		g_dnode.unset();
	}
}

void DepthSenseCapture::onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data)
{
	if (!g_bDeviceFound) {
		data.device.nodeAddedEvent().connect(this, &DepthSenseCapture::onNodeConnected);
		data.device.nodeRemovedEvent().connect(this, &DepthSenseCapture::onNodeDisconnected);
		g_bDeviceFound = true;
	}
}

void DepthSenseCapture::onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data)
{
	g_bDeviceFound = false;
}

}