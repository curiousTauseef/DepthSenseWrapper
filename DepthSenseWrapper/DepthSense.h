#ifndef DEPTHSENSE_H
#define DEPTHSENSE_H

#include <DepthSense.hxx>
#include <boost/thread.hpp>

namespace DepthSense 
{

	class DepthSenseImage 
	{
	public:
		unsigned char* image;
		float* depth;
		int width;
		int height;
	};

	class DepthSenseCapture 
	{
	public:
		DepthSenseCapture();
		~DepthSenseCapture();

		void grab();
		DepthSenseImage retrieve();

	private:
		boost::mutex mutex;
		unsigned char* tempImage;
		float* tempDepth;
		uint64_t depthTime;
		uint64_t imageTime;

		boost::thread* runningThread;
		void run();

		DepthSense::Context g_context;
		DepthSense::DepthNode g_dnode;
		DepthSense::ColorNode g_cnode;
		DepthSense::AudioNode g_anode;

		bool g_bDeviceFound;

		DepthSense::ProjectionHelper* g_pProjHelper;
		DepthSense::StereoCameraParameters g_scp;

		unsigned char* image;
		int width;
		int height;
		float* depth;
		int depthWidth;
		int depthHeight;

		// Device Handler
		void onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data);
		void onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data);

		// Node handler
		void onNodeConnected(DepthSense::Device device, DepthSense::Device::NodeAddedData data);
		void onNodeDisconnected(DepthSense::Device device, DepthSense::Device::NodeRemovedData data);
		void configureNode(DepthSense::Node node);
		void configureAudioNode();
		void configureDepthNode();
		void configureColorNode();

		// Samples handler
		void onNewAudioSample(DepthSense::AudioNode node, DepthSense::AudioNode::NewSampleReceivedData data);
		void onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);
		void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);
	};
}
#endif