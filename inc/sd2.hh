#ifndef SD2_HH
#define SD2_HH

// boost
#include <boost/asio.hpp>
// protobuf
#include <track.pb.h>
#include <situation.pb.h>

class SD2 {
public:
	SD2();
	~SD2();
	protobuf::Track& getTrack();
	protobuf::Situation& getCurrentSituation();

private:
	// boost
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::socket s;
	// protobuf
	protobuf::Track track;
	protobuf::Situation currentSituation;

	void receivePacket(std::vector<uint8_t> &buffer);
	void receiveTrack();
	void receiveSituation();
};

#endif
