#include <sd2.hh>

#define SD2_PORT 9000

SD2::SD2() : s(io_service) {
	boost::asio::ip::tcp::endpoint ep(boost::asio::ip::tcp::v4(), SD2_PORT);
	boost::asio::ip::tcp::acceptor ac(io_service, ep);
	ac.accept(s);
	receiveTrack();
};

SD2::~SD2() {
	// TODO correctly destr...close boost stuff
};

// get and set methods

protobuf::Track& SD2::getTrack() {
	return track;
};

protobuf::Situation& getCurrentSituation() {
	return currentSituation;
};

/*
 * Receive a Packet from SD2 (4 byte message length + message)
 */
void SD2::receivePacket(std::vector<uint8_t> &buffer) {
	uint32_t length = 0, bread = 0;

	// get message length
	boost::asio::read(s, boost::asio::buffer(&length, 4));
	length = ntohl(length);

	// receive data
	buffer.resize(length);
	bread = boost::asio::read(s, boost::asio::buffer(buffer, length));
	if (bread != length); // TODO error receiveing packet
};

/*
 * Receive and parse track message
 */
void SD2::receiveTrack() {
	std::vector<uint8_t> message;
	receivePacket(message);

	if (track.ParseFromArray(message.data(), message.size()));
	else ; // TODO error parsing packet
};

/*
 * Receive and parse situation message
 */
void SD2::receiveSituation() {
	std::vector<uint8_t> message;
	receivePacket(message);

	if (currentSituation.ParseFromArray(message.data(), message.size()));
	else ; // TODO error parsing
};
