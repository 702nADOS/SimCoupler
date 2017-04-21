#ifndef SAVM_HH
#define SAVM_HH

class SAVM {
public:
  SAVM(std::string address, int port);
  ~SAVM();

private:
  // boost
  boost::asio::io_service io_service;
  boost::asio::ip::tcp::socket s;
};

#endif
