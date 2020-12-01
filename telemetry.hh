#pragma once

#include <memory>
#include <string>
#include <thread>
#include <map>

class Telemetry {
public:

  Telemetry() : m_recv_sock(0), m_status_recv_sock(0), m_sysid(0), m_compid(0),
                m_last_telemetry_packet_time(0), m_sender_valid(false), m_rec_bat_status(false),
                m_connected(false) {}

  bool start(const std::string &telemetry_host, uint16_t telemetry_port,
             const std::string &status_host, uint16_t status_port);

  bool get_value(const std::string &name, float &value) const;

  bool armed() const;
  void armed(bool val);

  bool connected() const;

private:
  typedef std::map<std::string, float> NVMap;

  void set_value(const std::string &name, float value);

  void reader_thread();
  void wfb_reader_thread();
  void control_thread();

  int m_recv_sock;
  int m_status_recv_sock;
  NVMap m_values;
  uint8_t m_sysid;
  uint8_t m_compid;
  double m_last_telemetry_packet_time;
  bool m_sender_valid;
  bool m_rec_bat_status;
  bool m_connected;
  std::shared_ptr<std::thread> m_receive_thread;
};
