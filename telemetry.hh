#pragma once

#include <memory>
#include <string>
#include <thread>
#include <map>

class Telemetry {
public:

  Telemetry(const std::string &telemetry_host = "0.0.0.0", uint16_t telemetry_port = 14550,
            const std::string &status_host = "0.0.0.0", uint16_t status_port = 5800);

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
