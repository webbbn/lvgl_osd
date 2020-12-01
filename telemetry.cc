
#include <sys/time.h>

#include <net/if.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <iostream>
#include <thread>
#include <deque>

#include <mavlink.h>

#include <telemetry.hh>
//#include <transfer_stats.hh>

inline double cur_time() {
  struct timeval t;
  gettimeofday(&t, 0);
  return double(t.tv_sec) + double(t.tv_usec) * 1e-6;
}

std::string hostname_to_ip(const std::string &hostname) {

  // Try to lookup the host.
  struct hostent *he;
  if ((he = gethostbyname(hostname.c_str())) == NULL) {
    fprintf(stderr, "Error: invalid hostname\n");
    return "";
  }

  struct in_addr **addr_list = (struct in_addr **)he->h_addr_list;
  for(int i = 0; addr_list[i] != NULL; i++) {
    //Return the first one;
    return inet_ntoa(*addr_list[i]);
  }

  return "";
}

int open_udp_socket_for_rx(uint16_t port, const std::string hostname) {

  // Try to open a UDP socket.
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    fprintf(stderr, "Error opening the UDP receive socket.\n");
    return -1;
  }

  // Set the socket options.
  int optval = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));
  setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (const void *)&optval, sizeof(optval));

  // Find to the receive port
  struct sockaddr_in saddr;
  bzero((char *)&saddr, sizeof(saddr));
  saddr.sin_family = AF_INET;
  saddr.sin_port = htons(port);

  // Lookup the IP address from the hostname
  std::string ip;
  if (hostname != "") {
    ip = hostname_to_ip(hostname);
    saddr.sin_addr.s_addr = inet_addr(ip.c_str());
  } else {
    saddr.sin_addr.s_addr = INADDR_ANY;
  }

  if (bind(fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
    fprintf(stderr, "Error binding to the UDP receive socket: %d\n", port);
    return -1;
  }

  return fd;
}

class SlidingSum {
public:

  SlidingSum(double time_window) : m_win(time_window), m_sum(0) { }

  double add(double time, double value) {
    // Insert the current time/value at the end of the queue and add it to the sum
    m_q.push_back(std::make_pair(time, value));
    m_sum += value;
    return adjust(time);
  }

private:

  double adjust(double time) {

    // Remove elements from the front of the queue that have fallen out of the time window
    while (m_q.size() > 1) {
      if ((time - m_q.front().first) > m_win) {
        m_sum -= m_q.front().second;
        m_q.pop_front();
      } else {
        break;
      }
    }

    // Return the current sum.
    return m_sum;
  }

  std::deque<std::pair<double, double> > m_q;
  double m_win;
  double m_sum;

};

Telemetry::Telemetry(const std::string &telemetry_host, uint16_t telemetry_port,
                     const std::string &status_host, uint16_t status_port) :
  m_sysid(0), m_compid(0), m_rec_bat_status(false), m_connected(false) {
  m_recv_sock = open_udp_socket_for_rx(telemetry_port, telemetry_host);
  m_status_recv_sock = open_udp_socket_for_rx(status_port, status_host);
  if ((m_recv_sock < 0) || (m_status_recv_sock < 0)) {
    fprintf(stderr, "Error binding to telemetry sockets\n");
  } else {
    printf("Opened telemetry port: %s:%d and status port %s:%d\n",
            telemetry_host.c_str(), telemetry_port, status_host.c_str(), status_port);
  }
  //std::thread([this]() { this->reader_thread(); }).detach();
  //std::thread([this]() { this->wfb_reader_thread(); }).detach();
  m_receive_thread.reset(new std::thread([this]() { this->reader_thread(); }));
}

bool Telemetry::get_value(const std::string &name, float &value) const {
  NVMap::const_iterator mi = m_values.find(name);
  if (mi == m_values.end()) {
    return false;
  }
  value = mi->second;
  return true;
}

void Telemetry::set_value(const std::string &name, float value) {
  m_values[name] = value;
}

bool Telemetry::connected() const {
  return m_connected;
}

void Telemetry::reader_thread() {
  mavlink_message_t msg;
  mavlink_status_t status;
  int max_length = 1024;
  uint8_t data[max_length];
  bool messages_requested = false;

  printf("reader_thread running\b");
  fflush(stdout);
  while(1) {
    ssize_t length = recv(m_recv_sock, data, max_length, 0);
    printf("received message with length = %ld\n", length);
    fflush(stdout);

    if (!m_connected) {
      //set_value("ip_address", m_sender_endpoint.address().to_string());
      m_connected = true;
    }

    bool heartbeat_received = false;
    for (size_t i = 0; i < length; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
        printf("Received msgid: %d from %d:%d\n", msg.msgid, msg.sysid, msg.compid);
        fflush(stdout);
	m_sysid = msg.sysid;
	m_compid = msg.compid;
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_POWER_STATUS:
	  break;
	case MAVLINK_MSG_ID_SYS_STATUS:
	  mavlink_sys_status_t sys_status;
	  mavlink_msg_sys_status_decode(&msg, &sys_status);
	  if (!m_rec_bat_status) {
	    set_value("voltage_battery", sys_status.voltage_battery / 1000.0);
	    set_value("current_battery",
		      std::max(sys_status.current_battery, static_cast<short>(0)) / 100.0);
	    set_value("battery_remaining", sys_status.battery_remaining);
	  }
	  break;
	case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
	  mavlink_nav_controller_output_t nav;
	  mavlink_msg_nav_controller_output_decode(&msg, &nav);
	  break;
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
	  mavlink_global_position_int_t pos;
	  mavlink_msg_global_position_int_decode(&msg, &pos);
	  set_value("latitude", static_cast<float>(pos.lat) * 1e-7);
	  set_value("longitude", static_cast<float>(pos.lon) * 1e-7);
	  set_value("altitude", static_cast<float>(pos.alt) / 1000.0);
	  set_value("relative_altitude", static_cast<float>(pos.relative_alt) / 1000.0);
	  set_value("speed", sqrt(pos.vx * pos.vx + pos.vy * pos.vy + pos.vz * pos.vz) / 100.0);
          // iNav is raw degrees (no scaling).
	  set_value("heading", static_cast<float>(pos.hdg));
	  //set_value("heading", static_cast<float>(pos.hdg) / 100.0);
	  break;
	case MAVLINK_MSG_ID_ATTITUDE:
	  mavlink_attitude_t att;
	  mavlink_msg_attitude_decode(&msg, &att);
	  set_value("roll", att.roll);
	  set_value("pitch", att.pitch);
	  set_value("yaw", att.yaw);
	  break;
	case MAVLINK_MSG_ID_STATUSTEXT:
	  mavlink_statustext_t status;
	  mavlink_msg_statustext_decode(&msg, &status);
	  break;
	case MAVLINK_MSG_ID_MISSION_CURRENT:
	  //std::cerr << "Mission current " << std::endl;
	  break;
	case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
	  //std::cerr << "Servo raw " << std::endl;
	  break;
	case MAVLINK_MSG_ID_RC_CHANNELS:
	  break;
	case MAVLINK_MSG_ID_PARAM_VALUE:
	  //std::cerr << "Param value " << std::endl;
	  break;
	case MAVLINK_MSG_ID_VIBRATION:
	  //std::cerr << "Vibration " << std::endl;
	  break;
	case MAVLINK_MSG_ID_HEARTBEAT: {
	  mavlink_heartbeat_t hb;
	  mavlink_msg_heartbeat_decode(&msg, &hb);
	  bool is_armed = (hb.base_mode & 0x80);
	  set_value("armed", is_armed ? 1.0 : 0.0);
	  set_value("mode", static_cast<float>(hb.custom_mode));
	  heartbeat_received = true;
	  break;
	}
	case MAVLINK_MSG_ID_VFR_HUD:
	  //std::cerr << "VFR HUD " << std::endl;
	  break;
	case MAVLINK_MSG_ID_RAW_IMU:
	  //std::cerr << "Raw IMU " << std::endl;
	  break;
	case MAVLINK_MSG_ID_SCALED_PRESSURE:
	  //std::cerr << "Scaled Pressure " << std::endl;
	  break;
	case MAVLINK_MSG_ID_GPS_RAW_INT:
	  mavlink_gps_raw_int_t rawgps;
	  mavlink_msg_gps_raw_int_decode(&msg, &rawgps);
	  set_value("gps_fix_type", rawgps.fix_type);
	  set_value("gps_HDOP", rawgps.eph / 100.0);
	  set_value("gps_VDOP", rawgps.epv / 100.0);
          if (rawgps.vel != UINT16_MAX) {
            set_value("gps_velosity", rawgps.vel / 100.0);
          }
          if (rawgps.cog != UINT16_MAX) {
            set_value("gps_ground_course", rawgps.cog * 100.0);
          }
          if (rawgps.satellites_visible != 255) {
            set_value("gps_num_sats", rawgps.satellites_visible);
          }
	  //std::cerr << "GSP Raw " << std::endl;
	  break;
	case MAVLINK_MSG_ID_SYSTEM_TIME:
	  //std::cerr << "System Time " << std::endl;
	  break;
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
	  //std::cerr << "Local position " << std::endl;
	  break;
	case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
	  //std::cerr << "Autopilot version " << std::endl;
	  break;
	case MAVLINK_MSG_ID_COMMAND_ACK:
	  //std::cerr << "Command ACK " << std::endl;
	  break;
	case MAVLINK_MSG_ID_BATTERY_STATUS:
	  mavlink_battery_status_t bat;
	  mavlink_msg_battery_status_decode(&msg, &bat);
/*
	  if (bat.voltages[0] != INT16_MAX) {
	    set_value("voltage_battery", bat.voltages[0] / 1000.0);
	    set_value("current_battery",
		      std::max(bat.current_battery, static_cast<short>(0)) / 100.0);
	    set_value("battery_remaining", bat.battery_remaining);
	    m_rec_bat_status = true;
	  }
*/
	  break;
	case MAVLINK_MSG_ID_HOME_POSITION:
	  mavlink_home_position_t home;
	  mavlink_msg_home_position_decode(&msg, &home);
	  set_value("home_latitude", home.latitude * 1e-7);
	  set_value("home_longitude", home.longitude * 1e-7);
	  set_value("home_altitude", home.altitude);
	  break;
	case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
          {
            mavlink_rc_channels_raw_t rc;
            mavlink_msg_rc_channels_raw_decode(&msg, &rc);
            set_value("chan1", rc.chan1_raw);
            set_value("chan2", rc.chan2_raw);
            set_value("chan3", rc.chan3_raw);
            set_value("chan4", rc.chan4_raw);
            set_value("chan5", rc.chan5_raw);
            set_value("chan6", rc.chan6_raw);
            set_value("chan7", rc.chan7_raw);
            set_value("chan8", rc.chan8_raw);
            set_value("rc_rssi", 70.0 * static_cast<float>(rc.rssi) / 255.0 - 90.0);
          }
	  break;
	case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
          {
            mavlink_gps_global_origin_t origin;
            mavlink_msg_gps_global_origin_decode(&msg, &origin);
            set_value("home_latitude", origin.latitude * 1e-7);
            set_value("home_longitude", origin.longitude * 1e-7);
            set_value("home_altitude", origin.altitude);
            // Calculate the home direction.
            float gps_lat = 0;
            float gps_lon = 0;
            get_value("latitude", gps_lat);
            get_value("longitude", gps_lon);
            if ((gps_lat != 0) || (gps_lon != 0)) {
              gps_lat *= M_PI / 180.0;
              gps_lon *= M_PI / 180.0;
              double hlat = origin.latitude * M_PI / 180.0;
              double hlon = origin.latitude * M_PI / 180.0;
              double dlon = hlon - gps_lon;
              double x = sin(dlon) * sin(hlat);
              double y = cos(gps_lat) * sin(hlat) - sin(gps_lat) * cos(hlat) * cos(dlon);
              double hdir = atan2(y, x) * 180.0 / M_PI;
              set_value("home_direction", hdir);
            }
          }
          break;
	default:
	  std::cerr << "Received packet: SYS: " << int(msg.sysid)
		    << ", COMP: " << int(msg.compid)
		    << ", LEN: " << int(msg.len)
		    << ", MSG ID: " << msg.msgid << std::endl;
	  break;
	}
      }
    }

    if (heartbeat_received && !messages_requested) {
      const uint8_t MAVStreams[] = {
				    MAV_DATA_STREAM_RAW_SENSORS,
				    MAV_DATA_STREAM_EXTENDED_STATUS,
				    MAV_DATA_STREAM_RC_CHANNELS,
				    MAV_DATA_STREAM_POSITION,
				    MAV_DATA_STREAM_EXTRA1,
				    MAV_DATA_STREAM_EXTRA2,
				    MAVLINK_MSG_ID_ATTITUDE,
				    MAVLINK_MSG_ID_RADIO_STATUS
      };
      const uint16_t MAVRates[] = { 2, 5, 2, 5, 2, 2, 20, 2 };
      uint8_t data[max_length];
      for (size_t i = 0; i < sizeof(MAVStreams); ++i) {
/*
	int len = mavlink_msg_request_data_stream_pack(m_sysid, m_compid,
						       reinterpret_cast<mavlink_message_t*>(data),
						       1, 1, MAVStreams[i], MAVRates[i], 1);
	m_send_sock.send_to(boost::asio::buffer(data, len), m_sender_endpoint);
*/
	int len = mavlink_msg_message_interval_pack(m_sysid, m_compid,
						    reinterpret_cast<mavlink_message_t*>(data),
						    MAVStreams[i], 1000000 / MAVRates[i]);
	//m_recv_sock.send_to(boost::asio::buffer(data, len), m_sender_endpoint);
      }
      messages_requested = true;
    }

  }
}

void Telemetry::wfb_reader_thread() {
#if 0
  double last_recv = 0;
  double last_send = 0;
  // We want to display a time window of the last 10 seconds, but we want
  // to update it every second, so we'll keep a sliding window of 10 snapshots of
  // the stats, one for each of the last 10 seconds.
  std::deque<std::pair<double, wifibroadcast_rx_status_forward_t> > prev_stats;
  wifibroadcast_rx_status_forward_t prev_link_stats;
  double rx_quality = 0;
  float link_timeout = 5.0;
  boost::asio::ip::udp::endpoint sender_endpoint;

  while(1) {

    // Receive the next link status message
    wifibroadcast_rx_status_forward_t link_stats;
    size_t len = m_status_recv_sock.receive_from
      (boost::asio::buffer(&link_stats, sizeof(link_stats)), sender_endpoint);
    if (len != sizeof(link_stats)) {
      continue;
    }

    // Insert a new stats message into the queue if the second has rolled over.
    double time = cur_time();
    if ((prev_stats.size() == 0) || (trunc(time) > trunc(prev_stats.back().first))) {

      // Insert this stats message on the queue
      prev_stats.push_back(std::make_pair(time, link_stats));

      // Remove the head element(s) if it is / they are out of the time window.
      while (!prev_stats.empty() && ((time - prev_stats.front().first) > 10.0)) {
        prev_stats.pop_front();
      }

      // Calculate the stats and update the telemetry values.
      uint32_t total_packets = 0;
      uint32_t dropped_packets = 0;
      uint32_t bad_blocks = 0;
      if (prev_stats.size() > 0) {
        const auto &head = prev_stats.front().second;
        total_packets = link_stats.received_packet_cnt - head.received_packet_cnt;
        dropped_packets = link_stats.lost_packet_cnt - head.lost_packet_cnt;
        bad_blocks = link_stats.damaged_block_cnt - head.damaged_block_cnt;
      }
      set_value("rx_video_rssi", link_stats.adapter[0].current_signal_dbm);
      set_value("rx_video_packet_count", total_packets);
      set_value("rx_video_dropped_packets", dropped_packets);
      set_value("rx_video_bad_blocks", bad_blocks);
      set_value("rx_video_quality",
                (total_packets == 0) ? 100.0 :
                std::max(100.0 - 10.0 * dropped_packets / total_packets, 0.0));
      set_value("rx_video_bitrate", 1000.0 * link_stats.kbitrate);
    }

    prev_link_stats = link_stats;
  }
#endif
}
