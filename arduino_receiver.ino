/*
  arduino_receiver.ino
  Receives LiDAR scan data from the Raspberry Pi over UDP and prints it
  to the Serial Monitor.

  Wiring: Arduino Ethernet Shield (W5100/W5500) on top of the Arduino,
          plugged into the Cisco switch alongside the Pi.

  Packet layout (13 bytes, little-endian) — must match pi_to_arduino.py:
    [0:4]  az_rad   float   azimuth in radians
    [4:8]  el_rad   float   elevation in radians
    [8:12] range_m  float   range in metres
    [12]   detected byte    1 = valid return, 0 = no return within 90 m
*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ── Network config ─────────────────────────────────────────────────────────────
// MAC: use the sticker on the bottom of your Ethernet shield if it has one.
byte mac[]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x20 };
IPAddress arduino_ip(192, 168, 50, 20);   // must be unused on your subnet
IPAddress gateway(192, 168, 50, 1);
IPAddress subnet(255, 255, 255, 0);
const unsigned int LOCAL_PORT = 5005;     // must match ARDUINO_PORT in pi_to_arduino.py

// ── Packet ─────────────────────────────────────────────────────────────────────
#define PACKET_SIZE 13

EthernetUDP udp;
byte packet_buf[PACKET_SIZE];

// Safely reinterpret 4 bytes from a buffer as a float (avoids UB).
float bytes_to_float(byte* buf) {
  float val;
  memcpy(&val, buf, 4);
  return val;
}

void setup() {
  Serial.begin(115200);
  Ethernet.begin(mac, arduino_ip, gateway, gateway, subnet);
  udp.begin(LOCAL_PORT);
  Serial.println("[+] Arduino listening on 192.168.50.20:5005");
}

void loop() {
  int packet_len = udp.parsePacket();
  if (packet_len < PACKET_SIZE) return;   // nothing yet or truncated

  udp.read(packet_buf, PACKET_SIZE);

  float az_rad  = bytes_to_float(packet_buf);
  float el_rad  = bytes_to_float(packet_buf + 4);
  float range_m = bytes_to_float(packet_buf + 8);
  bool  detected = packet_buf[12];

  Serial.print("az=");      Serial.print(degrees(az_rad),  2);
  Serial.print("deg  el="); Serial.print(degrees(el_rad),  2);
  Serial.print("deg  range="); Serial.print(range_m, 3);
  Serial.print("m  detected="); Serial.println(detected ? "YES" : "NO");
}
