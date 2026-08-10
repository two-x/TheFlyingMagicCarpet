/* wifiauth.h
 *
 *   Shared WiFi AP credentials for the CarpetLightWifiESP radio bridge.
 *   Plaintext by design -- this password only needs to keep out people
 *   unfamiliar with the code during live operation, not resist a real
 *   attacker (confirmed with project owner). Committed to the repo so
 *   both CarpetLightWifiESP (which actually hosts the AP) and
 *   CarpetLightCode's visualizer build (which displays these values to
 *   the user for manual connection) always read the same source of
 *   truth -- edit here, not in either downstream copy.
 */

#ifndef __WIFIAUTH_H
#define __WIFIAUTH_H

#define WIFI_SSID     "CarpetLightBox"
#define WIFI_PASSWORD "flyaway"

#endif
