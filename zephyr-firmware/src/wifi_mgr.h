/* wifi_mgr.h - simple blocking WiFi connect helper */
#ifndef WIFI_MGR_H
#define WIFI_MGR_H
/**
 * @brief  Connect to the configured AP.  Blocks until associated or timeout.
 * @return 0 on success, negative errno on failure.
 */
int wifi_mgr_connect(void);
#endif
