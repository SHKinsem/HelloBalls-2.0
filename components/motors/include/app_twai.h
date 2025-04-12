#pragma once

#ifndef APP_TWAI_H
#define APP_TWAI_H

#ifdef __cplusplus
extern "C" {
#endif


void install_twai_driver();
void uninstall_twai_driver();
void can_task(void);

#ifdef __cplusplus
}
#endif

#endif // APP_TWAI_H