/**
 * 
 */
#include "app_sentry_diagnosis_commom.hpp"

uint8_t OfflineListBytes[APP_SENTRY_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE] = {0};

void app_sentry_diag_SetDevice(uint8_t device_code, int offline_states)
{
    uint8_t dominator = (offline_states==0) ? 0x00 : 0x01 ;

    #if APP_SENTRY_DIAG_FLAG_IN_BIT
    OfflineListBytes[ (device_code/8) ] &= ( dominator << (device_code%8) );
    #endif // APP_SENTRY_DIAG_FLAG_IN_BIT
}
uint8_t app_sentry_diag_GetDevice(uint8_t device_code)
{
    #if APP_SENTRY_DIAG_FLAG_IN_BIT
    return 
    (
        OfflineListBytes[ (device_code/8) ] 
        & 
        ( 0x01 << (device_code%8) )
    );
    #endif // APP_SENTRY_DIAG_FLAG_IN_BIT}
}

