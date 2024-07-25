#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H


#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        printf("Error in function '%s' on line %d: %d (%s). Aborting.\n", #fn, __LINE__, (int)temp_rc, rcl_get_error_string().str); \
        rcl_reset_error(); \
        vTaskDelete(NULL); \
    } \
}
#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        printf("Error in function '%s' on line %d: %d (%s). Continuing.\n", #fn, __LINE__, (int)temp_rc, rcl_get_error_string().str); \
        rcl_reset_error(); \
    } \
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  CHECK_AGENT
};

extern states state;

#endif // COMMON_DEFS_H