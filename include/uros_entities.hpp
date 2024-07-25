#ifndef UROS_ENTITIES_HPP
#define UROS_ENTITIES_HPP

#include <rclc/rclc.h>
#include <rclc/executor.h>

extern rclc_executor_t executor;

void service_callback(void * request, void * response);

bool create_entities();
void destroy_entities(const char* error_message);

#endif // UROS_ENTITIES_HPP
