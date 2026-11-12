#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <stdbool.h>
#include "htmlfile.h"

bool webpage_start_http_servers(void);
bool handle_request(const char *request,
                    char *response,
                    size_t response_size);

#endif
