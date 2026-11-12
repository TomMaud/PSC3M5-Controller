#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <stdbool.h>
#include "htmlfile.h"

bool webpage_start_http_servers(void);

void webpage_stop_http_servers(void);
#endif
