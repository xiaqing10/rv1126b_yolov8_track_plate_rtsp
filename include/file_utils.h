#ifndef _RKNN_DEMO_FILE_UTILS_H_
#define _RKNN_DEMO_FILE_UTILS_H_

#include <stddef.h>

int  read_data_from_file(const char *path, char **out_data);
int  write_data_to_file(const char *path, const char *data, size_t size);

#endif
