#include "file_utils.h"
#include <stdio.h>
#include <stdlib.h>

int read_data_from_file(const char *path, char **out_data)
{
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        printf("fopen %s fail!\n", path);
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char *data = (char *)malloc(size);
    if (data == NULL) {
        fclose(fp);
        return -1;
    }
    if (fread(data, 1, size, fp) != (size_t)size) {
        free(data);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    *out_data = data;
    return size;
}

int write_data_to_file(const char *path, const char *data, size_t size)
{
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        printf("fopen %s fail!\n", path);
        return -1;
    }
    size_t n = fwrite(data, 1, size, fp);
    fclose(fp);
    return (n == size) ? 0 : -1;
}
