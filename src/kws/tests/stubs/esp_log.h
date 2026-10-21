#pragma once
void test_log(const char *, ...);
#define ESP_LOGI(tag, ...) test_log(__VA_ARGS__)
