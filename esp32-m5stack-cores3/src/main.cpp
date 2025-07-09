#include "main.h"

#ifndef LINUX_BUILD
extern "C" void app_main(void) {
}
#else
int main(void) {
}
#endif
