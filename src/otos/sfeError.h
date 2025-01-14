#pragma once

#include <stdint.h>

// an error 0=okay -1=failure >0=informative error
// ^ is a lie, >0=number of bytes read/written

typedef int32_t sfeError_t;

const sfeError_t kErrFail = -1;
const sfeError_t kErrOkay = 0;