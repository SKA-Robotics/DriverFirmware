#include "filter.h"

float ApplyIirFilter(iir_filter_t* filter, float y) {
    y = filter->coefficient * filter->prevY + (1 - filter->coefficient) * y;
    filter->prevY = y;
    return y;
}