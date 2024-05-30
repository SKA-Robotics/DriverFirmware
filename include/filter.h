#ifndef FILTER_H
#define FILTER_H

typedef struct {
    float coefficient;
    float prevY;
} iir_filter_t;

float ApplyIirFilter(iir_filter_t* filter, float y);

#endif // FILTER_H