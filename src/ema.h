#ifndef EMA
#define EMA

constexpr float alpha{0.0};    // value between 0.0 to 1.0 inclusive
float lastOutput{0.0};

// EMA filter
// y[i] = alpha * x[i] + (1 - alpha) * y[i - 1]
float filterEMA(float input) {
    lastOutput = alpha * input + (1 - alpha) * lastOutput;
    return lastOutput;
}

#endif