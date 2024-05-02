class NotchFilter {
private:
    float xn_1 = 0; // Input delay element (n-1)
    float xn_2 = 0; // Input delay element (n-2)
    float yn_1 = 0; // Output delay element (n-1)
    float yn_2 = 0; // Output delay element (n-2)

    // Filter coefficients
    float b0, b1, b2; // Numerator coefficients
    float a1, a2;     // Denominator coefficients

public:
    NotchFilter(float centerFreq, float sampleRate, float bandwidth) {
        float w0 = 2 * PI * centerFreq / sampleRate;
        float alpha = sin(w0) * sinh(log(2) / 2 * bandwidth * w0 / sin(w0));

        // Calculate filter coefficients (Second-order notch filter)
        b0 = 1;
        b1 = -2 * cos(w0);
        b2 = 1;
        a1 = -2 * cos(w0);
        a2 = 1 + alpha;

        // Normalize coefficients
        b0 /= a2;
        b1 /= a2;
        b2 /= a2;
        a1 /= a2;
        a2 /= a2;
    }

    float filter(float input) {
        // Apply the notch filter
        float output = b0 * input + b1 * xn_1 + b2 * xn_2 - a1 * yn_1 - a2 * yn_2;

        // Update delay elements
        xn_2 = xn_1;
        xn_1 = input;
        yn_2 = yn_1;
        yn_1 = output;

        return output;
    }
};
