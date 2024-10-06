/* void sendResultsOverUART(float32_t freqA, float32_t freqB, float32_t freqC, float32_t* phaseAngles, const char* phaseSequence)
{
    char msg[200];

    // Send frequencies
    snprintf(msg, sizeof(msg), "Phase A Frequency: %.2f Hz\r\n", freqA);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "Phase B Frequency: %.2f Hz\r\n", freqB);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "Phase C Frequency: %.2f Hz\r\n", freqC);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Send phase angles
    snprintf(msg, sizeof(msg), "Phase Angle AB: %.2f degrees\r\n", phaseAngles[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "Phase Angle AC: %.2f degrees\r\n", phaseAngles[1]);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Send phase sequence
    snprintf(msg, sizeof(msg), "Phase Sequence: %s\r\n", phaseSequence);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void calculatePhaseAnglesAndSequence(float32_t* zeroCrossingTimes, float32_t* phaseAngles, char* phaseSequence)
{
    // Calculate time differences between zero crossings
    // zeroCrossingTimes[0]: Phase A
    // zeroCrossingTimes[1]: Phase B
    // zeroCrossingTimes[2]: Phase C

    float32_t deltaAB = zeroCrossingTimes[1] - zeroCrossingTimes[0];
    float32_t deltaAC = zeroCrossingTimes[2] - zeroCrossingTimes[0];

    // Adjust for wrap-around
    float32_t samplesPerCycle = (float32_t)FS / 50.0f; // Approximate samples per cycle
    if (deltaAB < 0) deltaAB += samplesPerCycle;
    if (deltaAC < 0) deltaAC += samplesPerCycle;

    // Convert time differences to phase angles
    phaseAngles[0] = (deltaAB / samplesPerCycle) * 360.0f; // Phase angle between A and B
    phaseAngles[1] = (deltaAC / samplesPerCycle) * 360.0f; // Phase angle between A and C

    // Normalize angles to [0, 360)
    if (phaseAngles[0] < 0) phaseAngles[0] += 360.0f;
    if (phaseAngles[1] < 0) phaseAngles[1] += 360.0f;

    // Determine phase sequence
    if (phaseAngles[0] > 0 && phaseAngles[0] < 180)
    {
        strcpy(phaseSequence, "ABC");
    }
    else if (phaseAngles[0] > 180 && phaseAngles[0] < 360)
    {
        strcpy(phaseSequence, "ACB");
    }
    else
    {
        strcpy(phaseSequence, "Unknown");
    }
}

float32_t calculateFrequencyFromZCD(float32_t* signal, float32_t* firstZeroCrossingTime)
{
    // Find maximum amplitude to set dynamic thresholds
    float32_t maxAmplitude = 0.0f;
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
        float32_t absVal = fabsf(signal[i]);
        if (absVal > maxAmplitude)
        {
            maxAmplitude = absVal;
        }
    }

    // Set hysteresis thresholds based on signal amplitude
    const float32_t upperThreshold = 0.7f * maxAmplitude;
    const float32_t lowerThreshold = 0.0f;

    uint8_t crossingDetected = 0;
    float32_t zeroCrossings[BUFFER_SIZE / 2]; // Maximum possible zero crossings
    uint32_t zeroCrossingCount = 0;

    for (uint32_t i = 1; i < BUFFER_SIZE; i++)
    {
        float32_t prevSample = signal[i - 1];
        float32_t currSample = signal[i];

        if (!crossingDetected)
        {
            // Detect positive-going zero crossing with hysteresis
            if (prevSample < lowerThreshold && currSample >= upperThreshold)
            {
                // Linear interpolation to estimate zero crossing
                float32_t fraction = (lowerThreshold - prevSample) / (currSample - prevSample);
                float32_t crossingIndex = (float32_t)(i - 1) + fraction;
                zeroCrossings[zeroCrossingCount++] = crossingIndex;
                crossingDetected = 1;
            }
        }
        else
        {
            // Wait until signal crosses below lowerThreshold to reset detection
            if (prevSample > upperThreshold && currSample <= lowerThreshold)
            {
                crossingDetected = 0;
            }
        }
    }

    // Ensure we have detected enough zero crossings
    if (zeroCrossingCount < 2)
    {
        // Not enough zero crossings detected
        return -1.0f; // Indicate error
    }

    // Save the first zero crossing time
    *firstZeroCrossingTime = zeroCrossings[0];

    // Calculate periods between zero crossings
    float32_t periods[BUFFER_SIZE / 2];
    uint32_t periodCount = 0;

    for (uint32_t i = 1; i < zeroCrossingCount; i++)
    {
        float32_t periodSamples = zeroCrossings[i] - zeroCrossings[i - 1];
        periods[periodCount++] = periodSamples;
    }

    // Compute average period
    float32_t totalPeriod = 0.0f;
    for (uint32_t i = 0; i < periodCount; i++)
    {
        totalPeriod += periods[i];
    }
    float32_t averagePeriodSamples = totalPeriod / (float32_t)periodCount;

    // Calculate frequency from zero crossings
    float32_t zeroCrossingFrequency = (float32_t)FS / averagePeriodSamples;

    return zeroCrossingFrequency;
} */

/* void processPhase(uint8_t phaseIndex, float32_t* frequency)
{
    char msg[100];

    // Extract data for the specified phase
    splitADCBuffer(phaseIndex);

    // Apply FIR filter
    arm_fir_init_f32(&S, NUM_TAPS, firCoeffs32, firStateF32, BUFFER_SIZE);
    arm_fir_f32(&S, inputF32, outputF32, BUFFER_SIZE);

    // Remove DC bias
    float32_t meanValue;
    arm_mean_f32(outputF32, BUFFER_SIZE, &meanValue);
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
        outputF32[i] -= meanValue;
    }

    // Zero-Crossing Detection
    float32_t zeroCrossings[BUFFER_SIZE]; // Store indices of zero crossings
    uint32_t zeroCrossingCount = 0;

    // Detect zero crossings
    for (uint32_t i = 1; i < BUFFER_SIZE; i++)
    {
        float32_t previousSample = outputF32[i - 1];
        float32_t currentSample = outputF32[i];

        // Check for zero crossing (sign change)
        if ((previousSample >= 0.0f && currentSample < 0.0f) ||
            (previousSample < 0.0f && currentSample >= 0.0f))
        {
            zeroCrossings[zeroCrossingCount++] = (float32_t)i;
        }
    }

    // Ensure we have detected enough zero crossings
    if (zeroCrossingCount < 3) // Need at least 3 zero crossings to calculate one full period
    {
        snprintf(msg, sizeof(msg), "Insufficient zero crossings detected for Phase %c.\r\n", 'A' + phaseIndex);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        *frequency = -1.0f; // Indicate error
        return;
    }

    // Calculate periods between zero crossings (full periods)
    float32_t periods[BUFFER_SIZE / 2];
    uint32_t periodCount = 0;

    for (uint32_t i = 2; i < zeroCrossingCount; i += 2)
    {
        float32_t periodSamples = zeroCrossings[i] - zeroCrossings[i - 2];
        periods[periodCount++] = periodSamples;
    }

    // Ensure we have at least one period
    if (periodCount == 0)
    {
        snprintf(msg, sizeof(msg), "No complete periods detected for Phase %c.\r\n", 'A' + phaseIndex);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        *frequency = -1.0f; // Indicate error
        return;
    }

    // Compute average period
    float32_t totalPeriod = 0.0f;
    for (uint32_t i = 0; i < periodCount; i++)
    {
        totalPeriod += periods[i];
    }
    float32_t averagePeriodSamples = totalPeriod / (float32_t)periodCount;

    // Calculate frequency from zero crossings
    *frequency = (float32_t)FS / averagePeriodSamples;
} */


/* void detect_zcd(){
    float32_t zeroCrossings[BUFFER_SIZE]; // Store indices of zero crossings
    uint32_t zeroCrossingCount = 0;
    char msg[50];
    // Detect zero crossings
    for (uint32_t i = 1; i < BUFFER_SIZE; i++)
    {
        float32_t previousSample = outputF32[i - 1];
        float32_t currentSample = outputF32[i];

        // Check for zero crossing (sign change)
        if ((previousSample >= 0.0f && currentSample < 0.0f) ||
            (previousSample < 0.0f && currentSample >= 0.0f))
        {
            zeroCrossings[zeroCrossingCount++] = (float32_t)i;
        }
    }

    // Ensure we have detected enough zero crossings
    if (zeroCrossingCount < 3) // Need at least 3 zero crossings to calculate one full period
    {
        snprintf(msg, sizeof(msg), "Insufficient zero crossings detected.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // Calculate periods between zero crossings (full periods)
    float32_t periods[BUFFER_SIZE / 2];
    uint32_t periodCount = 0;

    for (uint32_t i = 2; i < zeroCrossingCount; i += 2)
    {
        float32_t periodSamples = zeroCrossings[i] - zeroCrossings[i - 2];
        periods[periodCount++] = periodSamples;
    }

    // Ensure we have at least one period
    if (periodCount == 0)
    {
        snprintf(msg, sizeof(msg), "No complete periods detected.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // Compute average period
    float32_t totalPeriod = 0.0f;
    for (uint32_t i = 0; i < periodCount; i++)
    {
        totalPeriod += periods[i];
    }
    float32_t averagePeriodSamples = totalPeriod / (float32_t)periodCount;

    // Calculate frequency from zero crossings
    float32_t zeroCrossingFrequency = (float32_t)FS / averagePeriodSamples;

    // Send zero-crossing frequency over UART
    snprintf(msg, sizeof(msg), "Zero-Crossing Frequency: %.2f Hz\r\n", zeroCrossingFrequency);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
} */