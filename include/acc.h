
#pragma once
#include <Adafruit_CircuitPlayground.h>
// Accelerometer data
#define MAX_RECORD_TIME 3 // Max amount of time you can record for in seconds
#define LOOP_DELAY 50     // Loop delay in milliseconds

// Use loop delay to calculate the number of accelerometer data to record
#define MOVING_AVERAGE_WINDOW_SIZE 4 // Square moving average sampling size
#define MAX_DATA_BUFFER_SIZE (int)(1000 * MAX_RECORD_TIME / (LOOP_DELAY * MOVING_AVERAGE_WINDOW_SIZE))

// Accelerometer data structures
class AccelerometerSample
{
private:
    float x, y, z; // accelerometer x,y,z data
public:
    friend class AccelerometerBuffer;
    float &operator[](uint8_t index);
};

// Circular buffer of accelerometer data with a head and tail and an n point moving average
class AccelerometerBuffer
{
private:
    uint8_t head, tail;                                                // Tail is the index of the next element to be added and head points to the first(oldest element)
    AccelerometerSample data_buffer[MAX_DATA_BUFFER_SIZE];             // Actual data
    AccelerometerSample moving_avg_buffer[MOVING_AVERAGE_WINDOW_SIZE]; // Moving average data used to calculate moving average
    uint8_t moving_avg_window_index;
    uint8_t current_size;                       // CUrrnt buffer size
    uint8_t max_size;                           // Max size of the buffer
    void push_back(AccelerometerSample sample); // Add a sample to the circular data buffer
    AccelerometerSample calculate_moving_avg(); // Calculate the moving average of the buffer
    void add_to_average_buffer();

public:
    float dtw_value;
    float dtw_percentage_decrease;
    AccelerometerBuffer();

    void reset(); // Reset head and tail to 0

    // Setter
    void sample_data();                                // Read from accelerometer, add to moving_avg_buffer, and calculate moving average if reached the window size
    void sample_data(AccelerometerBuffer &to_compare); // Read from accelerometer, add to moving_avg_buffer, and calculate moving average if reached the window size
    void set_max_size(uint8_t size);                   // Set the max size of the buffer

    // Getter
    int get_size(); // Get the current of the buffer using head and tail
    AccelerometerSample &get(uint8_t index);

    // Override indexing operator
    AccelerometerSample &operator[](uint8_t index);
    static float dtw(AccelerometerBuffer &x, AccelerometerBuffer &y);
};