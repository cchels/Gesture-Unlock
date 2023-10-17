// // Import acc.h
#include <acc.h>
#include <math.h>



float &AccelerometerSample::operator[](uint8_t index)
{
    switch (index)
    {
    case 0:
        return this->x;
    case 1:
        return this->y;
    case 2:
        return this->z;
    default:
        return this->x;
    }
}

AccelerometerBuffer::AccelerometerBuffer()
{
    // Initialize head and tail of circular buffer
    this->reset();
    this->max_size = MAX_DATA_BUFFER_SIZE;
}

void AccelerometerBuffer::sample_data()
{
    // Read from accelerometer, add to moving_avg_buffer, and calculate moving average if reached the window size
    AccelerometerSample sample;
    sample.x = CircuitPlayground.motionX();
    sample.y = CircuitPlayground.motionY();
    sample.z = CircuitPlayground.motionZ();

    // Add sample to moving average buffer
    this->moving_avg_buffer[this->moving_avg_window_index++] = sample;

    // Calculate moving average and add it to the data buffer if we reached the max size
    if (this->moving_avg_window_index == MOVING_AVERAGE_WINDOW_SIZE)
    {
        AccelerometerSample moving_avg = this->calculate_moving_avg();
        this->push_back(moving_avg);
        this->moving_avg_window_index = 0;
    }
}

void AccelerometerBuffer::sample_data(AccelerometerBuffer &to_compare)
{
    float old_dtw;
    // Read from accelerometer, add to moving_avg_buffer, and calculate moving average if reached the window size
    AccelerometerSample sample;
    sample.x = CircuitPlayground.motionX();
    sample.y = CircuitPlayground.motionY();
    sample.z = CircuitPlayground.motionZ();

    // Add sample to moving average buffer
    this->moving_avg_buffer[this->moving_avg_window_index++] = sample;

    // Calculate moving average and add it to the data buffer if we reached the max size
    if (this->moving_avg_window_index == MOVING_AVERAGE_WINDOW_SIZE)
    {
        AccelerometerSample moving_avg = this->calculate_moving_avg();
        old_dtw = this->dtw_value;
        this->dtw_value = AccelerometerBuffer::dtw(*this, to_compare); // Serial.println("Pushing back");
        // Store the percentage decrease from the old dtw value
        if (old_dtw != -1)
        {
            this->dtw_percentage_decrease = (old_dtw - this->dtw_value) / old_dtw;
        }
        this->push_back(moving_avg);
        this->moving_avg_window_index = 0;
    }
}

void AccelerometerBuffer::push_back(AccelerometerSample sample)
{
    // Increment head if we reached the max size
    if (this->get_size() == (int)this->max_size) // if buffer full = size = 9 === max size = 9 = 8 push head forward
    {
        this->head = (this->head + 1) % this->max_size;
    }
    else
    {
        this->current_size++;
    }

    // Add element using tail
    this->data_buffer[this->tail] = sample;

    // Increment tail
    this->tail = (this->tail + 1) % this->max_size;
}

int AccelerometerBuffer::get_size() // Broken when overloaded goes back to zero
{
    return this->current_size;
}

AccelerometerSample AccelerometerBuffer::calculate_moving_avg()
{
    // Calculate the moving average of the buffer
    AccelerometerSample moving_avg;
    moving_avg.x = 0;
    moving_avg.y = 0;
    moving_avg.z = 0;

    for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++)
    {
        moving_avg.x += this->moving_avg_buffer[i].x;
        moving_avg.y += this->moving_avg_buffer[i].y;
        moving_avg.z += this->moving_avg_buffer[i].z;
    }

    moving_avg.x /= MOVING_AVERAGE_WINDOW_SIZE;
    moving_avg.y /= MOVING_AVERAGE_WINDOW_SIZE;
    moving_avg.z /= MOVING_AVERAGE_WINDOW_SIZE;
    Serial.print("Moving avg: ");
    Serial.println(moving_avg.x);
    return moving_avg;
}

AccelerometerSample &AccelerometerBuffer::get(uint8_t index)
{
    // Get the sample at the given index
    return this->data_buffer[(this->head + index) % this->max_size];
}

AccelerometerSample &AccelerometerBuffer::operator[](uint8_t index)
{
    return this->get(index);
}

void AccelerometerBuffer::reset()
{
    // Reset head and tail of circular buffer
    this->head = 0;
    this->tail = 0;
    this->moving_avg_window_index = 0;
    this->max_size = MAX_DATA_BUFFER_SIZE;
    this->dtw_value = -1;
}

void AccelerometerBuffer::set_max_size(uint8_t size)
{
    // Set the max size of the buffer
    this->max_size = size;
}

float p_norm(AccelerometerBuffer &a, AccelerometerBuffer &b, float p, int ai, int bi)
{
    float sum = 0.0;
    for (int i = 0; i < 3; i++)
    {
        sum += pow(fabs(a[ai][i] - b[bi][i]), p);
    }
    return pow(sum, 1.0 / p);
}

// DTW algorithm with space optimization using the sliding window technique
float AccelerometerBuffer::dtw(AccelerometerBuffer &a, AccelerometerBuffer &b)
{
    int n = a.get_size();
    int o = b.get_size(); 
    float p = 2; // Euclidian normal
    float d[o]; // 1-D array 
    float prev, tmp, min_val;
    d[0] = p_norm(a, b, p, 0, 0);
    for (int i = 1; i < o; i++) {
        d[i] = d[i-1] + p_norm(a, b, p, 0, i);
    }
    for (int i = 1; i < n; i++) {
        prev = d[0]; // reuse previous value
        d[0] += p_norm(a, b, p, i, 0);
        for (int j = 1; j < o; j++) {
            tmp = p_norm(a, b, p, i, j);
            min_val = fmin(prev, fmin(d[j-1], d[j])) + tmp;
            prev = d[j];
            d[j] = min_val;
        }
    }
    return d[o-1];
}
