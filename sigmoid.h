

/* Basic sigmoid call */
int sigmoid(float M, float time, float*result);

/* ms between steps  - too small isn't going to get scheduled */
#define MOVE_SIGMOID_LATENCY MOVE_LATENCY

#define SIGMOID_ERR	6.2126
