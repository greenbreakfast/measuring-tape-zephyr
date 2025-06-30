#ifndef DISTANCE_SENSOR_H_
#define DISTANCE_SENSOR_H_

int distance_sensor_init(void);
int distance_sensor_get_distance_mm(int32_t *distance_mm);

#endif // DISTANCE_SENSOR_H_