#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);

extern binary_semaphore_t image_process_ready_sem;

#endif /* PROCESS_IMAGE_H */
