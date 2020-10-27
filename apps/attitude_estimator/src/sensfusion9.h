#ifndef SENSORFUSION9_H_
#define SENSORFUSION9_H_

void sensfusion9Init(void);

void sensfusion9Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void sensfusion9GetQuaternion(float* q);
void sensfusion9GetEulerRPY(float* angles);

#endif /* SENSORFUSION9_H_ */
