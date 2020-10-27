#ifndef SENSORFUSION9_H_
#define SENSORFUSION9_H_

void sensfusion9Init(void);

void sensfusion9Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void sensfusion9GetQuaternion(double* qx, double* qy, double* qz, double* qw);
void sensfusion9GetEulerRPY(double* roll, double* pitch, double* yaw);

#endif /* SENSORFUSION9_H_ */
