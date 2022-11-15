// TO-do: funcion 

#define ROBOT_ARM1_LENGTH 91.61 //92.84  FISRT SEGMENT  
#define ROBOT_ARM2_LENGTH 105.92 //106.79 SECOND SEGMENT

#define MAXSPEED 90 //rpm
#define maxspeed 

// Inverse kinematic formulas for SCARA ROBOT
// This function returns C angle using law of cosines
float lawOfCosines(float a, float b, float c) 
{
  return acosf((a*a + b*b - c*c) / (2.0f * a * b));
}
// Euclidean distance between 2 points (0,0 to x,y)
float distance(float x, float y)
{
  return sqrt(x*x +y*y);
}

void InverseKinematic(float x, float y, float len1, float len2, uint8_t elbow, float *A1, float *A2)
{
  float dist;
  float D1,D2;

  if (elbow==1)  // inverse elbow solution: reverse X axis, and final angles.
    x = -x;
  dist = distance(x,y);
  if (dist > (ROBOT_ARM1_LENGTH+ROBOT_ARM2_LENGTH)){
    dist = (ROBOT_ARM1_LENGTH+ROBOT_ARM2_LENGTH)-0.001f;
  }
  D1 = atan2(y,x); 
  D2 = lawOfCosines(dist, len1, len2);   
  *A1 = (D1+D2+ROBOT_AXIS_DEFINITION)*RAD2GRAD;
  *A2 = lawOfCosines(len1,len2,dist)*RAD2GRAD-180; 
  if (elbow==1){
    *A1 = -*A1;
    *A2 = -*A2;
  }
}

