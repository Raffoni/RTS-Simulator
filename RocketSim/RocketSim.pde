import Jama.Matrix;

PShape parafoil;

Vector pos = new Vector(0, 0, 0);

Vector vel_com = new Vector(0, 0, 0);
Vector vel_p = new Vector(0, 0, 0);
Vector vel_c = new Vector(0, 0, 0);

Vector acc = new Vector(0, 0, 0);

float rotX = 0;
float rotY = 0;
float rotZ = PI/4;

Vector angular_vel = new Vector(0, 0, 0);
Vector angular_accel = new Vector(0, 0, 0);

float GRAVITY = 0.01;

float PAYLOAD_MASS = 135;
float CANOPY_MASS = 13;
float SYSTEM_MASS = 0;

float PAYLOAD_SURFACE = 0.5;
float CANOPY_SURFACE = 21; 

float R1 = 0.5; //Distance between System CoM and payload CoM
float R2 = 7.5; // Distance between System CoM and Canopy CoM

float WING_LIFT_COEFFICIENT = 0.001;
float WING_DRAG_COEFFICIENT = 0.001;
float WING_A
float WING_B
float WING_C
float WING_T
float WING_AR

float CL_0 = 0.4;
float CL_ALPHA = 2;
float CD_0 = 0.15;
float CD_ALPHA = 1;

//Debug Constants
int WIREGRID_SIZE = 500; // The size of each cube of the wiregrid
int WIREGRID_DIST = 3; // The maximum number of cubes of the wiregrid in each direction

// Dynamic Matrices
Matrix MF = new Matrix(3, 3); // Apparent Masses
Matrix IF = new Matrix(3, 3); // Apparent Inertias
Matrix SYSTEM_A = new Matrix(6, 6);

class Vector {
  float x, y, z;
  Vector (float x, float y, float z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  float Dot(Vector v) {
    return (this.x * v.x) + (this.y * v.y) + (this.z * v.z);
  }
  
  Vector Cross(Vector v) {
    return new Vector((this.y * v.z) - (v.y * this.z),
                      (this.x * v.z) - (v.x * this.z),
                      (this.x * v.y) - (v.x * this.y));  
  }
  
  Vector Scale(float s) {
    return new Vector(this.x * s, this.y * s, this.z * s); 
  }
  
  Vector Add(Vector v) {
    float x = this.x + v.x;
    float y = this.y + v.y;
    float z = this.z + v.z;
    return new Vector(x, y, z);
  }
  
  float Magnitude() {
    return sqrt((this.x * this.x) + (this.y * this.y) + (this.z * this.z)); 
  }
  
  Vector Normalise() {
    float mag = this.Magnitude();
    return  new Vector(this.x / mag, this.y / mag, this.z / mag); 
  }
  
  Vector Project(Vector v) { // The projection of this vector onto the vector v
    float s = this.Dot(v.Normalise());
    return v.Scale(s);
  }
}

void setup() {
  size(800, 800, P3D);
  parafoil = loadShape("Parafoil.obj");
  pos.x = 0;
  pos.y = 0;
  
  SYSTEM_MASS = PAYLOAD_MASS + CANOPY_MASS;
  // mass matrix
  Matrix MASS_MATRIX = new Matrix(3, 3, SYSTEM_MASS);
  // apparent mass matrix
  double[][] MFvals = {  {0.848*QUARTER_PI*DENSITY*WING_T*WING_T*WING_B*(1+(8/3)*WING_A*WING_A*WING_A), 0., 0.},
                         {0., 0.339*QUARTER_PI*DENSITY*((WING_T*WING_T)+2*WING_A*WING_A*(1-WING_T*WING_T))*WING_C, 0.},
                         {0., 0., (WING_AR/(1+WING_AR))*QUARTER_PI*DENSITY*WING_C*WING_C*WING_B*sqrt(1+2*WING_A*WING_A*(1-WING_T*WING_T))}};
  MF = new Matrix(MFvals, 3, 3);
  
  Matrix INERTIA_MATRIX = new Matrix(3, 3, SYSTEM_INERTIA);
  double[][] IFvals = {  {0.055*(WING_AR/(1+WING_AR))*DENSITY*WING_C*WING_C*WING_B*WING_B*WING_B, 0, 0},
                         {0.0308*(WING_AR/(1+WING_AR))*DENSITY*pow(WING_C, 4)*WING_B*(1+(HALF_PI/6)*(1+WING_AR)*WING_AR*WING_A*WING_T*WING_T)},
                         {0.0555*DENSITY*WING_T*WING_T*pow(WING_B, 3)*(1+8*WING_A*WING_A)}};
  IF = new Matrix(IFvals, 3, 3);                       
  
  SYSTEM_A.setMatrix(new int[]{0, 1, 2}, new int[]{0, 1, 2}, MF.plus(MASS_MATRIX));
  SYSTEM_A.setMatrix(new int[]{0, 1, 2}, new int[]{3, 4, 5}, new Matrix(3, 3));
  // (The lower left submatrix is time dependant)
  SYSTEM_A.setMatrix(new int[]{3, 4, 5}, new int[]{3, 4, 5}, IF.plus(INERTIA_MATRIX));
}

void draw() {
  // ---- PARAFOIL DYNAMICS ----
  Vector up = angleToVector(rotX, rotY, rotZ + HALF_PI);
  double[][] RGPvals = {{0, -up.z, up.y},
                        {up.z, 0, -up.x},
                        {-up.y, up.x, 0}};
  Matrix RGP = new Matrix(RGPvals, 3, 3);
  SYSTEM_A.setMatrix(new int[]{3, 4, 5}, new int[]{0, 1, 2}, RGP.times(MF));
  
  double[][] WBvals = { {-sin(rotY)*(PAYLOAD_MASS + CANOPY_MASS)*GRAVITY},
                        {sin(rotX)*cos(rotY)*(PAYLOAD_MASS + CANOPY_MASS)*GRAVITY},
                        {cos(rotX)*cos(rotY)*(PAYLOAD_MASS + CANOPY_MASS)*GRAVITY}};
  Matrix WB = new Matrix(WBvals, 3, 1);
  
  Matrix Omega = new Matrix(new double[][] {{0, -angular_vel.z, angular_vel.y},
                                            {angular_vel.z, 0, -angular_vel.x},
                                            {-angular_vel.y, angular_vel.x, 0});
  
  vel_p = vel_com.Add(matrixTimesVector(Omega, up.Scale(-R1)));
  vel_c = vel_com.Add(matrixTimesVector(Omega, up.Scale(R2)));
  
  float angleOfAttack = atan(vel_c.z/vel_c.x);
  
  float payloadDragCoeff = CD_0+CD_ALPHA*pow(angleOfAttack, 2);
  Matrix PayloadAeroForce = new Matrix(new double[][] {{vel_p.x}, {vel_p.y}, {vel_p.z}}).arrayTimes(new Matrix(3, 1, -0.5*FLUID_DENSITY*PAYLOAD_SURFACE*vel_p.Magnitude()*payloadDragCoeff));
  
  float canopyDragCoeff = CD_0 + CD_ALPHA*pow(angleOfAttack, 2);
  float canopyLiftCoeff = CL_0 + CL_ALPHA*angleOfAttack;
  Matrix CanopyAeroForce = new Matrix(new double[][] {{vel_c.z}, {0}, {-vel_c.x}}).arrayTimes(new Matrix(3, 1, -0.5*FLUID_DENSITY*CANOPY_SURFACE*vel.Magnitude()*payloadDragCoeff))
  
  
  Matrix A = new Matrix(, 6, 6);
  Matrix X = A.solve(B)//Solve the system
  
  pos.x += vel.x;
  pos.y += vel.y;
  pos.z += vel.z;
  
  rotY += 0.01;
  rotZ += 0.01;
  
  
  // ---- RENDERING ----
  background(84, 152, 255); //Refresh Screen
  camera(pos.x, pos.y, pos.z + 800, pos.x, pos.y, pos.z, 0, 1, 0);
  
  // --- DEBUG ---
  strokeWeight(2); // Debug stroke weight
  
  text("rotX: " + rotX, 10, 10);
  text("rotY: " + rotY, 10, 20);
  text("rotZ: " + rotZ, 10, 30);
  text("posX: " + pos.x, 120, 10);
  text("posY: " + pos.y, 120, 20);
  text("posZ: " + pos.z, 120, 30);
  text("lift: ", 230, 10);
  
  // Draw wiregrid to give sense of velocity
  noFill();
  int rx = round(pos.x/WIREGRID_SIZE) * WIREGRID_SIZE;
  int ry = round(pos.y/WIREGRID_SIZE) * WIREGRID_SIZE;
  int rz = round(pos.z/WIREGRID_SIZE) * WIREGRID_SIZE;
  stroke(255, 0, 0);
  for (int i = -WIREGRID_DIST; i < WIREGRID_DIST; i++) {
    for (int j = -WIREGRID_DIST; j < WIREGRID_DIST; j++) {
      for (int k = -WIREGRID_DIST; k < WIREGRID_DIST; k++) {
        pushMatrix();
        translate(rx + (i * WIREGRID_SIZE) , ry + (j * WIREGRID_SIZE), rz + (k * WIREGRID_SIZE));
        box(WIREGRID_SIZE);
        popMatrix();
      }
    }
  }
  
  translate(pos.x, pos.y, pos.z);

  stroke(255, 0, 0);
  line(0, 0, 0, forwards.x, forwards.y, forwards.z);
  text("forwards", forwards.x, forwards.y, forwards.z);
  
  stroke(0, 255, 0);
  line(0, 0, 0, vel.x, vel.y, vel.z);
  text("velocity", vel.x, vel.y, vel.z);
  
  //stroke(0, 0, 255);
  //line(0, 0, 0, lift.x, lift.y, lift.z);
  //text("lift", lift.x, lift.y, lift.z);
  
  // --------------
  
  strokeWeight(0.1);
  scale(20);
  
  rotateX(rotX);
  rotateY(rotY);
  rotateZ(rotZ);
  
  shape(parafoil, 0, 0);
  
  stroke(0, 0, 0);
  line(0, 0, 0, -1, -5.2, 0);
  line(-2.5, 0, 0, -1, -5.2, 0);
  
  line(0, -0.2, -2.1, -1, -5.2, 0);
  line(-2.5, -0.2, -2.1, -1, -5.2, 0);
  
  line(0, -0.9, -3.6, -1, -5.2, 0);
  line(-2.5, -0.9, -3.6, -1, -5.2, 0);
  
  line(0, -1.4, -4.8, -1, -5.2, 0);
  line(-2.5, -1.4, -4.8, -1, -5.2, 0);
  
  line(0, -0.2, 2.1, -1, -5.2, 0);
  line(-2.5, -0.2, 2.1, -1, -5.2, 0);
  
  line(0, -0.9, 3.6, -1, -5.2, 0);
  line(-2.5, -0.9, 3.6, -1, -5.2, 0);
  
  line(0, -1.4, 4.8, -1, -5.2, 0);
  line(-2.5, -1.4, 4.8, -1, -5.2, 0);
  
  rotateX(-rotX);
  rotateY(-rotY);
  rotateZ(-rotZ);
}

Vector angleToVector(float rX, float rY, float rZ) {
  float x = cos(rZ)*cos(rY);
  float y = sin(rZ);
  float z = -sin(rY);
  return new Vector(x, y, z);
}

Vector matrixTimesVector(Matrix m, Vector v) { //Matrix must be 3*3
  float x = v.Dot(new Vector(m.get(0, 0), m.get(0, 1), m.get(0, 2)));
  float y = v.Dot(new Vector(m.get(1, 0), m.get(1, 1), m.get(1, 2)));
  float z = v.Dot(new Vector(m.get(2, 0), m.get(2, 1), m.get(2, 2)));
  return new Vector(x, y, z);
}
