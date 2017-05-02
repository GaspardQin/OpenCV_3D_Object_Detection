

class Particle
{
public:
    double *position;        // 粒子位置 
    double *velocity;        // 粒子的速度 
    double fit;              // 适应度，对应函数值 
    int dim;                 // 维数  
public:   
    Particle(){};
    Particle( int d )
    {
        dim = d;
    }
    void setDim(int d)
    {
        dim = d;
    }
};
