

class Particle
{
public:
    double *position;        // ����λ�� 
    double *velocity;        // ���ӵ��ٶ� 
    double fit;              // ��Ӧ�ȣ���Ӧ����ֵ 
    int dim;                 // ά��  
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
