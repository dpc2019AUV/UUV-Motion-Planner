#ifndef DUBINS_H_
#define DUBINS_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#define EPSILON (10e-10)

#define EDUBOK        (0)   /* No error */
#define EDUBCOCONFIGS (1)   /* Colocated configurations */
#define EDUBPARAM     (2)   /* Path parameterisitation error */
#define EDUBBADRHO    (3)   /* the rho value is invalid */
#define EDUBNOPATH    (4)   /* no connection between configurations with this word */

using namespace std;

typedef enum 
{
    L_SEG = 0,
    S_SEG = 1,
    R_SEG = 2
} SegmentType;

/* The segment types for each of the Path types */
const SegmentType DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

typedef enum 
{
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
} DubinsPathType;

typedef struct 
{
    /* the initial configuration */
    double qi[3];        
    /* the lengths of the three segments */
    double param[3];     
    /* model forward velocity / model angular velocity */
    double rho;          
    /* the path type described */
    DubinsPathType type; 
} DubinsPath;

typedef struct 
{
    double alpha;
    double beta;
    double d;
    double sa;
    double sb;
    double ca;
    double cb;
    double c_ab;
    double d_sq;
} DubinsIntermediateResults;


class Dubins
{
    public:

        Dubins(double* q0, double* q1, double r);

        ~Dubins();

        //typedef int (*DubinsPathSamplingCallback)(double q[3], double t, void* user_data);

        //int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double rho);

        int dubins_path( double* q0, double* q1, double rho, DubinsPathType pathType);

        double dubins_path_length();

        void dubins_segment( double t, double* qi, double* qt, SegmentType type);

        double dubins_segment_length(int i);

        double dubins_segment_length_normalized( int i );

        DubinsPathType dubins_path_type();

        int dubins_path_sample(double t, double* q);

        int dubins_path_sample_many(double stepSize);

        int dubins_path_endpoint(double* q);

        int dubins_extract_subpath(double t, DubinsPath* newpath);

        int dubins_word(DubinsPathType pathType, double* out);

        int dubins_intermediate_results(double* q0, double* q1, double rho);

        double fmodr( double x, double y);

        double mod2pi( double theta );

        int dubins_LSL(  double* out );
        int dubins_RSR(  double* out );
        int dubins_LSR(  double* out );
        int dubins_RSL(  double* out );
        int dubins_LRL(  double* out );
        int dubins_RLR(  double* out );

        DubinsPath* path_;
        DubinsIntermediateResults* in_;
};  
#endif   // DUBINS_H



