#include "path_planning/dubins.h"

Dubins::Dubins(double* q0, double* q1, double r)
{
    in_ = new DubinsIntermediateResults;
    path_ = new DubinsPath;
    int i, errcode;
    double params[3];
    double cost;
    double best_cost = INFINITY;
    int best_word = -1;
    errcode = dubins_intermediate_results(q0, q1, r);  

    path_->qi[0] = *q0;
    path_->qi[1] = *(q0 + 1);
    path_->qi[2] = *(q0 + 2);
    path_->rho = r;
 
    for( i = 0; i < 6; i++ ) 
    {
        DubinsPathType pathType = (DubinsPathType)i;
        errcode = dubins_word(pathType, params);
        if(errcode == EDUBOK) 
        {
            cost = params[0] + params[1] + params[2];
            if(cost < best_cost) 
            {
                best_word = i;
                best_cost = cost;
                path_->param[0] = params[0];
                path_->param[1] = params[1];
                path_->param[2] = params[2];
                path_->type = pathType;
            }
        }
    }
    // cout << "path_param = " << path_->param[0] << "; " << path_->param[1] <<
    //         "; " << path_->param[2] << endl;
}

Dubins::~Dubins()
{
    if (path_ != NULL)
        delete path_;
    if (in_ != NULL)
        delete in_;
}

double Dubins::fmodr( double x, double y)
{
    return x - y*floor(x/y);
}

double Dubins::mod2pi( double theta )
{
    return fmodr( theta, 2 * M_PI );
}

int Dubins::dubins_path(double q0[3], double q1[3], double rho, DubinsPathType pathType)
{
    int errcode;
    errcode = dubins_intermediate_results(q0, q1, rho);  
    if(errcode == EDUBOK) {
        double params[3];
        errcode = dubins_word(pathType, params);
        if(errcode == EDUBOK) {
            path_->param[0] = params[0];
            path_->param[1] = params[1];
            path_->param[2] = params[2];
            path_->qi[0] = q0[0];
            path_->qi[1] = q0[1];
            path_->qi[2] = q0[2];
            path_->rho = rho;
            path_->type = pathType;
        }
    }
    return errcode;
}

double Dubins::dubins_path_length()
{
    double length = 0.;
    length += path_->param[0];
    length += path_->param[1];
    length += path_->param[2];
    length = length * path_->rho;
    //cout << "length = " << length << endl;
    return length;
}

double Dubins::dubins_segment_length( int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INFINITY;
    }
    return path_->param[i] * path_->rho;
}

double Dubins::dubins_segment_length_normalized( int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INFINITY;
    }
    return path_->param[i];
} 

DubinsPathType Dubins::dubins_path_type() 
{
    return path_->type;
}

void Dubins::dubins_segment( double t, double qi[3], double qt[3], SegmentType type)
{
    double st = sin(qi[2]);
    double ct = cos(qi[2]);
    if( type == L_SEG ) {
        qt[0] = +sin(qi[2]+t) - st;
        qt[1] = -cos(qi[2]+t) + ct;
        qt[2] = t;
    }
    else if( type == R_SEG ) {
        qt[0] = -sin(qi[2]-t) + st;
        qt[1] = +cos(qi[2]-t) - ct;
        qt[2] = -t;
    }
    else if( type == S_SEG ) {
        qt[0] = ct * t;
        qt[1] = st * t;
        qt[2] = 0.0;
    }
    qt[0] += qi[0];
    qt[1] += qi[1];
    qt[2] += qi[2];
}

int Dubins::dubins_path_sample( double t, double q[3] )
{
    /* tprime is the normalised variant of the parameter t */
    double tprime = t / path_->rho;
    double qi[3]; /* The translated initial configuration */
    double q1[3]; /* end-of segment 1 */  
    double q2[3]; /* end-of segment 2 */  
    const SegmentType* types = DIRDATA[path_->type];
    double p1, p2;

    if( t < 0 || t > dubins_path_length() + 0.01 ) {
        return EDUBPARAM;
    }

    /* initial configuration */
    qi[0] = 0.0;
    qi[1] = 0.0;
    qi[2] = path_->qi[2];

    /* generate the target configuration */
    p1 = path_->param[0];
    p2 = path_->param[1];
    dubins_segment( p1, qi, q1, types[0] );  //end point of first segment
    dubins_segment( p2, q1, q2, types[1] );  //end point of second segment
    if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );
    }
    else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
    }

    /* scale the target configuration, translate back to the original starting point */
    q[0] = q[0] * path_->rho + path_->qi[0];
    q[1] = q[1] * path_->rho + path_->qi[1];
    q[2] = mod2pi(q[2]);

    return EDUBOK;
}

int Dubins::dubins_path_sample_many(double stepSize) 
{
    int retcode;
    double q[3];
    double x = 0.0;
    double length = dubins_path_length();   
    while( x <  length ) 
    {
        dubins_path_sample( x, q );  
        x += stepSize;
        cout << "q[] = " << q[0] << "  " << q[1] << "  " << q[2] << endl;
    }
    return 0;
}

int Dubins::dubins_path_endpoint( double q[3] )
{
    return dubins_path_sample( dubins_path_length() - EPSILON, q );
}

int Dubins::dubins_extract_subpath( double t, DubinsPath* newpath )
{
    /* calculate the true parameter */
    double tprime = t / path_->rho;

    if((t < 0) || (t > dubins_path_length()))
    {
        return EDUBPARAM; 
    }

    /* copy most of the data */
    newpath->qi[0] = path_->qi[0];
    newpath->qi[1] = path_->qi[1];
    newpath->qi[2] = path_->qi[2];
    newpath->rho   = path_->rho;
    newpath->type  = path_->type;

    /* fix the parameters */
    newpath->param[0] = fmin( path_->param[0], tprime );
    newpath->param[1] = fmin( path_->param[1], tprime - newpath->param[0]);
    newpath->param[2] = fmin( path_->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}

int Dubins::dubins_intermediate_results(double* q0, double* q1, double rho)
{
    double dx, dy, D, d, theta, alpha, beta;
    if( rho <= 0.0 ) {
        return EDUBBADRHO;
    }

    dx = *q1 - *q0;
    dy = *(q1 + 1) - *(q0 + 1);
    D = sqrt( dx * dx + dy * dy );
    d = D / rho;
    theta = 0;

    /* test required to prevent domain errors if dx=0 and dy=0 */
    if(d > 0) {
        theta = mod2pi(atan2( dy, dx ));
    }
    alpha = mod2pi(*(q0 + 2) - theta);
    beta  = mod2pi(*(q1 + 2) - theta);

    in_->alpha = alpha;
    in_->beta  = beta;
    in_->d     = d;
    in_->sa    = sin(alpha);
    in_->sb    = sin(beta);
    in_->ca    = cos(alpha);
    in_->cb    = cos(beta);
    in_->c_ab  = cos(alpha - beta);
    in_->d_sq  = d * d;

    return EDUBOK;
}

int Dubins::dubins_LSL(double out[3]) 
{
    double tmp0, tmp1, p_sq;
    
    tmp0 = in_->d + in_->sa - in_->sb;
    p_sq = 2 + in_->d_sq - (2*in_->c_ab) + (2 * in_->d * (in_->sa - in_->sb));

    if(p_sq >= 0) {
        tmp1 = atan2( (in_->cb - in_->ca), tmp0 );
        out[0] = mod2pi(tmp1 - in_->alpha);
        out[1] = sqrt(p_sq);
        out[2] = mod2pi(in_->beta - tmp1);
        return EDUBOK;
    }
    return EDUBNOPATH;
}


int Dubins::dubins_RSR(double out[3]) 
{
    double tmp0 = in_->d - in_->sa + in_->sb;
    double p_sq = 2 + in_->d_sq - (2 * in_->c_ab) + (2 * in_->d * (in_->sb - in_->sa));
    if( p_sq >= 0 ) {
        double tmp1 = atan2( (in_->ca - in_->cb), tmp0 );
        out[0] = mod2pi(in_->alpha - tmp1);
        out[1] = sqrt(p_sq);
        out[2] = mod2pi(tmp1 -in_->beta);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int Dubins::dubins_LSR(double out[3]) 
{
    double p_sq = -2 + (in_->d_sq) + (2 * in_->c_ab) + (2 * in_->d * (in_->sa + in_->sb));
    if( p_sq >= 0 ) {
        double p    = sqrt(p_sq);
        double tmp0 = atan2( (-in_->ca - in_->cb), (in_->d + in_->sa + in_->sb) ) - atan2(-2.0, p);
        out[0] = mod2pi(tmp0 - in_->alpha);
        out[1] = p;
        out[2] = mod2pi(tmp0 - mod2pi(in_->beta));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int Dubins::dubins_RSL(double out[3]) 
{
    double p_sq = -2 + in_->d_sq + (2 * in_->c_ab) - (2 * in_->d * (in_->sa + in_->sb));
    if( p_sq >= 0 ) {
        double p    = sqrt(p_sq);
        double tmp0 = atan2( (in_->ca + in_->cb), (in_->d - in_->sa - in_->sb) ) - atan2(2.0, p);
        out[0] = mod2pi(in_->alpha - tmp0);
        out[1] = p;
        out[2] = mod2pi(in_->beta - tmp0);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int Dubins::dubins_RLR(double out[3]) 
{
    double tmp0 = (6. - in_->d_sq + 2*in_->c_ab + 2*in_->d*(in_->sa - in_->sb)) / 8.;
    double phi  = atan2( in_->ca - in_->cb, in_->d - in_->sa + in_->sb );
    if( fabs(tmp0) <= 1) {
        double p = mod2pi((2*M_PI) - acos(tmp0) );
        double t = mod2pi(in_->alpha - phi + mod2pi(p/2.));
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(in_->alpha - in_->beta - t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int Dubins::dubins_LRL(double out[3])
{
    double tmp0 = (6. - in_->d_sq + 2*in_->c_ab + 2*in_->d*(in_->sb - in_->sa)) / 8.;
    double phi = atan2( in_->ca - in_->cb, in_->d + in_->sa - in_->sb );
    if( fabs(tmp0) <= 1) {
        double p = mod2pi( 2*M_PI - acos( tmp0) );
        double t = mod2pi(-in_->alpha - phi + p/2.);
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(mod2pi(in_->beta) - in_->alpha -t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int Dubins::dubins_word(DubinsPathType pathType, double out[3]) 
{
    int result;
    switch(pathType)
    {
    case LSL:
        result = dubins_LSL(out);
        break;
    case RSL:
        result = dubins_RSL(out);
        break;
    case LSR:
        result = dubins_LSR(out);
        break;
    case RSR:
        result = dubins_RSR(out);
        break;
    case LRL:
        result = dubins_LRL(out);
        break;
    case RLR:
        result = dubins_RLR(out);
        break;
    default:
        result = EDUBNOPATH;
    }
    return result;
}




