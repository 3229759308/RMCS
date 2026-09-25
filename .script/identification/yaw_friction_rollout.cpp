#include <cmath>
#include <algorithm>
// Exact linear decay, implicit smooth friction. One initialization per segment.
extern "C" void rollout(int n, const double* t, const double* u, double initial,
                        const double* p, double eps, int substeps, double* output) {
    const double J=p[0], b=p[1], fc=p[2], bias=p[3];
    double w=initial; output[0]=w;
    for(int i=1;i<n;++i){
        const double dt=(t[i]-t[i-1])/substeps;
        const double a=std::exp(-b*dt/J);
        const double g=b>0 ? -std::expm1(-b*dt/J)/b : dt/J;
        for(int step=0;step<substeps;++step){
            const double rhs=a*w+g*(u[i-1]-bias), k=g*fc;
            double lo=rhs-k, hi=rhs+k;
            double x=std::clamp(w,lo,hi);
            for(int it=0;it<32;++it){
                const double z=std::tanh(x/eps), f=x+k*z-rhs;
                if(std::abs(f)<1e-11) break;
                if(f>0) hi=x; else lo=x;
                const double candidate=x-f/(1+k*(1-z*z)/eps);
                x=(candidate>lo && candidate<hi) ? candidate : (lo+hi)/2;
            }
            w=x;
        }
        output[i]=w;
    }
}
