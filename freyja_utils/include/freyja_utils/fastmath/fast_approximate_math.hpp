/* Fast and approximate mathematical utilities.
Here are some faster implementations of oft-used functions. The speedup is
generally obtained by sacrificing a bit of accuracy (e.g., trig. functions),
and sometimes correctness (in 'IEEE standards' sense). Most common use-cases
for these are in simulations where a bit of noise is needed: for instance,
using these functions, (sin^2 + cos^2) will be "almost =1.0" but not really.

** WARNING: DO NOT USE these without significant testing in deployed code! **

Some functions may set the printer on fire.

  ~ aj. Nov 2017. 
*/

#include <cstdint>
#include <cstring>
namespace freyja_utils
{
  namespace fast_approx
  {
    constexpr double pi = 3.14159;
    constexpr double accg = 9.81;
    constexpr double fact3 = 3.0*2.0;
    constexpr double fact5 = 5.0*4.0*fact3;
    constexpr double fact7 = 7.0*6.0*fact5;
    constexpr inline double square(const double &x) { return x*x; }
    constexpr inline double cube(const double &x) { return x*x*x; }
    // fast sign function, returns {+1 or -1}, returns -1 for x==0
    #define posneg(x) ((x>0)*2-1)
    // fast and usually correct signum function
    #define signum(x) ((0<x) - (x<0))
    constexpr inline double abs( const double &x ) noexcept { return x > 0? x: -x; }
    constexpr inline int round(double x) noexcept
    { return static_cast<int>(x+0.5*posneg(x)); }
    constexpr inline double fmod( double x, double y ) noexcept
    {  return x - int(x/y)*y;  }
    constexpr inline double constrainAngleRad( double x ) noexcept
    {
      // limit angle to [-pi,pi]
      x = fast_approx::fmod(x+pi,2*pi);
      return( x < 0 ? x+pi : x-pi );
    }
    constexpr inline double sine(const double &a) noexcept
      { return a - cube(a)/fact3 + a*a*cube(a)/fact5 - a*cube(a)*cube(a)/fact7; }
    constexpr inline double cosine(const double &a) noexcept
      { return 1.0 - a*a/2.0 + a*cube(a)/(4*fact3) - cube(a)*cube(a)/(6*fact5); }
    
    constexpr inline double angleDiffRad(double a, double b) noexcept
      { return (pi - fast_approx::abs(fast_approx::abs(a - b) - pi)); }
  }

  namespace fast_checks
  {
    constexpr uint64_t inf_nan_double = uint64_t(0xffe0000000000000);
    static inline bool isnan(double x) noexcept 
    { 
      static_assert(sizeof(double)==sizeof(uint64_t), "Platform uses non-64-bit double types, NaN comparisons will fail.");
      uint64_t r;
      std::memcpy(&r, &x, sizeof(x));
      return (r << 1)  > inf_nan_double;
    }
  }
}