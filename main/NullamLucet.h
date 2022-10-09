/*
*
*/

#include <inttypes.h>
#include <stdint.h>
#include <AbstractPlatform.h>

#ifndef __NULLAM_LUCET_DRIVER_H__
#define __NULLAM_LUCET_DRIVER_H__

/*
* Options object
*/
class LuxTurpisOpts {
  public:
    LuxTurpisOpts() {};
    LuxTurpisOpts(const LuxTurpisOpts* o) {};

  private:
};



class LuxTurpis {
  public:
    LuxTurpis(const LuxTurpisOpts* opts);
    ~LuxTurpis();

    void printDebug(StringBuilder*);


  protected:


  private:
    const LuxTurpisOpts _opts;
};


#endif   // __NULLAM_LUCET_DRIVER_H__
