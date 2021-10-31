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
class NullamLucetOpts {
  public:
    NullamLucetOpts() {};
    NullamLucetOpts(const NullamLucetOpts* o) {};

  private:
};



class NullamLucet {
  public:
    NullamLucet(const NullamLucetOpts* opts);
    ~NullamLucet();

    void printDebug(StringBuilder*);


  protected:


  private:
    const NullamLucetOpts _opts;
};


#endif   // __NULLAM_LUCET_DRIVER_H__
