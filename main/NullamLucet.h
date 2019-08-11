/*
*
*/

#include <inttypes.h>
#include <stdint.h>
#include <Platform/Platform.h>

#ifndef __NULLAM_LUCET_DRIVER_H__
#define __NULLAM_LUCET_DRIVER_H__

/*
* Options object
*/
class NullamLucetOpts {
  public:

  private:
};



class NullamLucet : public EventReceiver,
  #ifdef MANUVR_CONSOLE_SUPPORT
    public ConsoleInterface,
  #endif
    {
  public:
    NullamLucet(const NullamLucetOpts* opts);
    ~NullamLucet();

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    void printDebug(StringBuilder*);

    #ifdef MANUVR_CONSOLE_SUPPORT
      /* Overrides from ConsoleInterface */
      uint consoleGetCmds(ConsoleCommand**);
      inline const char* consoleName() { return "NullamLucet";  };
      void consoleCmdProc(StringBuilder* input);
    #endif  //MANUVR_CONSOLE_SUPPORT


  protected:
    int8_t attached();


  private:
    const NullamLucetOpts _opts;
};


#endif   // __NULLAM_LUCET_DRIVER_H__
