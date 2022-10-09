#include "LuxTurpis.h"


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/*
* Constructor.
*/
LuxTurpis::LuxTurpis(const LuxTurpisOpts* o) : _opts(o) {
}


/*
* Destructor.
*/
LuxTurpis::~LuxTurpis() {
}



/*
* Dump this item to the dev log.
*/
void LuxTurpis::printDebug(StringBuilder* output) {
  output->concat("\n==< LuxTurpis >======================\n");
}
