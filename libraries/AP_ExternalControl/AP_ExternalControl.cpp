#include "AP_ExternalControl.h"

// singleton instance
AP_ExternalControl *AP_ExternalControl::singleton;

AP_ExternalControl::AP_ExternalControl()
{
    singleton = this;
}


namespace AP {

AP_ExternalControl *externalcontrol()
{
    return AP_ExternalControl::get_singleton();
}

};
