#include "Generated/Time.h" // might change to brackets for include path

#include "AP_XRCE_Generic_Fn_T.h"

// Code generated table based on the enabled topics.
// Mavgen is using python, loops are not readable.
// Can use jinja to template (like Flask)


const struct Topic_table topics[] = {
    {
        .label = "my_qos_label__t",
        .serialize = static_cast<Generic_serialize_topic_fn_t>(&Time_serialize_topic),
        .deserialize = static_cast<Generic_deserialize_topic_fn_t>(&Time_deserialize_topic),
        .size_of = static_cast<Generic_size_of_topic_fn_t>(&Time_size_of_topic),
    },
    {
        // TODO
    }
};